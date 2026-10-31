#!/usr/bin/env python3
"""Record frames from a demo camera in Gazebo, with the arm's joint positions for every frame.

Orchestration only (Python): planning was done by the C++ nodes. Two modes:

Replay (keep-upright GIF): ask move_group to bring the arm to the start of each saved trajectory
(JSON from the benchmark node), execute it through /execute_trajectory and store the frames of a
dedicated RGB camera (spawned here, so normal runs never pay for rendering it).

    record_demo.py --traj q12_none.json --traj q12_orientation.json --out-dir frames/

  An execution that MoveIt reports as CONTROL_FAILED (the trajectory controller aborted because a
  joint lagged its reference by more than the 0.2 rad path tolerance) is retried from the start,
  up to --attempts times in all; every attempt is logged. Any other error code fails at once.

Live (direct-vs-MoveIt GIF): record while a command runs (scripts/record_gifs.sh runs
pcp_direct's bench_direct on one query, which plans and executes it with both pipelines).

    record_demo.py --out-dir frames/ --during ros2 launch pcp_direct bench_direct.launch.py ...

Each recording directory gets frame_NNNN.png and frames.json: the camera model, every frame's
simulation time stamp and the joint positions interpolated from /joint_states at that stamp (so
overlays such as the tool's tilt are computed from recorded joint states, not typed), and, in
live mode, the whole /joint_states log.
"""

import argparse
import json
import pathlib
import queue
import shutil
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan
from PIL import Image as PilImage
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Demo camera: in front of and left of the robot, looking into the shelf compartments.
CAMERA = {
    "width": 480,
    "height": 360,
    "horizontal_fov": 0.95,
    # Gazebo camera frame: looks along +x, image up is +z. SDF pose, rpy = Rz(Y) Ry(P) Rx(R).
    "pose_xyz_rpy": [-0.55, -0.95, 1.05, 0.0, 0.43, 0.76],
}
CAMERA_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="demo_camera">
    <static>true</static>
    <link name="link">
      <sensor name="demo_camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{fps}</update_rate>
        <topic>/demo_camera/image</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>10</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""
CODE_NAMES = {
    getattr(MoveItErrorCodes, k): k
    for k in dir(MoveItErrorCodes)
    if k.isupper() and isinstance(getattr(MoveItErrorCodes, k), int)
}


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder:
    """Demo-camera frames (written to disk by a thread) and /joint_states, both sim-stamped."""

    def __init__(self, node):
        self.lock = threading.Lock()
        self.have_frame = False
        self.recording = False
        self.out = None
        self.keep_every = 1
        self.seen = 0
        self.stamps = []
        self.joint_log = []  # [t, q in JOINTS order]
        self.queue = queue.Queue()
        threading.Thread(target=self.writer, daemon=True).start()
        node.create_subscription(Image, "/demo_camera/image", self.on_image, 10)
        node.create_subscription(JointState, "/joint_states", self.on_joints, 100)

    def writer(self):
        while True:
            path, img = self.queue.get()
            PilImage.fromarray(img).save(path, compress_level=1)
            self.queue.task_done()

    def on_image(self, msg):
        with self.lock:
            self.have_frame = True
            if not self.recording:
                return
            self.seen += 1
            if (self.seen - 1) % self.keep_every:
                return
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            path = self.out / f"frame_{len(self.stamps):04d}.png"
            self.stamps.append(stamp_s(msg.header.stamp))
            self.queue.put((path, img[:, :, :3].copy()))

    def on_joints(self, msg):
        if not set(JOINTS) <= set(msg.name):
            return
        index = {n: i for i, n in enumerate(msg.name)}
        row = [stamp_s(msg.header.stamp)] + [msg.position[index[n]] for n in JOINTS]
        with self.lock:
            if self.recording:
                self.joint_log.append(row)

    def start(self, out, keep_every=1):
        if out.exists():
            shutil.rmtree(out)
        out.mkdir(parents=True)
        with self.lock:
            self.out = out
            self.keep_every = keep_every
            self.seen = 0
            self.stamps = []
            self.joint_log = []
            self.recording = True

    def stop(self):
        with self.lock:
            self.recording = False
            stamps, log = self.stamps, self.joint_log
        self.queue.join()
        return stamps, log


def frame_joints(stamps, log):
    """Joint positions at each frame stamp, linearly interpolated from the /joint_states log."""
    if len(log) < 2:
        raise RuntimeError("fewer than two /joint_states messages while recording")
    arr = np.array(sorted(log))
    return [
        [float(np.interp(t, arr[:, 0], arr[:, 1 + j])) for j in range(len(JOINTS))] for t in stamps
    ]


def save_meta(out, stamps, log, extra, with_log=False):
    meta = {
        "camera": CAMERA,
        "joint_names": JOINTS,
        "frames": [
            {"file": f"frame_{i:04d}.png", "stamp": t, "q": q}
            for i, (t, q) in enumerate(zip(stamps, frame_joints(stamps, log), strict=True))
        ],
        **extra,
    }
    if with_log:
        meta["joint_states"] = sorted(log)
    (out / "frames.json").write_text(json.dumps(meta))


def joint_goal(names, values):
    goal = Constraints()
    for n, v in zip(names, values, strict=True):
        goal.joint_constraints.append(
            JointConstraint(
                joint_name=n, position=v, tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0
            )
        )
    return goal


def wait(future, timeout):
    t0 = time.monotonic()
    while not future.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.05)
    return future.done()


def execute(node, client, traj):
    goal = ExecuteTrajectory.Goal(trajectory=traj)
    fut = client.send_goal_async(goal)
    if not wait(fut, 30.0) or not fut.result().accepted:
        raise RuntimeError("execute_trajectory goal rejected")
    res = fut.result().get_result_async()
    if not wait(res, 180.0):
        raise RuntimeError("execute_trajectory timed out")
    return res.result().result.error_code.val


def move_to(node, plan_client, exec_client, names, q):
    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name = "ur_manipulator"
    mpr.pipeline_id = "ompl"
    mpr.planner_id = "RRTConnectkConfigDefault"
    mpr.allowed_planning_time = 10.0
    mpr.num_planning_attempts = 1
    mpr.max_velocity_scaling_factor = 0.5
    mpr.max_acceleration_scaling_factor = 0.5
    mpr.start_state.is_diff = True
    mpr.goal_constraints.append(joint_goal(names, q))
    for _ in range(5):
        fut = plan_client.call_async(req)
        if not wait(fut, 40.0):
            continue
        res = fut.result().motion_plan_response
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            code = execute(node, exec_client, res.trajectory)
            if code == MoveItErrorCodes.SUCCESS:
                return
    raise RuntimeError("could not move to the query start")


def to_robot_trajectory(data, time_scale):
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = data["joint_names"]
    for p in data["points"]:
        t = p["t"] * time_scale
        pt = JointTrajectoryPoint(positions=p["q"])
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        traj.joint_trajectory.points.append(pt)
    return traj


def replay(args, node, rec):
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    exec_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    if not plan_client.wait_for_service(timeout_sec=60) or not exec_client.wait_for_server(
        timeout_sec=60
    ):
        raise RuntimeError("move_group services not available")
    for path in args.traj:
        data = json.loads(pathlib.Path(path).read_text())
        if data["joint_names"] != JOINTS:
            raise RuntimeError(f"{path}: unexpected joint order {data['joint_names']}")
        out = pathlib.Path(args.out_dir) / pathlib.Path(path).stem
        for attempt in range(1, args.attempts + 1):
            move_to(node, plan_client, exec_client, data["joint_names"], data["points"][0]["q"])
            time.sleep(1.0)
            rec.start(out, args.keep_every)
            code = execute(node, exec_client, to_robot_trajectory(data, args.time_scale))
            time.sleep(args.hold)
            stamps, log = rec.stop()
            print(
                f"[record_demo] {path}: attempt {attempt}/{args.attempts}: execution code {code}"
                f" ({CODE_NAMES.get(code, '?')}), {len(stamps)} frames -> {out}",
                flush=True,
            )
            if code == MoveItErrorCodes.SUCCESS:
                break
            if code != MoveItErrorCodes.CONTROL_FAILED:
                raise RuntimeError(f"execution of {path} failed with code {code}")
            if attempt == args.attempts:
                raise RuntimeError(
                    f"execution of {path} failed with CONTROL_FAILED in all {args.attempts}"
                    " attempts"
                )
            print(
                "[record_demo] CONTROL_FAILED: the controller aborted the execution; moving back"
                " to the start and executing the same trajectory again",
                flush=True,
            )
        save_meta(
            out,
            stamps,
            log,
            {
                "trajectory": str(path),
                "time_scale": args.time_scale,
                "attempts": attempt,
                "execution_code": code,
            },
        )


def live(args, node, rec):
    out = pathlib.Path(args.out_dir)
    rec.start(out, args.keep_every)
    print(f"[record_demo] recording while running: {' '.join(args.during)}", flush=True)
    rc = subprocess.run(args.during, check=False).returncode
    time.sleep(args.hold)
    stamps, log = rec.stop()
    print(f"[record_demo] command exited {rc}; {len(stamps)} frames -> {out}", flush=True)
    save_meta(out, stamps, log, {"command": args.during, "command_exit": rc}, with_log=True)
    return rc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", action="append", help="replay: trajectory JSON (repeatable)")
    ap.add_argument("--during", nargs=argparse.REMAINDER, help="live: command to record (last)")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--time-scale", type=float, default=2.0, help="slow playback by this factor")
    ap.add_argument("--fps", type=int, default=10, help="demo camera rate (sim time)")
    ap.add_argument("--keep-every", type=int, default=1, help="store every n-th camera frame")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds recorded after the motion")
    ap.add_argument("--attempts", type=int, default=3, help="replay: tries per CONTROL_FAILED")
    args = ap.parse_args()
    if bool(args.traj) == bool(args.during):
        ap.error("give either --traj (replay) or --during (live)")

    sdf = CAMERA_SDF.format(
        fps=args.fps, hfov=CAMERA["horizontal_fov"], w=CAMERA["width"], h=CAMERA["height"]
    )
    # `create` overrides the model's <pose> with its -x/-y/-z/-R/-P/-Y arguments (default 0),
    # so the pose goes on the command line. A second recording in the same simulation reuses the
    # camera already there (create then fails, harmlessly).
    keys = ["-x", "-y", "-z", "-R", "-P", "-Y"]
    created = subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create", "-string", sdf, "-name", "demo_camera"]
        + [str(v) for kv in zip(keys, CAMERA["pose_xyz_rpy"], strict=True) for v in kv],
        check=False,
        timeout=60,
    )
    print(f"[record_demo] spawn demo_camera: exit {created.returncode}", flush=True)
    bridge = subprocess.Popen(
        [
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "/demo_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ]
    )
    rclpy.init()
    node = rclpy.create_node("pcp_record_demo")
    rec = Recorder(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    rc = 0
    try:
        t0 = time.monotonic()
        while not rec.have_frame and time.monotonic() - t0 < 60:
            time.sleep(0.2)
        if not rec.have_frame:
            raise RuntimeError("no frames from /demo_camera/image")
        if args.traj:
            replay(args, node, rec)
        else:
            rc = live(args, node, rec)
    finally:
        bridge.terminate()
        bridge.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
