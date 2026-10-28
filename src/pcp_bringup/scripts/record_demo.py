#!/usr/bin/env python3
"""Replay saved benchmark trajectories in Gazebo and record frames from a demo camera.

Orchestration only (Python): planning was done by the C++ benchmark node, which saved the
trajectories as JSON; this script asks move_group to bring the arm to the query's start, executes
each saved trajectory through /execute_trajectory, and stores the frames of a dedicated RGB camera
(spawned here, so normal runs never pay for rendering it).

    record_demo.py --traj q12_none.json --traj q12_orientation.json --out-dir frames/
"""

import argparse
import json
import pathlib
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
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectoryPoint

# Demo camera: in front of and left of the robot, looking into the shelf compartments.
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
          <horizontal_fov>0.95</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>10</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


class Recorder:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.latest = None
        self.frames = []
        self.recording = False
        node.create_subscription(Image, "/demo_camera/image", self.on_image, 10)

    def on_image(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        with self.lock:
            self.latest = img.copy()
            if self.recording:
                self.frames.append(self.latest)


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
    if not wait(res, 120.0):
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", action="append", required=True, help="trajectory JSON (repeatable)")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--time-scale", type=float, default=2.0, help="slow playback by this factor")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--hold", type=float, default=1.0, help="seconds recorded after the motion")
    args = ap.parse_args()

    sdf = CAMERA_SDF.format(fps=args.fps, w=480, h=360)
    # `create` overrides the model's <pose> with its -x/-y/-z/-R/-P/-Y arguments (default 0),
    # so the pose goes on the command line.
    pose = {"-x": -0.55, "-y": -0.95, "-z": 1.05, "-R": 0.0, "-P": 0.43, "-Y": 0.76}
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create", "-string", sdf, "-name", "demo_camera"]
        + [str(v) for kv in pose.items() for v in kv],
        check=True,
        timeout=60,
    )
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
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    exec_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if not plan_client.wait_for_service(timeout_sec=60) or not exec_client.wait_for_server(
            timeout_sec=60
        ):
            raise RuntimeError("move_group services not available")
        t0 = time.monotonic()
        while rec.latest is None and time.monotonic() - t0 < 60:
            time.sleep(0.2)
        if rec.latest is None:
            raise RuntimeError("no frames from /demo_camera/image")
        for path in args.traj:
            data = json.loads(pathlib.Path(path).read_text())
            names = data["joint_names"]
            move_to(node, plan_client, exec_client, names, data["points"][0]["q"])
            time.sleep(1.0)
            with rec.lock:
                rec.frames = []
                rec.recording = True
            code = execute(node, exec_client, to_robot_trajectory(data, args.time_scale))
            time.sleep(args.hold)
            with rec.lock:
                rec.recording = False
                frames = rec.frames
            out = pathlib.Path(args.out_dir) / pathlib.Path(path).stem
            out.mkdir(parents=True, exist_ok=True)
            for i, f in enumerate(frames):
                PilImage.fromarray(f).save(out / f"frame_{i:04d}.png")
            print(f"[record_demo] {path}: execution code {code}, {len(frames)} frames -> {out}")
            if code != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"execution of {path} failed with code {code}")
    finally:
        bridge.terminate()
        bridge.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
