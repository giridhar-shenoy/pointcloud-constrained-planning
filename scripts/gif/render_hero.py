#!/usr/bin/env python3
"""Render the hero GIF's frames in RViz on a virtual display (scripts/record_gifs.sh runs this
under xvfb-run after the simulation has stopped; no Gazebo needed).

    render_hero.py --urdf ur5e_rgbd.urdf --map map.npz --traj q5_none.json \
        --executed frames/q5_none/frames.json --rviz scripts/gif/hero.rviz --out-dir hero/

Everything shown is recorded data: the depth-camera point cloud and move_group's octomap saved
by capture_map.py, the planned path (tool0 positions by forward kinematics along the planned
joint path, densified to 0.01 rad), and the arm's joint positions recorded while that path was
executed in Gazebo (record_demo.py). Timeline, at --fps:
  1. points  the cloud move_group builds the map from (the robot's own points removed) appears,
             revealed in the camera's image-row order;
  2. map     the occupied voxels appear in the same row order (where the camera saw them), the
             points fade out. The real map fills in within one camera update; the gradual reveal
             only shows which points produced which voxels;
  3. path    the planned tool path is drawn from start to goal;
  4. motion  the arm moves through its recorded joint positions (sim time, uniformly resampled);
  5. hold.
Writes frame_NNNN.png (the RViz window) and frames.json (phase of every frame, playback rate).
"""

import argparse
import json
import math
import os
import pathlib
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from PIL import ImageGrab
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from urdf_fk import UrdfFk, densify
from visualization_msgs.msg import Marker, MarkerArray

DEPTH_HFOV = 1.2217  # rad, pcp_description rgbd_camera.xacro
DEPTH_SIZE = (320, 240)
PHASES = [("points", 2.4), ("map", 2.6), ("path", 1.6), ("motion", None), ("hold", 1.4)]


def depth_rows(fk, xyz):
    """Image row (0 = top) of world points in the depth camera (camera_link: x forward, z up;
    320 x 240 px), so points and voxels can be revealed in the order the camera scans them."""
    cam = fk.pose("camera_link", {})
    xc = (xyz - cam[:3, 3]) @ cam[:3, :3]
    f = (DEPTH_SIZE[0] / 2) / math.tan(DEPTH_HFOV / 2)
    return DEPTH_SIZE[1] / 2 - f * xc[:, 2] / xc[:, 0]


def render_urdf(urdf_text):
    """The URDF without the visual of its 5 m x 5 m ground_plane link (it would hide the grid)."""
    return re.sub(
        r'(<link name="ground_plane">.*?)<visual>.*?</visual>', r"\1", urdf_text, flags=re.S
    )


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def points(xyz):
    return [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in xyz]


def marker(mid, kind, pts, scale, rgba, colors=None):
    m = Marker()
    m.header.frame_id = "world"
    m.ns = "gif"
    m.id = mid
    m.type = kind
    m.pose.orientation.w = 1.0
    if not pts:
        m.action = Marker.DELETE
        return m
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color = color(*rgba)
    m.points = pts
    if colors is not None:
        m.colors = colors
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--traj", required=True, help="planned trajectory JSON (benchmark node)")
    ap.add_argument("--executed", required=True, help="frames.json of its Gazebo execution")
    ap.add_argument("--rviz", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--motion-s", type=float, default=5.0, help="GIF seconds for the motion")
    ap.add_argument("--settle", type=float, default=0.35, help="s between publish and grab")
    args = ap.parse_args()
    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    fk = UrdfFk(args.urdf)
    data = np.load(args.map)
    cloud = data["cloud_xyz"]
    voxels = data["voxels"]
    traj = json.loads(pathlib.Path(args.traj).read_text())
    executed = json.loads(pathlib.Path(args.executed).read_text())
    names = executed["joint_names"]
    if traj["joint_names"] != names:
        raise RuntimeError("joint order differs between the trajectory and the recording")

    # Reveal orders and colours.
    cloud_pts = points(cloud[np.argsort(depth_rows(fk, cloud), kind="stable")])
    vox_order = np.argsort(depth_rows(fk, voxels), kind="stable")
    vox = voxels[vox_order]
    zmin, zmax = float(voxels[:, 2].min()), float(voxels[:, 2].max())
    vox_colors = [
        color(0.86 + 0.12 * s, 0.42 + 0.36 * s, 0.16 + 0.22 * s)
        for s in ((vox[:, 2] - zmin) / max(zmax - zmin, 1e-9))
    ]
    vox_pts = points(vox)
    tool_path = np.array(
        [
            fk.pose("tool0", dict(zip(names, q, strict=True)))[:3, 3]
            for q in densify([p["q"] for p in traj["points"]])
        ]
    )
    seg = np.linalg.norm(np.diff(tool_path, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)]) / max(seg.sum(), 1e-9)
    frames = executed["frames"]
    stamps = np.array([f["stamp"] for f in frames])
    qs = np.array([f["q"] for f in frames])

    # Timeline: (phase, progress 0..1) per output frame.
    timeline = []
    for phase, dur in PHASES:
        n = max(1, round((args.motion_s if dur is None else dur) * args.fps))
        timeline += [(phase, (i + 1) / n) for i in range(n)]
    exec_s = float(stamps[-1] - stamps[0])
    traj_s = float(traj["points"][-1]["t"])

    params = out / "rsp.yaml"
    params.write_text(
        json.dumps(
            {
                "robot_state_publisher": {
                    "ros__parameters": {
                        "robot_description": render_urdf(pathlib.Path(args.urdf).read_text())
                    }
                }
            }
        )
    )
    procs = [
        subprocess.Popen(
            [
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "--params-file",
                str(params),
            ],
            stdout=subprocess.DEVNULL,
        ),
        subprocess.Popen(
            ["rviz2", "-d", args.rviz, "--fullscreen"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ),
    ]
    rclpy.init()
    node = rclpy.create_node("pcp_render_hero")
    js_pub = node.create_publisher(JointState, "/joint_states", 10)
    mk_pub = node.create_publisher(MarkerArray, "/gif/markers", 10)

    def publish(q, markers):
        js = JointState(name=names, position=[float(v) for v in q])
        js.header.stamp = node.get_clock().now().to_msg()
        js_pub.publish(js)
        mk_pub.publish(MarkerArray(markers=markers))

    def state(phase, prog):
        n_cloud, cloud_alpha, n_vox, path_frac, q = 0, 1.0, 0, 0.0, qs[0]
        done = [p for p, _ in PHASES]
        reached = done.index(phase)
        if reached >= 0:
            n_cloud = len(cloud_pts) if reached > 0 else int(prog * len(cloud_pts))
        if reached >= 1:
            n_vox = len(vox_pts) if reached > 1 else int(prog * len(vox_pts))
            cloud_alpha = 0.0 if reached > 1 else 1.0 - 0.85 * prog
        if reached >= 2:
            path_frac = 1.0 if reached > 2 else prog
        if reached >= 3:
            t = stamps[0] + (exec_s if reached > 3 else prog * exec_s)
            q = [np.interp(t, stamps, qs[:, j]) for j in range(qs.shape[1])]
        k = int(np.searchsorted(arc, path_frac, side="right")) if path_frac > 0 else 0
        goal_pts = points(tool_path[-1:]) if reached >= 2 else []
        return q, [
            marker(
                0,
                Marker.POINTS,
                cloud_pts[:n_cloud] if cloud_alpha > 0 else [],
                0.007,
                (0.55, 0.85, 1.0, cloud_alpha),
            ),
            marker(1, Marker.CUBE_LIST, vox_pts[:n_vox], 0.0185, (1, 1, 1, 1), vox_colors[:n_vox]),
            marker(
                2,
                Marker.LINE_STRIP,
                points(tool_path[:k]) if k >= 2 else [],
                0.014,
                (0.25, 1.0, 0.45, 1.0),
            ),
            marker(3, Marker.SPHERE_LIST, goal_pts, 0.04, (0.25, 1.0, 0.45, 1.0)),
        ]

    try:
        display = os.environ["DISPLAY"]
        # RViz needs the robot description, TF and a first render: publish the start state for a
        # while and wait until the window shows more than the background.
        t0 = time.monotonic()
        while True:
            publish(qs[0], state("points", 0.0)[1])
            rclpy.spin_once(node, timeout_sec=0.5)
            colors = ImageGrab.grab(xdisplay=display).getcolors(1 << 16)
            if time.monotonic() - t0 > 15 and (colors is None or len(colors) > 50):
                break
            if time.monotonic() - t0 > 120:
                raise RuntimeError("RViz did not render within 120 s")
        meta = []
        for i, (phase, prog) in enumerate(timeline):
            q, markers = state(phase, prog)
            publish(q, markers)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(args.settle)
            ImageGrab.grab(xdisplay=display).convert("RGB").save(
                out / f"frame_{i:04d}.png", compress_level=1
            )
            meta.append({"phase": phase, "progress": prog})
        (out / "frames.json").write_text(
            json.dumps(
                {
                    "fps": args.fps,
                    "frames": meta,
                    "motion_gif_s": args.motion_s,
                    "executed_sim_s": exec_s,
                    "planned_trajectory_s": traj_s,
                    "time_scale": executed.get("time_scale"),
                    "points_shown": len(cloud_pts),
                    "voxels_shown": len(vox_pts),
                    "query": traj["query_id"],
                }
            )
        )
        print(
            f"[render_hero] {len(timeline)} frames -> {out} ({len(cloud_pts)} points,"
            f" {len(vox_pts)} voxels; motion: {exec_s:.1f} s of sim time in {args.motion_s} s)",
            flush=True,
        )
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
