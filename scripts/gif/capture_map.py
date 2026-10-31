#!/usr/bin/env python3
"""Save what the hero GIF shows from a running simulation: one depth-camera point cloud and
move_group's octomap, both in the world frame (scripts/record_gifs.sh runs this first).

    capture_map.py --urdf ur5e_rgbd.urdf --out map.npz

Waits until move_group's octomap has stopped changing (same occupied-voxel count on three
polls a second apart, as the benchmark node waits for a settled map), then stores, world frame:
  cloud_xyz   the points move_group inserts into the octomap: /camera/points_filtered, i.e. the
              camera cloud without the robot's own links (padded 3 cm; the URDF's ground plane
              is one of them, so the floor goes too) and beyond the 2.0 m max_range
  raw_xyz     the raw /camera/points cloud within 2.0 m, for reference
  voxels      centres of the occupied 2 cm voxels of the octomap (pruned leaves expanded)
The octomap message arrives in octomap's full serialisation, decoded here (no octomap Python
bindings on the system).
"""

import argparse
import struct
import sys
import time

import numpy as np
import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from urdf_fk import UrdfFk

TREE_DEPTH = 16  # octomap's fixed key depth
MAX_RANGE = 2.0  # m, sensors_3d.yaml max_range


def occupied_voxels(data, resolution):
    """Decode octomap's full format (per node: float log-odds, uint8 child mask, children
    depth-first; child bit 0/1/2 = +x/+y/+z half) into finest-level voxel centres."""
    pos = 0
    keys = []

    def node(kx, ky, kz, depth):
        nonlocal pos
        (logodds,) = struct.unpack_from("<f", data, pos)
        children = data[pos + 4]
        pos += 5
        if children == 0:
            if logodds > 0.0:  # occupancy threshold 0.5 (log-odds 0), octomap's default
                keys.append((kx, ky, kz, TREE_DEPTH - depth))
            return
        half = 1 << (TREE_DEPTH - depth - 1)
        for i in range(8):
            if children & (1 << i):
                node(
                    kx + (half if i & 1 else 0),
                    ky + (half if i & 2 else 0),
                    kz + (half if i & 4 else 0),
                    depth + 1,
                )

    if data:
        node(0, 0, 0, 0)
    if pos != len(data):
        raise RuntimeError(f"octomap decode consumed {pos} of {len(data)} bytes")
    centres = []
    for kx, ky, kz, level in keys:  # a pruned leaf at `level` covers 2^level keys per axis
        n = 1 << level
        r = np.arange(n)
        gx, gy, gz = np.meshgrid(kx + r, ky + r, kz + r, indexing="ij")
        k = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        centres.append((k - (1 << (TREE_DEPTH - 1)) + 0.5) * resolution)
    return np.concatenate(centres) if centres else np.zeros((0, 3))


def get_octomap(node, client):
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.OCTOMAP
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    if not fut.done() or fut.result() is None:
        raise RuntimeError("/get_planning_scene did not answer")
    om = fut.result().scene.world.octomap
    return om, bytes(np.asarray(om.octomap.data, dtype=np.int8).tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    fk = UrdfFk(args.urdf)

    rclpy.init()
    node = rclpy.create_node("pcp_capture_map")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    clouds = {"/camera/points": [], "/camera/points_filtered": []}
    for topic, store in clouds.items():
        # Best effort: move_group publishes the filtered cloud best-effort (sensor data QoS).
        node.create_subscription(PointCloud2, topic, store.append, qos_profile_sensor_data)
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise RuntimeError("/get_planning_scene not available")
        t0 = time.monotonic()
        counts = []
        while True:
            om, data = get_octomap(node, client)
            if data and (om.octomap.binary or om.octomap.id != "OcTree"):
                raise RuntimeError(f"octomap format {om.octomap.id}, binary {om.octomap.binary}")
            n = len(occupied_voxels(data, om.octomap.resolution)) if data else 0
            counts.append(n)
            if n > 0 and len(counts) >= 3 and len(set(counts[-3:])) == 1:
                break
            if time.monotonic() - t0 > args.timeout:
                raise RuntimeError("the octomap did not settle")
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        voxels = occupied_voxels(data, om.octomap.resolution)
        frame = om.header.frame_id
        for store in clouds.values():
            store.clear()
        while not all(clouds.values()) and time.monotonic() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not all(clouds.values()):
            raise RuntimeError(f"no message on {[t for t, v in clouds.items() if not v]}")
        world = {}
        for topic, store in clouds.items():
            msg = store[0]
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts = pts[np.linalg.norm(pts, axis=1) <= MAX_RANGE]
            cam = fk.pose(msg.header.frame_id, {})
            world[topic] = pts @ cam[:3, :3].T + cam[:3, 3]
        xyz, raw = world["/camera/points_filtered"], world["/camera/points"]
        if len(xyz) == 0:
            raise RuntimeError("/camera/points_filtered is empty")
        np.savez_compressed(
            args.out,
            cloud_xyz=xyz.astype(np.float32),
            raw_xyz=raw.astype(np.float32),
            voxels=voxels.astype(np.float32),
            resolution=np.array(om.octomap.resolution),
        )
        print(
            f"[capture_map] octomap ({frame}): {len(voxels)} occupied voxels at"
            f" {om.octomap.resolution} m, settled after {len(counts)} polls; camera cloud:"
            f" {len(raw)} points within {MAX_RANGE} m, {len(xyz)} after removing the robot"
            f" -> {args.out}",
            flush=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
