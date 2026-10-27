#!/usr/bin/env python3
"""Smoke check against a running sim.launch.py (orchestration only; planning happens in move_group).

1. Wait until /monitored_planning_scene carries a non-empty world.octomap.
2. Ask move_group (/plan_kinematic_path) for one unconstrained RRTConnect plan from the arm's
   current state to a tool-down configuration inside the lower-left shelf compartment.
3. Ask for a plan to a configuration whose tool sits inside the middle shelf board. The board is
   only known to MoveIt through the octomap, so this must be rejected; if it is accepted the
   octomap is not part of collision checking and the smoke test fails.
4. Exit 0 only if all three checks pass.
"""

import argparse
import sys
import time

import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import GetMotionPlan

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Tool pointing down at roughly (0.54, 0.25, 0.24) m, inside the lower-left compartment.
GOAL = [0.2, -1.2, 1.8, -2.1708, -1.5708, 0.0]
# Tool at roughly (0.49, 0.13, 0.49) m, i.e. inside the middle board (z 0.48-0.50).
GOAL_IN_BOARD = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


def plan_to(node, client, joint_goal, planning_time):
    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name = "ur_manipulator"
    mpr.pipeline_id = "ompl"
    mpr.planner_id = "RRTConnectkConfigDefault"
    mpr.num_planning_attempts = 1
    mpr.allowed_planning_time = planning_time
    mpr.start_state.is_diff = True  # plan from the current (simulated) state
    goal = Constraints()
    for name, value in zip(JOINTS, joint_goal, strict=True):
        goal.joint_constraints.append(
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=1e-3,
                tolerance_below=1e-3,
                weight=1.0,
            )
        )
    mpr.goal_constraints.append(goal)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=planning_time + 30.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("planning service call timed out")
    res = future.result().motion_plan_response
    return res.error_code.val == MoveItErrorCodes.SUCCESS, res


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--octomap-timeout", type=float, default=150.0)
    parser.add_argument("--planning-time", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("pcp_smoke_check")
    latest = {}

    def on_scene(msg: PlanningScene) -> None:
        if len(msg.world.octomap.octomap.data) > 0:
            latest["scene"] = msg

    node.create_subscription(PlanningScene, "/monitored_planning_scene", on_scene, 10)
    t0 = time.monotonic()
    while "scene" not in latest and time.monotonic() - t0 < args.octomap_timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    if "scene" not in latest:
        print(f"[smoke] FAIL: no octomap on /monitored_planning_scene in {args.octomap_timeout} s")
        return 1
    octomap = latest["scene"].world.octomap
    print(
        f"[smoke] octomap OK: {len(octomap.octomap.data)} bytes, resolution "
        f"{octomap.octomap.resolution} m, frame '{octomap.header.frame_id}', "
        f"after {time.monotonic() - t0:.1f} s"
    )

    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=60.0):
        print("[smoke] FAIL: /plan_kinematic_path not available")
        return 1
    ok_free, res = plan_to(node, client, GOAL, args.planning_time)
    print(
        f"[smoke] plan to free goal {'OK' if ok_free else 'FAIL'}: "
        f"error_code {res.error_code.val}, "
        f"planning_time {res.planning_time:.3f} s, "
        f"{len(res.trajectory.joint_trajectory.points)} waypoints"
    )
    ok_board, res = plan_to(node, client, GOAL_IN_BOARD, args.planning_time)
    print(
        "[smoke] plan into the octomap board "
        f"{'REJECTED (OK)' if not ok_board else 'ACCEPTED (FAIL)'}: "
        f"error_code {res.error_code.val}"
    )
    ok = ok_free and not ok_board
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
