"""Direct pipeline vs MoveIt 2 on the same queries (pcp_direct bench_direct).

Plan only (no Gazebo; CI mini-benchmark):
    ros2 launch pcp_direct bench_direct.launch.py output_csv:=$PWD/results/tmp/d5.csv max_queries:=5
With execution, against a running pcp_bringup sim.launch.py (scripts/bench_direct.sh):
    ros2 launch pcp_direct bench_direct.launch.py execute:=true ...
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pcp_moveit_config import build_moveit_config

NAMESPACE = "pcp_direct"
# Parameters MoveItCpp only needs to execute (trajectory execution manager + controller list).
EXECUTION_KEYS = ("moveit_controller_manager", "moveit_simple_controller_manager")


def share(pkg, *parts):
    return PathJoinSubstitution([FindPackageShare(pkg), *parts])


def launch_setup(context):
    execute = LaunchConfiguration("execute").perform(context).lower() == "true"
    params = build_moveit_config().to_dict()
    # No sensors: MoveItCpp's monitor would otherwise build its own octomap from the camera.
    params = {k: v for k, v in params.items() if k not in ("sensors", "camera_points")}
    if not execute:
        # Without a controller the execution manager would wait for one at start-up.
        params = {k: v for k, v in params.items() if k not in EXECUTION_KEYS}
    lc = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    bench = Node(
        package="pcp_direct",
        executable="bench_direct",
        name="bench_direct",
        namespace=NAMESPACE,
        output="screen",
        # debug shows OMPL's "Path simplification took ..." lines (MoveIt routes OMPL's log here).
        ros_arguments=[
            "--log-level",
            f"{NAMESPACE}.bench_direct.moveit.planners.ompl:={lc['ompl_log_level']}",
        ],
        parameters=[
            params,
            share("pcp_direct", "config", "bench_direct.yaml"),
            {
                "use_sim_time": execute,
                "bench.execute": execute,
                "bench.queries_file": lc["queries_file"],
                "bench.output_csv": lc["output_csv"],
                "bench.commit": lc["commit"],
                "bench.max_queries": int(lc["max_queries"]),
                "bench.first_query": int(lc["first_query"]),
                "bench.probe_queries": int(lc["probe_queries"]),
                "bench.constraint": lc["constraint"],
                "bench.shelf_yaml": share("pcp_scenes", "config", "shelf.yaml"),
                "bench.ompl_yaml": share("pcp_moveit_config", "config", "ompl_planning.yaml"),
                "bench.joint_limits_yaml": share("pcp_direct", "config", "joint_limits.yaml"),
            },
        ],
        remappings=[
            # MoveIt's current-state monitor and the direct executor read the sim's joints.
            ("joint_states", "/joint_states"),
            # MoveIt's simple controller manager resolves its action client relative to the
            # node namespace; point it at the controller the sim already runs.
            (
                "joint_trajectory_controller/follow_joint_trajectory",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ),
        ],
    )
    loader = Node(
        package="pcp_direct",
        executable="shelf_boxes_loader",
        name="shelf_boxes_loader",
        namespace=NAMESPACE,
        output="screen",
        parameters=[{"shelf_yaml": share("pcp_scenes", "config", "shelf.yaml")}],
    )
    return [
        bench,
        loader,
        RegisterEventHandler(
            OnProcessExit(target_action=bench, on_exit=[EmitEvent(event=Shutdown())])
        ),
    ]


ARGS = {
    "queries_file": "results/queries_boxes.json",
    "output_csv": "results/bench_direct.csv",
    "commit": "unknown",
    "max_queries": "-1",
    "first_query": "0",
    "execute": "false",
    "ompl_log_level": "info",
    # >0: before the benchmark, solve that many queries in MoveIt's own planning context with a
    # counting validity checker, and the same queries with the direct planner (meta file only).
    "probe_queries": "0",
    # none | orientation (SHOULD: v0.1's keep-upright constraint in both pipelines)
    "constraint": "none",
}


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)]
    )
