"""Run the C++ benchmark node against a running sim.launch.py (move_group owns the octomap).

ros2 launch pcp_benchmark benchmark.launch.py output_csv:=results/bench.csv max_queries:=10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pcp_moveit_config import build_moveit_config, octomap_params


def generate_launch_description():
    args = {
        "queries_file": "results/queries.json",
        "output_csv": "results/bench.csv",
        "commit": "unknown",
        "max_queries": "-1",
        "first_query": "0",
    }
    moveit_config = build_moveit_config()
    bench = Node(
        package="pcp_benchmark",
        executable="pcp_benchmark",
        name="pcp_benchmark",
        # Own namespace: MoveItCpp's planning-scene publisher must not feed back into move_group.
        namespace="pcp_benchmark",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            octomap_params(),
            PathJoinSubstitution([FindPackageShare("pcp_benchmark"), "config", "benchmark.yaml"]),
            {
                "use_sim_time": True,
                "bench.queries_file": LaunchConfiguration("queries_file"),
                "bench.output_csv": LaunchConfiguration("output_csv"),
                "bench.commit": ParameterValue(LaunchConfiguration("commit"), value_type=str),
                "bench.max_queries": ParameterValue(
                    LaunchConfiguration("max_queries"), value_type=int
                ),
                "bench.first_query": ParameterValue(
                    LaunchConfiguration("first_query"), value_type=int
                ),
            },
        ],
        remappings=[
            ("monitored_planning_scene", "/monitored_planning_scene"),
            ("joint_states", "/joint_states"),
        ],
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
        + [
            bench,
            RegisterEventHandler(
                OnProcessExit(target_action=bench, on_exit=[EmitEvent(event=Shutdown())])
            ),
        ]
    )
