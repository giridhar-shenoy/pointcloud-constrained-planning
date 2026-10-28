"""Collision agreement test: MoveIt's planning scene vs the direct Pinocchio/coal checker.

No Gazebo. MoveItCpp (no 3-D sensors, so no octomap) and the shelf box loader run in their own
namespace, so a move_group running elsewhere is neither read nor modified.

    ros2 launch pcp_direct agreement.launch.py \
        output_csv:=$PWD/results/collision_agreement.csv count:=500
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pcp_moveit_config import build_moveit_config

NAMESPACE = "pcp_direct_agreement"


def moveit_params_without_sensors():
    """MoveIt parameters for an in-process MoveItCpp with no octomap updater."""
    params = build_moveit_config().to_dict()
    return {k: v for k, v in params.items() if k not in ("sensors", "camera_points")}


def generate_launch_description():
    shelf_yaml = PathJoinSubstitution([FindPackageShare("pcp_scenes"), "config", "shelf.yaml"])
    agreement = Node(
        package="pcp_direct",
        executable="collision_agreement",
        name="collision_agreement",
        namespace=NAMESPACE,
        output="screen",
        parameters=[
            moveit_params_without_sensors(),
            {
                # MoveItCpp loads its pipelines from this list (as in pcp_benchmark's YAML).
                "planning_pipelines.pipeline_names": ["ompl"],
                "agree.shelf_yaml": shelf_yaml,
                "agree.output_csv": LaunchConfiguration("output_csv"),
                "agree.count": ParameterValue(LaunchConfiguration("count"), value_type=int),
                "agree.seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                "agree.commit": ParameterValue(LaunchConfiguration("commit"), value_type=str),
            },
        ],
    )
    loader = Node(
        package="pcp_direct",
        executable="shelf_boxes_loader",
        name="shelf_boxes_loader",
        namespace=NAMESPACE,
        output="screen",
        parameters=[{"shelf_yaml": shelf_yaml}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_csv", default_value="results/collision_agreement.csv"),
            DeclareLaunchArgument("count", default_value="500"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("commit", default_value="unknown"),
            agreement,
            loader,
            RegisterEventHandler(
                OnProcessExit(target_action=agreement, on_exit=[EmitEvent(event=Shutdown())])
            ),
        ]
    )
