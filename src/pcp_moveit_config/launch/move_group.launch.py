"""move_group with the octomap updater fed by the simulated RGB-D camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from pcp_moveit_config import build_moveit_config, octomap_params


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_config = build_moveit_config()
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            octomap_params(),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "publish_robot_description_semantic": True,
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            move_group,
        ]
    )
