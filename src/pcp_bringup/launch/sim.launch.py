"""Single-arm bring-up: UR5e + RGB-D camera in the shelf world (headless), the camera bridge,
and move_group building an octomap from the camera point cloud.

    ros2 launch pcp_bringup sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    move_group_delay = LaunchConfiguration("move_group_delay")

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "description_file": PathJoinSubstitution(
                [FindPackageShare("pcp_description"), "urdf", "ur5e_rgbd.urdf.xacro"]
            ),
            # ur_sim_control builds gz_args as " -s -r -v 4 " + world_file, so the headless
            # rendering flag (EGL, no X server; needed for the depth camera) rides on world_file.
            "world_file": [world, " --headless-rendering"],
            "gazebo_gui": "false",
            "launch_rviz": "false",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("pcp_bringup"), "config", "bridges.yaml"]
                ),
                "use_sim_time": True,
            }
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pcp_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("pcp_bringup"), "worlds", "shelf.sdf"]
                ),
            ),
            DeclareLaunchArgument(
                "move_group_delay",
                default_value="5.0",
                description="Seconds to wait before starting move_group (robot spawn first).",
            ),
            ur_sim,
            camera_bridge,
            TimerAction(period=move_group_delay, actions=[move_group]),
        ]
    )
