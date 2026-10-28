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
            # ur_sim_control's spawner loads the controller without --param-file; with
            # ros2_control 6.10 / gz_ros2_control 3.0.9 the controller then starts with no
            # `joints` and fails to initialize. Keep it inactive there and spawn it below.
            "initial_joint_controller": "joint_trajectory_controller",
            "activate_joint_controller": "false",
        }.items(),
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
    )
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_jtc_with_params",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
        output="screen",
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
        launch_arguments={
            "use_sim_time": "true",
            "octomap_resolution": LaunchConfiguration("octomap_resolution"),
        }.items(),
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
                "octomap_resolution",
                default_value="0.02",
                description="Octomap voxel edge (m); 0.02 is the documented default.",
            ),
            DeclareLaunchArgument(
                "move_group_delay",
                default_value="5.0",
                description="Seconds to wait before starting move_group (robot spawn first).",
            ),
            ur_sim,
            camera_bridge,
            TimerAction(period=4.0, actions=[jtc_spawner]),
            TimerAction(period=move_group_delay, actions=[move_group]),
        ]
    )
