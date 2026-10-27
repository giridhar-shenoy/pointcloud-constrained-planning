"""Shared MoveIt configuration for move_group, the benchmark node and the tests."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def _share(pkg: str) -> Path:
    return Path(get_package_share_directory(pkg))


def build_moveit_config(ur_type: str = "ur5e"):
    """MoveItConfigs for the UR5e + camera description with this package's planner settings."""
    controllers = _share("ur_simulation_gz") / "config" / "ur_controllers.yaml"
    return (
        MoveItConfigsBuilder(robot_name="ur", package_name="pcp_moveit_config")
        .robot_description(
            file_path=str(_share("pcp_description") / "urdf" / "ur5e_rgbd.urdf.xacro"),
            mappings={"ur_type": ur_type, "name": "ur", "simulation_controllers": str(controllers)},
        )
        .robot_description_semantic(
            file_path=str(_share("ur_moveit_config") / "srdf" / "ur.srdf.xacro"),
            mappings={"name": "ur"},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path=str(_share("ur_moveit_config") / "config" / "joint_limits.yaml"))
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )


def octomap_params() -> dict:
    with open(_share("pcp_moveit_config") / "config" / "octomap.yaml") as f:
        return yaml.safe_load(f)
