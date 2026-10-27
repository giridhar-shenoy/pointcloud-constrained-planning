#!/usr/bin/env bash
# Expand the UR5e + camera URDF and the UR SRDF to plain files.
#   make_robot_files.sh <out.urdf> <out.srdf>
# Same xacros and arguments as pcp_moveit_config.build_moveit_config().
set -euo pipefail
urdf_out="$1"
srdf_out="$2"
desc="$(ros2 pkg prefix pcp_description 2>/dev/null)/share/pcp_description" || true
if [ ! -d "${desc}" ]; then
  # During `colcon build` the overlay may not be sourced yet; fall back to the source tree.
  desc="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../pcp_description" && pwd)"
fi
controllers="$(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/config/ur_controllers.yaml"
xacro "${desc}/urdf/ur5e_rgbd.urdf.xacro" ur_type:=ur5e name:=ur \
  simulation_controllers:="${controllers}" >"${urdf_out}"
xacro "$(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/srdf/ur.srdf.xacro" name:=ur \
  >"${srdf_out}"
