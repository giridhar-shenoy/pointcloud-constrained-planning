# Reproducible environment for reviewers on any distro (Jazzy, Humble, macOS, ...).
# Base tag verified on Docker Hub via the registry v2 API on 2026-10-28.
FROM ros:lyrical-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# ROS 2 Lyrical binaries the workspace needs (MoveIt 2.15, UR Gazebo sim, ros_gz bridge; v0.2:
# Pinocchio 4 + coal + OMPL for the direct pipeline),
# Mesa EGL for headless ogre2 rendering of the depth camera, and the Python libs used by
# scripts/aggregate.py.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-lyrical-moveit-ros-planning \
      ros-lyrical-moveit-ros-planning-interface \
      ros-lyrical-moveit-ros-move-group \
      ros-lyrical-moveit-ros-perception \
      ros-lyrical-moveit-planners-ompl \
      ros-lyrical-moveit-planners-chomp \
      ros-lyrical-moveit-planners-stomp \
      ros-lyrical-moveit-kinematics \
      ros-lyrical-moveit-configs-utils \
      ros-lyrical-moveit-simple-controller-manager \
      ros-lyrical-ur-description \
      ros-lyrical-ur-moveit-config \
      ros-lyrical-ur-simulation-gz \
      ros-lyrical-gz-ros2-control \
      ros-lyrical-ros-gz-bridge \
      ros-lyrical-ros-gz-sim \
      ros-lyrical-ros-gz-interfaces \
      ros-lyrical-xacro \
      ros-lyrical-ament-cmake-gtest \
      ros-lyrical-pinocchio \
      ros-lyrical-coal \
      ros-lyrical-ompl \
      ros-lyrical-control-msgs \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-matplotlib \
      python3-numpy \
      python3-scipy \
      python3-yaml \
      python3-pytest \
      python3-pil \
      nlohmann-json3-dev \
      libyaml-cpp-dev \
      libegl1 \
      libegl-mesa0 \
      libgl1-mesa-dri \
      ffmpeg \
      git \
      git-lfs \
      procps \
      sudo \
    && rm -rf /var/lib/apt/lists/*

# Non-root user for the devcontainer. Ubuntu 24.04+ images ship a default `ubuntu` user on
# UID 1000; remove it so the dev user can take that UID and match a typical host user.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /usr/local/bin/pcp-entrypoint.sh
RUN chmod +x /usr/local/bin/pcp-entrypoint.sh

USER ${USERNAME}
RUN rosdep update --rosdistro lyrical \
    && echo "source /opt/ros/lyrical/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo 'if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc

ENV ROS_DOMAIN_ID=12 \
    GZ_PARTITION=r2 \
    MAKEFLAGS=-j4
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/pcp-entrypoint.sh"]
CMD ["bash"]
