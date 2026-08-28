# ROS 2 Kilted + Gazebo Sim (gz-sim 9 / Ionic) teaching image.
#
# To move to another distro, change BOTH the tag and the ROS_DISTRO arg --
# the apt package names below are distro-qualified.
FROM ros:kilted

ARG ROS_DISTRO=kilted
ENV ROS_DISTRO=${ROS_DISTRO}

# Avoid tzdata and friends stopping the build to ask questions.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- simulation: Gazebo Sim and the ROS <-> gz bridge ---
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-gz-ros2-control \
    # --- ros2_control ---
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-pid-controller \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-kinematics \
    ros-${ROS_DISTRO}-pinocchio \
    python3-scipy \
    # --- robot descriptions used by the mobile manipulator ---
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-simulation-gz \
    ros-${ROS_DISTRO}-ur-controllers \
    ros-${ROS_DISTRO}-robotiq-description \
    # --- navigation and mapping ---
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-twist-mux \
    # --- teleoperation ---
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    # --- visualisation and introspection ---
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    # --- build tooling and GL ---
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    mesa-utils \
    x11-apps \
    terminator \
 && rm -rf /var/lib/apt/lists/*

# Match the host user id so files created in the bind-mounted workspace are not
# owned by an unexpected uid. Override with --build-arg on a host that is not 1000.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && if ! id -u ${USERNAME} >/dev/null 2>&1; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
    fi \
 && apt-get update && apt-get install -y --no-install-recommends sudo \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && rm -rf /var/lib/apt/lists/*

COPY .devcontainer/setup-ros.sh /usr/local/bin/setup-ros.sh
RUN chmod +x /usr/local/bin/setup-ros.sh

USER ${USERNAME}
CMD ["/bin/bash"]
