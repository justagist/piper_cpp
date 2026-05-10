# ------------------------------------------------------------------------------
# Base image: ROS 2 Jazzy (desktop-full)
# ------------------------------------------------------------------------------
FROM osrf/ros:jazzy-desktop-full

# Avoid tz/locale prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

# ------------------------------------------------------------------------------
# System deps & ROS dev tooling
# ------------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      sudo \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-venv \
      can-utils \
      iproute2 \
      libsocketcan-dev \
      ethtool \
      ros-jazzy-hardware-interface \
      ros-jazzy-controller-manager \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-trajectory-controller \
      ros-jazzy-forward-command-controller \
      ros-jazzy-position-controllers \
      ros-jazzy-moveit-ros-move-group \
      ros-jazzy-moveit-kinematics \
      ros-jazzy-moveit-planners-ompl \
      ros-jazzy-moveit-simple-controller-manager \
      ros-jazzy-moveit-ros-visualization \
      ros-jazzy-moveit-configs-utils \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Create or rename non-root user with passwordless sudo
#
# Some base images already have UID 1000. If so, rename that user to USERNAME
# instead of trying to create a duplicate UID.
# ------------------------------------------------------------------------------
RUN set -eux; \
    USER_UID="${USER_UID:-1000}"; \
    USER_GID="${USER_GID:-1000}"; \
    \
    EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)"; \
    EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1 || true)"; \
    \
    if [ -n "${EXISTING_USER}" ] && [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
        usermod -l "${USERNAME}" "${EXISTING_USER}"; \
        usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
    fi; \
    \
    if [ -n "${EXISTING_GROUP}" ] && [ "${EXISTING_GROUP}" != "${USERNAME}" ]; then \
        groupmod -n "${USERNAME}" "${EXISTING_GROUP}"; \
    elif [ -z "${EXISTING_GROUP}" ]; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi; \
    \
    if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
        useradd \
          --uid "${USER_UID}" \
          --gid "${USER_GID}" \
          --create-home \
          --shell /bin/bash \
          "${USERNAME}"; \
    fi; \
    \
    usermod -aG sudo "${USERNAME}"; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

# ------------------------------------------------------------------------------
# Workspace path
# ------------------------------------------------------------------------------
ENV WS_DIR=/home/${USERNAME}/ws

WORKDIR ${WS_DIR}

RUN mkdir -p ${WS_DIR}/src && \
    chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"

# ------------------------------------------------------------------------------
# Copy your package into the workspace.
# During normal devcontainer use, this is replaced by the bind mount.
# ------------------------------------------------------------------------------
COPY . ${WS_DIR}/src/piper_cpp/

# ------------------------------------------------------------------------------
# Build with colcon
# ------------------------------------------------------------------------------
RUN . /opt/ros/jazzy/setup.sh && \
    cd ${WS_DIR} && \
    colcon build --symlink-install --packages-select piper_cpp piper_cpp_ros piper_cpp_moveit piper_description

# ------------------------------------------------------------------------------
# Make workspace writable by the dev user
# ------------------------------------------------------------------------------
RUN chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"

# ------------------------------------------------------------------------------
# ROS entrypoint: source ROS + workspace on container start
# ------------------------------------------------------------------------------
RUN printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -e' \
  'source /opt/ros/jazzy/setup.bash' \
  'if [ -f "$HOME/ws/install/setup.bash" ]; then' \
  '    source "$HOME/ws/install/setup.bash"' \
  'fi' \
  'exec "$@"' \
  > /ros_entrypoint.sh \
  && chmod +x /ros_entrypoint.sh

USER ${USERNAME}

WORKDIR /home/${USERNAME}/ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]