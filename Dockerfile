# ------------------------------------------------------------------------------
# Base image: ROS 2 Jazzy (desktop-full)
# ------------------------------------------------------------------------------
FROM osrf/ros:jazzy-desktop-full

# Avoid tz/locale prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------------------------
# System deps & ROS dev tooling
#  - build-essential, cmake: compile C++ packages
#  - python3-colcon-common-extensions: convenience for colcon
#  - python3-rosdep: dependency resolver
#  - can-utils, iproute2: helpful for SocketCAN (candump, ip link set can0 up, etc.)
# ------------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-venv \
      can-utils \
      iproute2 \
      libsocketcan-dev \
      ethtool \
      && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Initialize rosdep (safe if re-run)
# ------------------------------------------------------------------------------
# RUN rosdep init 2>/dev/null || true && \
#     rosdep update

# ------------------------------------------------------------------------------
# Create a colcon workspace
# ------------------------------------------------------------------------------
WORKDIR /ws
RUN mkdir -p src

# Copy your package into the workspace.
# If your repo is already a single ROS 2 package, this is fine.
# If it’s a multi-pkg repo, copy the whole tree or adjust as needed.
COPY . /ws/src/piper_cpp/

# ------------------------------------------------------------------------------
# Resolve package dependencies via rosdep (from source, ignore already-installed)
# ------------------------------------------------------------------------------
# RUN . /opt/ros/jazzy/setup.sh && \
#     rosdep install --from-paths src --ignore-src -r -y

# ------------------------------------------------------------------------------
# Build with colcon
# ------------------------------------------------------------------------------
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select piper_cpp

# ------------------------------------------------------------------------------
# ROS entrypoint (source ROS + workspace on container start)
# ------------------------------------------------------------------------------
RUN printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -e' \
  'source /opt/ros/jazzy/setup.bash' \
  'if [ -f /ws/install/setup.bash ]; then' \
  '    source /ws/install/setup.bash' \
  'fi' \
  'exec "$@"' \
  > /ros_entrypoint.sh \
  && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
