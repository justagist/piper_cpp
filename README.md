# Piper CPP

- [piper_cpp](piper_cpp) -- C++ SDK for the AgileX Piper 6-DOF arm with gripper. This is a standalone pure C++ library.
- [piper_cpp_ros](piper_cpp_ros) -- A ros2_control hardware interface for the AgileX Piper 6-DOF arm and its optional
parallel-jaw gripper, built on top of `piper_cpp`. Exposes position / velocity / effort state and position commands per
arm joint, plus a 7th prismatic gripper joint when enabled.
- [piper_cpp_moveit](piper_cpp_moveit) -- MoveIt 2 configuration for the AgileX Piper 6-DOF arm. Plans on the arm
alone or, with `with_gripper:=true`, on the arm + gripper together.

[![piper_cpp.gif](https://media.githubusercontent.com/media/justagist/_assets/refs/heads/main/piper_cpp/piper_cpp.gif)](https://media.githubusercontent.com/media/justagist/_assets/refs/heads/main/piper_cpp/piper_cpp.gif)

## Install the C++ library via apt (Ubuntu 24.04, amd64)

The standalone (non-ROS) `piper_cpp` library is published as Debian packages:

```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://justagist.github.io/piper_cpp/piper-cpp-archive-keyring.gpg \
  -o /etc/apt/keyrings/piper-cpp-archive-keyring.gpg
echo "deb [signed-by=/etc/apt/keyrings/piper-cpp-archive-keyring.gpg] https://justagist.github.io/piper_cpp noble main" \
  | sudo tee /etc/apt/sources.list.d/piper-cpp.list
sudo apt update
sudo apt install libpiper-cpp-dev   # headers + static lib + CMake config; `piper-cpp-tools` for the CLI utilities
```

## License

MIT - see [LICENSE](../LICENSE).
