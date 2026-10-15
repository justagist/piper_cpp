# Piper CPP

[![ci](https://github.com/justagist/piper_cpp/actions/workflows/ci.yml/badge.svg)](https://github.com/justagist/piper_cpp/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](piper_cpp/CMakeLists.txt)

- [piper_cpp](piper_cpp) -- C++ SDK for the AgileX Piper 6-DOF arm with gripper. This is a standalone pure C++ library.
- [piper_cpp_ros](piper_cpp_ros) -- A ros2_control hardware interface for the AgileX Piper 6-DOF arm and its optional
parallel-jaw gripper, built on top of `piper_cpp`. Exposes position / velocity / effort state and position commands per
arm joint, plus a 7th prismatic gripper joint when enabled.
- [piper_cpp_moveit](piper_cpp_moveit) -- MoveIt 2 configuration for the AgileX Piper 6-DOF arm. Plans on the arm
alone or, with `with_gripper:=true`, on the arm + gripper together.

[![piper_cpp.gif](https://media.githubusercontent.com/media/justagist/_assets/refs/heads/main/piper_cpp/piper_cpp.gif)](https://media.githubusercontent.com/media/justagist/_assets/refs/heads/main/piper_cpp/piper_cpp.gif)

## License

MIT — see [LICENSE](../LICENSE).