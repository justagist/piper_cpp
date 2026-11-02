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

## Supported protocol versions

The CAN protocol follows the official [`piper_sdk`](https://github.com/agilexrobotics/piper_sdk)
protocol definitions at commit
[`b847e539`](https://github.com/agilexrobotics/piper_sdk/tree/b847e5393a7ce6ff8508d713eca7bdd3a42421b4).
The protocol is chosen by the interface class you construct, never detected from the arm.

| Protocol | Firmware (per `piper_sdk`) | Upstream definition | `piper_cpp` class | `piper_cpp_ros` |
| --- | --- | --- | --- | --- |
| V2 (default) | V1.5-2 up to, not including, V1.8-8 | [`INTERFACE_V2.MD`](https://github.com/agilexrobotics/piper_sdk/blob/b847e5393a7ce6ff8508d713eca7bdd3a42421b4/assets/V2/INTERFACE_V2.MD) | `PiperInterfaceV2` | `protocol_version:=2` |
| V3 | V1.8-8 and later | [`INTERFACE_V3.MD`](https://github.com/agilexrobotics/piper_sdk/blob/b847e5393a7ce6ff8508d713eca7bdd3a42421b4/assets/V3/INTERFACE_V3.MD), [`CHANGELOG-V3.MD`](https://github.com/agilexrobotics/piper_sdk/blob/b847e5393a7ce6ff8508d713eca7bdd3a42421b4/assets/V3/CHANGELOG-V3.MD) | `PiperInterfaceV3` | `protocol_version:=3` |

`INTERFACE_V2.MD` itself only says "V1.5-2 and later"; the upper bound comes from the
`piper_sdk` README, which says to use the V3 interface from firmware V1.8-8. On that
firmware V2 misreads at least MOVE M and the MIT commands.

Firmware older than V1.5-2 (the `piper_sdk` 0.0.x line) is not supported. Some V2 features
need newer firmware than V1.5-2 (for example MOVE CPV needs V1.8-1); the method docs say
where. V3 is tested against the Python SDK's encoder and decoder only, not on a physical arm. See
[Protocol V3](piper_cpp/README.md#protocol-v3) for what changes.

## License

MIT -- see [LICENSE](LICENSE).
