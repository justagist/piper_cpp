# piper_cpp

C++ SDK for the AgileX Piper 6-DOF arm with gripper, communicating over SocketCAN. A close
port of the official [`piper_sdk`](https://github.com/agilexrobotics/piper_sdk) Python package
with the same control surface and message coverage, but with several improvements such as
simpler and clearer APIs, better documentations, strongly-typed enums in place of raw protocol
bytes.

## Features

- Full message coverage of the v2 Piper protocol: arm status, joint feedback (high- and
  low-speed), end pose, gripper feedback, motor angle/speed/acc limits, end vel/acc, crash
  protection, gripper teaching pendant, instruction-response config, firmware version.
- Control APIs: enable/disable, motion-mode (granular per-field setters with TX-shadow
  merging), joint control, end-pose control, gripper control, MIT mode (torque + position
  mix control with Kp/Kd gains, in the style of the MIT mini-cheetah motor driver),
  emergency stop, reset, master/slave config, joint config, motor angle/speed/acc limits,
  end vel/acc limits, crash-protection levels, gripper teaching pendant params,
  master-arm home request.
- Strongly-typed enums for every protocol field that takes a non-obvious byte value.
- SIGINT-safe shutdown patterns demonstrated in every script (the gripper firmware will
  latch a fault if its command stream is cut mid-drive -- the demos send Disable frames on
  exit to prevent this).
- Forward kinematics with the manufacturer's DH offsets.
- Standalone CLI binaries for every common operation (enable, gripper sweep, joint sweep,
  go-to-zero, set-zero, set-role, read-firmware, reset, …) -- see [scripts/](scripts/).

## Requirements

- Linux with SocketCAN (any recent kernel).
- `libsocketcan-dev` and `pkg-config`.
- A C++17 compiler (`build-essential`).
- `cmake >= 3.10`.
- `can-utils` and `iproute2` are useful for bringing up the bus and sniffing frames
  (`candump`, `ip link`).

```bash
sudo apt install build-essential cmake pkg-config libsocketcan-dev can-utils iproute2
```

## Installation

### Building inside a ROS workspace (with colcon)

If you have a ROS environment available, drop this package into the `src/` directory of a
colcon workspace and build it like any other package. `piper_cpp` itself is a plain CMake
project (no ROS dependencies) -- colcon just drives the build.

```bash
# from the workspace root, e.g. /ws
colcon build --packages-select piper_cpp
source install/setup.bash
```

The CLI binaries (`piper_enable`, `piper_gripper_sweep`, …) end up on `PATH` after sourcing,
and the `piper_cpp` static library + headers are installed under `install/piper_cpp/`.

### Building standalone (without colcon)

If you are not in a ROS environment, build it with plain CMake. The package has no ROS
dependency.

```bash
cd src/piper_cpp/piper_cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# optional: install to /usr/local (or set CMAKE_INSTALL_PREFIX to your prefix of choice)
sudo cmake --install build
```

The CLI binaries land in `build/` (or `${CMAKE_INSTALL_PREFIX}/bin` after install). The
library installs as `libpiper_cpp.a` with headers under `include/piper_cpp/`.

## Bringing up the CAN bus

The Piper expects a 1 Mbit/s bus on the standard `can` interface naming. If your CAN device
is detected by the kernel but not yet up:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

Or use the bundled helper (brings up every detected adapter at 1 Mbit/s):

```bash
can_activate
```

## Quick start

A minimal control session:

```cpp
#include "piper_cpp/interface/piper_interface_v2.h"

using namespace piper_cpp;

int main()
{
    PiperInterfaceV2 piper("can0");
    piper.connectPort(/*can_init=*/true, /*piper_init=*/true, /*start_threads=*/true);

    piper.enableRobot();
    while (!piper.isEnabled()) std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Motion mode must be set explicitly after enable, or position commands are ignored.
    piper.sendMotionCtrl2(CtrlMode::CanCmd, MoveMode::MoveJ, /*speed=*/30,
                          MitMode::PositionVelocity);

    piper.controlJoints(0, 0, 0, 0, 0, 0);          // millideg
    piper.controlGripper(0, 1000, GripperStatusCode::Enable);  // pos in 0.001 mm, effort

    auto js = piper.getArmJointStates();
    if (js.is_valid) { /* js.value.joint_1 ... joint_6 in millideg */ }

    piper.disconnectPort(std::chrono::milliseconds{200});
}
```

A few subtleties learned the hard way (and worth knowing before commanding the arm):

- **Motion mode is required.** `enableRobot()` powers the joints but leaves the arm in a
  mode that silently ignores position commands. You must call `sendMotionCtrl2(...)` (or
  one of the granular setters: `setCtrlMode`, `setMoveMode`, `setSpeedPercentage`,
  `setMitMode`) at least once after enable.
- **The gripper needs a homing reference.** Send `setGripperZero()` before enabling, and
  give it ~1.5s to settle. After that the gripper accepts position commands. Without
  homing, every `controlGripper` call is silently dropped.
- **Stream the gripper command continuously.** The gripper firmware (seems to) auto-disable itself
  ~1s after the command stream stops, and a Ctrl-C in the middle of a drive could latch the
  firmware into a fault that only clears on power-cycle. Always send a few `Disable` frames
  before disconnecting (the bundled scripts demonstrate the SIGINT-safe pattern).
- **Master/slave (a.k.a. leader/follower) mode change requires a power cycle.** The
  master/slave config (`setAsMasterArm()` / `setAsSlaveArm()` / `setMasterSlaveConfig()`) is
  written to firmware and persists across reboots, but the arm only picks up the change at
  the next power cycle. Switching out of leader mode without power-cycling will leave the
  arm silently ignoring every motion command.
- **A `resetPiper()` is sometimes needed to recover from invisible faults.** The arm can end
  up in a state where motion commands look accepted on the bus but the joints/gripper don't move,
  with no obvious error in the feedback. `resetPiper()` (or the `piper_reset` CLI) drops
  power and clears all internal/error flags — after that, `enableRobot()` and motion-mode
  setup brings it back. It's a softer alternative to a physical power cycle.

## CLI binaries

All scripts in [scripts/](scripts/) are built as standalone CLI binaries. The most useful
ones during bringup:

| Binary                             | Purpose                                                               |
| ---------------------------------- | --------------------------------------------------------------------- |
| `can_activate`                     | Bring every detected CAN adapter up at 1 Mbit/s.                      |
| `piper_enable`                     | Enable / disable all joints + gripper motor (`--disable` to disable). |
| `piper_go_zero`                    | Drive all joints + gripper to zero (`--speed`, `--duration-s`).       |
| `piper_set_zero`                   | One-shot "set current position as zero" for joints or gripper.        |
| `piper_gripper_sweep`              | Open/close the gripper repeatedly. SIGINT-safe.                       |
| `piper_joint_ctrl_demo`            | Sinusoidal joint sweep demo.                                          |
| `piper_end_pose_demo`              | Cartesian Z-axis sweep demo.                                          |
| `piper_set_role`                   | Configure the arm as master or slave.                                 |
| `piper_read_firmware`              | Read and print the firmware version.                                  |
| `piper_reset`                      | Send the firmware reset command (drops power -- arm will fall).       |
| `piper_interface_v2_read_test`     | Live print of every cached state snapshot.                            |
| `piper_parser_v2_read_test`        | Decode raw CAN frames without state caching.                          |
| `piper_cpp_std_can_interface_read` | Lowest-level frame dump -- use to test the SocketCAN wrapper itself.  |

Each script has a usage block at the top of its source file describing flags and any
prerequisites.

## Repository layout

```
piper_cpp/
├── include/piper_cpp/        # public headers
│   ├── interface/            # PiperInterfaceV2 (main entry point)
│   ├── protocol/             # PiperParserV2 (CAN frame decode)
│   ├── types/                # message structs + control enums
│   ├── feedback/             # feedback message definitions
│   ├── std_can_interface.h   # SocketCAN wrapper
│   ├── can_utils.h           # CAN bus bring-up helpers
│   ├── fk.h                  # forward kinematics
│   └── piper_params.h        # arm geometry + DH params
├── src/                      # implementations
└── scripts/                  # CLI binaries
```

## License

MIT — see [LICENSE](../LICENSE).

Ported from [`piper_sdk`](https://github.com/agilexrobotics/piper_sdk) (MIT,
© 2024 Agilex Robotice Co., Ltd.).
