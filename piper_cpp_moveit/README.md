# piper_cpp_moveit

MoveIt 2 configuration for the AgileX Piper 6-DOF arm and its optional parallel-jaw
gripper. Plans against the URDF from `piper_description` and dispatches the resulting
trajectories through the `joint_trajectory_controller` (and `gripper_controller`, when
the gripper is enabled) exposed by [`piper_cpp_ros`](../piper_cpp_ros).

[![piper_cpp.gif](https://media.githubusercontent.com/media/justagist/_assets/refs/heads/main/piper_cpp/piper_cpp.gif)](https://media.githubusercontent.com/media/justagist/_assets/refs/heads/main/piper_cpp/piper_cpp.gif)

## Features

- `manipulator` planning group (kinematic chain `base_link` --> `link6`).
- `gripper` planning group (chain `gripper_base` --> `link7`), wired as the end-effector for
  `manipulator`. Only present when launched with `with_gripper:=true`.
- Group states: `home`, `ready` (arm), and `open`, `close` (gripper, gripper-only mode).
- Two SRDFs: [srdf/piper.srdf.xacro](srdf/piper.srdf.xacro) for arm-only and
  [srdf/piper_with_gripper.srdf.xacro](srdf/piper_with_gripper.srdf.xacro) for the gripper
  variant. Selected automatically by the launch file based on `with_gripper`.
- A single MoveIt controller config ([config/moveit_controllers.yaml](config/moveit_controllers.yaml))
  that lists both `joint_trajectory_controller` (joints 1-6) and `gripper_controller`
  (joint7). moveit_simple_controller_manager creates action handles lazily, so when
  `with_gripper:=false` the gripper handle just stays idle -- arm-only trajectories never
  exercise it.
- Joint limits ([config/joint_limits.yaml](config/joint_limits.yaml)) covering the six arm
  joints plus `joint7` (harmless when arm-only).

## Build

Depends on:

- [`piper_cpp_ros`](../piper_cpp_ros) (and transitively `piper_cpp` and
  `piper_description`).
- MoveIt 2: `ros-jazzy-moveit` (and friends -- see [package.xml](package.xml) for the
  full list).

```bash
colcon build --packages-up-to piper_cpp_moveit
source install/setup.bash
```

## Run

Bring the CAN bus up first (skip if running with `use_real_hardware:=false`):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### All-in-one

```bash
# Arm only, real hardware (default).
ros2 launch piper_cpp_moveit piper_moveit.launch.py

# Arm + gripper, real hardware.
ros2 launch piper_cpp_moveit piper_moveit.launch.py with_gripper:=true

# Arm + gripper, mock hardware (no CAN required, useful for sim-based development).
ros2 launch piper_cpp_moveit piper_moveit.launch.py with_gripper:=true use_real_hardware:=false
```

This starts the controller manager, robot_state_publisher, joint_state_broadcaster,
joint_trajectory_controller (and gripper_controller, when `with_gripper:=true`),
move_group, and RViz with the MoveIt panel pre-loaded.

### Hardware in one terminal, MoveIt in another

```bash
# terminal A
ros2 launch piper_cpp_ros piper_control.launch.py with_gripper:=true

# terminal B (after the controllers are up)
ros2 launch piper_cpp_moveit move_group.launch.py with_gripper:=true
```

The `with_gripper` and `use_real_hardware` flags must match between the two launches --
they're what selects the URDF, SRDF, and controllers config.

## Launch arguments

| Arg                        | Default | Forwarded to                                                     |
| -------------------------- | ------- | ---------------------------------------------------------------- |
| `with_gripper`             | `false` | URDF, SRDF, MoveIt controllers, `piper_cpp_ros` launch.          |
| `use_real_hardware`        | `true`  | URDF (`mock_components/GenericSystem` when `false`).             |
| `can_interface`            | `can0`  | `piper_cpp_ros` launch (real hardware only).                     |
| `speed_pct`                | `30`    | `piper_cpp_ros` launch (real hardware only).                     |
| `go_to_zero_on_activate`   | `true`  | `piper_cpp_ros` launch (real hardware only).                     |
| `gripper_max_effort`       | `1.0`   | `piper_cpp_ros` launch (real hardware + with_gripper only).      |
| `home_gripper_on_activate` | `false` | `piper_cpp_ros` launch (real hardware + with_gripper only).      |

## License

MIT -- see [../piper_cpp/LICENSE](../piper_cpp/LICENSE).
