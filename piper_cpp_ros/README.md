# piper_cpp_ros

ros2_control hardware interface for the AgileX Piper 6-DOF arm and its optional parallel-jaw
gripper, built on top of [`piper_cpp`](../piper_cpp).

The plugin (`piper_cpp_ros::PiperHardware`) opens the CAN bus via `piper_cpp`, enables the
arm, sets motion mode (`CanCmd / MoveJ / PositionVelocity`), optionally drives the joints
to zero on activation, and then streams `controlJoints()` setpoints from the latest
commanded positions every `write()` cycle. With `with_gripper:=true`, it also exposes a
7th prismatic joint and streams `controlGripper(..., Enable)` alongside the arm command.
This is the same control strategy demonstrated in
[piper_joint_sine_demo.cpp](../piper_cpp/scripts/piper_joint_sine_demo.cpp) and
[piper_gripper_sweep.cpp](../piper_cpp/scripts/piper_gripper_sweep.cpp).

For a no-hardware-required mode (mock joints, useful for MoveIt/RViz development), pass
`use_real_hardware:=false` -- the plugin is swapped for `mock_components/GenericSystem`.

## Features

- A `hardware_interface::SystemInterface` plugin (`piper_cpp_ros/PiperHardware`) covering
  arm + optional gripper.
- A `<ros2_control>` xacro snippet ([urdf/piper.ros2_control.xacro](urdf/piper.ros2_control.xacro))
  that you can include from your own URDF, plus two top-level wrappers:
  - [urdf/piper_with_ros2_control.urdf.xacro](urdf/piper_with_ros2_control.urdf.xacro) --
    arm only (uses `piper_no_gripper_description`).
  - [urdf/piper_with_gripper_with_ros2_control.urdf.xacro](urdf/piper_with_gripper_with_ros2_control.urdf.xacro) --
    arm + gripper. Composes `piper_no_gripper_description` with the package-local
    [urdf/piper_gripper.urdf.xacro](urdf/piper_gripper.urdf.xacro), which vendors the
    gripper geometry from `piper_description` with a `<mimic joint="joint7" multiplier="-1"/>`
    baked into joint8 so robot_state_publisher / MoveIt can derive its pose without a
    separate driver.
- Controller manager config ([config/piper_controllers.yaml](config/piper_controllers.yaml))
  with `joint_state_broadcaster`, `joint_position_controller`
  (`forward_command_controller/ForwardCommandController`),
  `joint_trajectory_controller` (`joint_trajectory_controller/JointTrajectoryController`),
  and `gripper_controller` (also a `JointTrajectoryController` -- only spawned when
  `with_gripper:=true`).
- A launch file ([launch/piper_control.launch.py](launch/piper_control.launch.py)) that
  brings everything up. Honours `with_gripper`, `use_real_hardware`,
  `gripper_max_effort`, and `home_gripper_on_activate`.

## State / command interfaces

| Joint                                              | State                                          | Command        |
| -------------------------------------------------- | ---------------------------------------------- | -------------- |
| `joint1`-`joint6`                                  | position (rad), velocity (rad/s), effort (N·m) | position (rad) |
| `joint7` (gripper, only with `with_gripper:=true`) | position (m, jaw stroke), effort (N·m)         | position (m)   |

Velocity and effort for the arm joints come from the per-joint high-speed CAN feedback
(~200 Hz). Effort is computed by `piper_cpp` from motor current via the manufacturer's
per-joint coefficient table. Gripper effort is read directly from the gripper feedback
frame.

`joint8` (the left finger) does not appear in the state/command tables: it's declared in
the URDF as a mimic of `joint7` (multiplier=-1), so `robot_state_publisher` derives its
pose automatically without needing a driver.

## Hardware parameters (set in the URDF's `<hardware>` block)

| Param                      | Default | Meaning                                                                                                                    |
| -------------------------- | ------- | -------------------------------------------------------------------------------------------------------------------------- |
| `can_interface`            | `can0`  | SocketCAN interface to open.                                                                                               |
| `speed_pct`                | `30`    | Speed-cap percentage forwarded to `MoveJ`.                                                                                 |
| `go_to_zero_on_activate`   | `true`  | Drive arm joints to zero before handing over to the controller.                                                            |
| `with_gripper`             | `false` | Expose a 7th prismatic joint for the parallel-jaw gripper.                                                                 |
| `gripper_max_effort`       | `1.0`   | Max gripper torque in N·m (range 0..5). Sent every cycle alongside the position target. Ignored when `with_gripper=false`. |
| `home_gripper_on_activate` | `false` | Set the current jaw position as gripper zero on activation. Only enable if you always activate from a known reference.   |

### Launch-only parameter

| Param               | Default | Meaning                                                                                                                                            |
| ------------------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `use_real_hardware` | `true`  | `false` swaps the `<plugin>` for `mock_components/GenericSystem` (no CAN, no Piper required). Useful for MoveIt/RViz development against the URDF. |

## Build

This package depends on:

- [`piper_cpp`](../piper_cpp) -- the underlying CAN/SDK library.
- The standard ros2_control stack (`ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`).
- [`piper_description`](https://github.com/agilexrobotics/piper_ros/tree/humble/src/piper_description)
  (use "humble" branch) -- for the default URDF the bundled launch file feeds to xacro. This is the **only**
  package from the upstream [`piper_ros`](https://github.com/agilexrobotics/piper_ros) repo that you need;
  the rest of `piper_ros` is unrelated and can be ignored if you only want this hardware interface. If
  you have your own URDF that includes [urdf/piper.ros2_control.xacro](urdf/piper.ros2_control.xacro),
  you don't need `piper_description` either, but using the one from `piper_description` is recommended.

Build the workspace with colcon:

```bash
colcon build --packages-up-to piper_cpp_ros
source install/setup.bash
```

## Run

Bring the CAN bus up first (skip if running with `use_real_hardware:=false`):

```bash
sudo ip link set can0 up type can bitrate 1000000
# or: use `can_activate` in the cli (installed with piper_cpp)
```

Then launch:

```bash
# Arm only, real hardware (default).
ros2 launch piper_cpp_ros piper_control.launch.py

# Arm + gripper, real hardware.
ros2 launch piper_cpp_ros piper_control.launch.py with_gripper:=true

# Arm + gripper, mock hardware (no CAN required).
ros2 launch piper_cpp_ros piper_control.launch.py with_gripper:=true use_real_hardware:=false
```

Override the defaults if needed:

```bash
ros2 launch piper_cpp_ros piper_control.launch.py \
    can_interface:=can0 \
    speed_pct:=30 \
    go_to_zero_on_activate:=true \
    with_gripper:=true \
    gripper_max_effort:=1.5 \
    home_gripper_on_activate:=false
```

## Driving the arm

### Joint trajectory controller (default; what MoveIt uses)

The launch file activates `joint_trajectory_controller` automatically. Send a
`FollowJointTrajectory` action goal:

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
      points: [
        { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 2 } },
        { positions: [0.1, 0.1, -0.1, 0.1, -0.1, 0.1], time_from_start: { sec: 5 } }
      ]
    }
  }"
```

### Forward command controller (raw position streaming)

Loaded inactive at launch. Activate it (after deactivating JTC, since they share the position
command interface):

```bash
ros2 control set_controller_state joint_trajectory_controller inactive
ros2 control set_controller_state joint_position_controller active
```

Then publish setpoints (rad):

```bash
ros2 topic pub /joint_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }"
```

## Driving the gripper

When `with_gripper:=true`, `gripper_controller` is spawned and exposes its own
`FollowJointTrajectory` action on `joint7`. Position is in metres of jaw stroke; the URDF
limits are `0..0.035` m.

```bash
# Open
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [joint7],
      points: [{ positions: [0.035], time_from_start: { sec: 1 } }]
    }
  }"

# Close
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [joint7],
      points: [{ positions: [0.0], time_from_start: { sec: 1 } }]
    }
  }"
```

The gripper firmware ignores position commands until it has been zeroed at least once
per power-up. If `home_gripper_on_activate:=false` (the default) and you haven't
zeroed the gripper yet, run [`piper_set_zero --gripper`](../piper_cpp/scripts/piper_set_zero.cpp)
once before sending position commands. Set `home_gripper_on_activate:=true` only if you
always activate with the jaws at a known reference (typically fully closed) -- it latches
whatever pose the jaws are in as the new zero, so calling it from a half-open state will
re-calibrate incorrectly.

## Integrating with your own URDF

Include the ros2_control snippet from your existing piper xacro:

```xml
<xacro:include filename="$(find piper_cpp_ros)/urdf/piper.ros2_control.xacro"/>
<xacro:piper_ros2_control can_interface="can0" speed_pct="30"/>
```

For arm + gripper:

```xml
<xacro:include filename="$(find piper_cpp_ros)/urdf/piper.ros2_control.xacro"/>
<xacro:piper_ros2_control
    can_interface="can0"
    speed_pct="30"
    with_gripper="true"
    gripper_jname="joint7"
    gripper_max_effort="1.0"/>
```

The macro declares `joint1`-`joint6` in that order, mapping to the Piper's motor 1 through
motor 6. When `with_gripper:=true`, the gripper joint (`joint7` by default) is appended.
Joint names must match what your URDF's `<robot>` definition uses.

If you're attaching the gripper to your own robot URDF, make sure your `joint8` (or
equivalent left-finger joint) carries a `<mimic joint="joint7" multiplier="-1.0"/>` tag,
otherwise robot_state_publisher won't have a way to compute its TF and MoveIt will warn
"Missing joint8". The bundled [piper_gripper.urdf.xacro](urdf/piper_gripper.urdf.xacro)
shows the pattern.

## Notes

- The hardware interface streams targets only when the controller manager is active. On
  `on_deactivate()`, joint motors stay enabled and hold the last commanded pose -- disabling
  motors here would drop the arm. If you want to deactivate, use `piper_cpp` API directly,
  or the shipped `piper_enable --disable` command from CLI (shipped with `piper_cpp`).
- `update_rate` in [piper_controllers.yaml](config/piper_controllers.yaml) sets the
  read/write rate. 200 Hz is used, this is what the robot seem to expect; the CAN bus and
  arm firmware comfortably handle this and higher.
- If you need to reset the arm out of a stuck state without restarting ROS, use the
  `piper_reset` CLI from `piper_cpp` -- the `on_cleanup` lifecycle transition will not clear
  firmware-side latched faults.
- Under `use_real_hardware:=false`, `mock_components/GenericSystem` mirrors position
  commands directly back as state, so JTC trajectories execute "perfectly" with no
  dynamics. CAN/speed/gripper-firmware params are silently ignored. Velocity and effort
  state default to 0 in mock mode.

## License

MIT -- see [../LICENSE](../LICENSE).
