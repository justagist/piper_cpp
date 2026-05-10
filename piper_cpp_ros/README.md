# piper_cpp_ros

ros2_control hardware interface for the AgileX Piper 6-DOF arm, built on top of
[`piper_cpp`](../piper_cpp).

The plugin (`piper_cpp_ros::PiperHardware`) opens the CAN bus via `piper_cpp`, enables the
arm, sets motion mode (`CanCmd / MoveJ / PositionVelocity`), optionally drives the joints to
zero on activation, and then streams `controlJoints()` setpoints from the latest commanded
positions every `write()` cycle. This is the same control strategy demonstrated in
[piper_joint_sine_demo.cpp](../piper_cpp/scripts/piper_joint_sine_demo.cpp).

## Features

- A `hardware_interface::SystemInterface` plugin (`piper_cpp_ros/PiperHardware`).
- A `<ros2_control>` xacro snippet ([urdf/piper.ros2_control.xacro](urdf/piper.ros2_control.xacro))
  that you can include from your own URDF, plus a top-level wrapper
  ([urdf/piper_with_ros2_control.urdf.xacro](urdf/piper_with_ros2_control.urdf.xacro)) that
  combines `piper_description`'s URDF with the ros2_control block.
- Controller manager config ([config/piper_controllers.yaml](config/piper_controllers.yaml))
  with `joint_state_broadcaster`, `joint_position_controller`
  (`forward_command_controller/ForwardCommandController`), and
  `joint_trajectory_controller` (`joint_trajectory_controller/JointTrajectoryController`).
- A launch file ([launch/piper_control.launch.py](launch/piper_control.launch.py)) that
  brings everything up.

## State / command interfaces

| Joint    | State (rad) | Command (rad) |
| -------- | ----------- | ------------- |
| `joint1` | position    | position      |
| ...      | position    | position      |
| `joint6` | position    | position      |

## Hardware parameters (set in the URDF's `<hardware>` block)

| Param                    | Default | Meaning                                                     |
| ------------------------ | ------- | ----------------------------------------------------------- |
| `can_interface`          | `can0`  | SocketCAN interface to open.                                |
| `speed_pct`              | `30`    | Speed-cap percentage forwarded to `MoveJ`.                  |
| `go_to_zero_on_activate` | `true`  | Drive joints to zero before handing over to the controller. |

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

Bring the CAN bus up first:

```bash
sudo ip link set can0 up type can bitrate 1000000
# or: use `can_activate` in the cli (installed with piper_cpp)
```

Then launch:

```bash
ros2 launch piper_cpp_ros piper_control.launch.py
```

Override the defaults if needed:

```bash
ros2 launch piper_cpp_ros piper_control.launch.py \
    can_interface:=can0 \
    speed_pct:=30 \
    go_to_zero_on_activate:=true
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

## Integrating with your own URDF

Include the ros2_control snippet from your existing piper xacro:

```xml
<xacro:include filename="$(find piper_cpp_ros)/urdf/piper.ros2_control.xacro"/>
<xacro:piper_ros2_control can_interface="can0" speed_pct="30"/>
```

The macro declares `joint1`-`joint6` in that order, mapping to the Piper's motor 1 through
motor 6. Joint names must match what your URDF's `<robot>` definition uses.

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

## License

MIT -- see [../LICENSE](../LICENSE).
