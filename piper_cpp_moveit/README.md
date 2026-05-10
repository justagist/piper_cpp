# piper_cpp_moveit

MoveIt 2 configuration for the AgileX Piper 6-DOF arm. Plans against the URDF from
`piper_description` and dispatches resulting trajectories through the
`joint_trajectory_controller` exposed by [`piper_cpp_ros`](../piper_cpp_ros).

## Build

Depends on:

- [`piper_cpp_ros`](../piper_cpp_ros) (and transitively `piper_cpp` and
  `piper_description`).
- MoveIt 2: `ros-jazzy-moveit` (and friends — see [package.xml](package.xml) for the
  full list).

```bash
colcon build --packages-up-to piper_cpp_moveit
source install/setup.bash
```

## Run

Bring the CAN bus up first:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### All-in-one

```bash
ros2 launch piper_cpp_moveit piper_moveit.launch.py
```

This starts the controller manager, robot_state_publisher, joint_state_broadcaster,
joint_trajectory_controller, move_group, and RViz with the MoveIt panel pre-loaded.

### Hardware in one terminal, MoveIt in another

```bash
# terminal A
ros2 launch piper_cpp_ros piper_control.launch.py

# terminal B (after JTC is up)
ros2 launch piper_cpp_moveit move_group.launch.py
```

## License

MIT — see [../piper_cpp/LICENSE](../piper_cpp/LICENSE).
