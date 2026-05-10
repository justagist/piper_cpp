"""
Brings up the Piper arm under ros2_control.

Spawns:
  - controller_manager / ros2_control_node
  - robot_state_publisher
  - joint_state_broadcaster
  - joint_trajectory_controller
  - joint_position_controller (loaded but not started; activate from CLI when needed)
  - gripper_controller (only when with_gripper:=true)

To run a different URDF, override `description_package`, `description_file`, and
`controllers_file` directly -- those take precedence over the with_gripper toggle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_actions(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file_arg = LaunchConfiguration("description_file").perform(context)
    controllers_file_arg = LaunchConfiguration("controllers_file").perform(context)
    can_interface = LaunchConfiguration("can_interface").perform(context)
    speed_pct = LaunchConfiguration("speed_pct").perform(context)
    go_to_zero_on_activate = LaunchConfiguration("go_to_zero_on_activate").perform(context)
    with_gripper = LaunchConfiguration("with_gripper").perform(context).lower() in (
        "true",
        "1",
    )
    gripper_max_effort = LaunchConfiguration("gripper_max_effort").perform(context)
    home_gripper_on_activate = LaunchConfiguration("home_gripper_on_activate").perform(
        context
    )

    if not description_file_arg:
        description_file_arg = (
            "piper_with_gripper_with_ros2_control.urdf.xacro"
            if with_gripper
            else "piper_with_ros2_control.urdf.xacro"
        )
    if not controllers_file_arg:
        controllers_file_arg = "piper_controllers.yaml"

    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare(description_package), "urdf", description_file_arg]
        ),
        " ",
        f"can_interface:={can_interface}",
        " ",
        f"speed_pct:={speed_pct}",
        " ",
        f"go_to_zero_on_activate:={go_to_zero_on_activate}",
    ]
    if with_gripper:
        xacro_cmd += [
            " ",
            f"gripper_max_effort:={gripper_max_effort}",
            " ",
            f"home_gripper_on_activate:={home_gripper_on_activate}",
        ]

    robot_description_content = Command(xacro_cmd)
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("piper_cpp_ros"), "config", controllers_file_arg]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Loaded but inactive -- activate from CLI when you want to use the forward controller:
    #   ros2 control set_controller_state joint_position_controller active
    spawn_fwd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )

    post_jsb_actions = [spawn_jtc, spawn_fwd]
    if with_gripper:
        spawn_gripper = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        )
        post_jsb_actions.append(spawn_gripper)

    delay_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=post_jsb_actions,
        )
    )

    return [rsp_node, control_node, spawn_jsb, delay_after_jsb]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("description_package", default_value="piper_cpp_ros"),
            # Empty default lets _build_actions pick a gripper-aware default. Override to
            # a specific filename to force a particular URDF.
            DeclareLaunchArgument("description_file", default_value=""),
            DeclareLaunchArgument("controllers_file", default_value=""),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("speed_pct", default_value="30"),
            DeclareLaunchArgument("go_to_zero_on_activate", default_value="true"),
            DeclareLaunchArgument(
                "with_gripper",
                default_value="false",
                description="Expose the parallel-jaw gripper as a 7th prismatic joint and "
                "spawn a gripper_controller for it.",
            ),
            DeclareLaunchArgument(
                "gripper_max_effort",
                default_value="1.0",
                description="Max gripper torque in N*m (range 0..5). Ignored unless "
                "with_gripper:=true.",
            ),
            DeclareLaunchArgument(
                "home_gripper_on_activate",
                default_value="false",
                description="Latch the current jaw position as gripper zero on activate. "
                "Only enable if you always activate from a known reference.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
