"""
Brings up the Piper arm under ros2_control.

Spawns:
  - controller_manager / ros2_control_node
  - robot_state_publisher
  - joint_state_broadcaster
  - joint_trajectory_controller
  - joint_position_controller (loaded but not started; activate from CLI when needed)

Default URDF: piper_cpp_ros/urdf/piper_with_ros2_control.urdf.xacro
  (wraps piper_description's piper_no_gripper_description.xacro and inserts the ros2_control
  block from piper.ros2_control.xacro.)

To run a different URDF, override `description_package` and `description_file`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
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


def generate_launch_description():
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    can_interface = LaunchConfiguration("can_interface")
    speed_pct = LaunchConfiguration("speed_pct")
    go_to_zero_on_activate = LaunchConfiguration("go_to_zero_on_activate")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "can_interface:=",
            can_interface,
            " ",
            "speed_pct:=",
            speed_pct,
            " ",
            "go_to_zero_on_activate:=",
            go_to_zero_on_activate,
        ]
    )
    # Wrap with ParameterValue(..., value_type=str) so launch_ros doesn't try to YAML-parse
    # the URDF (long strings with `:` / newlines look like YAML to its type sniffer).
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("piper_cpp_ros"), "config", controllers_file]
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

    # Spawn JSB first; once it's up, spawn JTC and the forward controller.
    delay_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_jtc, spawn_fwd],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("description_package", default_value="piper_cpp_ros"),
            DeclareLaunchArgument(
                "description_file",
                default_value="piper_with_ros2_control.urdf.xacro",
            ),
            DeclareLaunchArgument(
                "controllers_file", default_value="piper_controllers.yaml"
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("speed_pct", default_value="30"),
            DeclareLaunchArgument("go_to_zero_on_activate", default_value="true"),
            rsp_node,
            control_node,
            spawn_jsb,
            delay_after_jsb,
        ]
    )
