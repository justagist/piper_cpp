"""
All-in-one launch: brings up piper_cpp_ros (controller_manager + JTC) AND MoveIt's move_group
+ RViz with the MoveIt motion planning plugin.

Use this when you want a single command to go from a powered Piper to a MoveIt session.

If you already have piper_cpp_ros running (e.g. you started it from another launch), use
`move_group.launch.py` instead -- that one only spawns move_group + RViz and assumes
ros2_control is already up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    speed_pct = LaunchConfiguration("speed_pct")
    go_to_zero_on_activate = LaunchConfiguration("go_to_zero_on_activate")

    moveit_share = get_package_share_directory("piper_cpp_moveit")
    piper_ros_share = get_package_share_directory("piper_cpp_ros")

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_cpp_moveit")
        .robot_description(
            file_path=os.path.join(
                piper_ros_share, "urdf", "piper_with_ros2_control.urdf.xacro"
            ),
            mappings={
                "can_interface": "can0",
                "speed_pct": "30",
                "go_to_zero_on_activate": "true",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, "srdf", "piper.srdf.xacro")
        )
        .robot_description_kinematics(
            file_path=os.path.join(moveit_share, "config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(moveit_share, "config", "joint_limits.yaml")
        )
        .trajectory_execution(
            file_path=os.path.join(moveit_share, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Bring up piper_cpp_ros (controller_manager, robot_state_publisher, JSB, JTC).
    piper_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("piper_cpp_ros"), "launch", "piper_control.launch.py"]
            )
        ),
        launch_arguments={
            "can_interface": can_interface,
            "speed_pct": speed_pct,
            "go_to_zero_on_activate": go_to_zero_on_activate,
        }.items(),
    )

    move_group_params = moveit_config.to_dict()

    # MoveIt move_group node.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

    # RViz with MoveIt panel pre-loaded.
    rviz_config = os.path.join(moveit_share, "rviz", "piper_moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("speed_pct", default_value="30"),
            DeclareLaunchArgument("go_to_zero_on_activate", default_value="true"),
            piper_ros_launch,
            move_group_node,
            rviz_node,
        ]
    )
