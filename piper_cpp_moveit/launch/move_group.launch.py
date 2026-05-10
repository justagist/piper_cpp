"""
MoveIt-only launch. Brings up move_group + RViz, but assumes piper_cpp_ros's controller
manager (with joint_trajectory_controller active) is already running.

Useful when you want to keep the hardware bringup separate from the MoveIt session, or when
re-launching MoveIt without restarting the controller manager.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("piper_cpp_moveit")
    piper_ros_share = get_package_share_directory("piper_cpp_ros")

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_cpp_moveit")
        .robot_description(
            file_path=os.path.join(
                piper_ros_share, "urdf", "piper_with_ros2_control.urdf.xacro"
            ),
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

    move_group_params = moveit_config.to_dict()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

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

    return LaunchDescription([move_group_node, rviz_node])
