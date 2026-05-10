"""
MoveIt-only launch. Brings up move_group + RViz, but assumes piper_cpp_ros's controller
manager (with joint_trajectory_controller active, plus gripper_controller when
with_gripper:=true) is already running.

Useful when you want to keep the hardware bringup separate from the MoveIt session, or when
re-launching MoveIt without restarting the controller manager.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _build_actions(context, *args, **kwargs):
    use_real_hardware = LaunchConfiguration("use_real_hardware").perform(context)
    with_gripper = LaunchConfiguration("with_gripper").perform(context).lower() in (
        "true",
        "1",
    )

    moveit_share = get_package_share_directory("piper_cpp_moveit")
    piper_ros_share = get_package_share_directory("piper_cpp_ros")

    joint_limits_yaml = os.path.join(moveit_share, "config", "joint_limits.yaml")
    moveit_controllers_yaml = os.path.join(
        moveit_share, "config", "moveit_controllers.yaml"
    )

    if with_gripper:
        urdf_xacro = os.path.join(
            piper_ros_share, "urdf", "piper_with_gripper_with_ros2_control.urdf.xacro"
        )
        srdf_xacro = os.path.join(moveit_share, "srdf", "piper_with_gripper.srdf.xacro")
    else:
        urdf_xacro = os.path.join(
            piper_ros_share, "urdf", "piper_with_ros2_control.urdf.xacro"
        )
        srdf_xacro = os.path.join(moveit_share, "srdf", "piper.srdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_cpp_moveit")
        .robot_description(
            file_path=urdf_xacro, mappings={"use_real_hardware": use_real_hardware}
        )
        .robot_description_semantic(file_path=srdf_xacro)
        .robot_description_kinematics(
            file_path=os.path.join(moveit_share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=joint_limits_yaml)
        .trajectory_execution(file_path=moveit_controllers_yaml)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
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

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_real_hardware",
                default_value="true",
                description="Must match how piper_cpp_ros was launched. Only used to keep "
                "move_group's URDF identical to ros2_control's.",
            ),
            DeclareLaunchArgument(
                "with_gripper",
                default_value="false",
                description="Plan and execute on the parallel-jaw gripper as well as the arm. "
                "Must match how piper_cpp_ros was launched.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
