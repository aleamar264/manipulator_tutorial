from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("robot_description"),
            "urdf/bot.urdf.xacro",
        ),
        description="Absolute path to the urdf file",
    )

    robot_description: ParameterValue = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")])
    )

    joint_state_publisher: Node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"robot_description": robot_description}],
        name="robot_state_publisher",
    )

    joint_state_publisher_gui: Node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node: Node = Node(
        executable="rviz2",
        package="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("robot_description"),
                "rviz",
                "display.rviz",
            ),
        ],
        name="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [model_arg, joint_state_publisher, joint_state_publisher_gui, rviz_node]
    )
