import os
from os import pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_description = get_package_share_directory("robot_description")
    description_prefix = get_package_prefix("robot_description")

    model_path = os.path.join(bot_description, "models")
    model_path += pathsep + os.path.join(description_prefix, "share")

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            bot_description,
            "urdf",
            "bot.urdf.xacro",
        ),
        description="Absolute path to the urdf file",
    )

    robot_description: ParameterValue = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node: Node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        )
    )
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        )
    )

    spwan_robot: Node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bot", "-topic", "robot_description"],
        output="screen",
    )

    return LaunchDescription(
        [
            env_variable,
            model_arg,
            robot_state_publisher_node,
            start_gazebo_server,
            start_gazebo_client,
            spwan_robot,
        ]
    )
