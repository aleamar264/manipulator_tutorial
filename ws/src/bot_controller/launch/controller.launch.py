import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim", 
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    bot_description = get_package_share_directory("robot_description")
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
        Command(["xacro ", LaunchConfiguration("model"), 'is_sim:=False']), value_type=str
    )

    robot_state_publisher_node: Node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim)
    )

    controller_manager: Node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": is_sim},
                     os.path.join(get_package_share_directory("bot_controller"),
                                  "config","bot_controllers.yaml")],
        condition=UnlessCondition(is_sim)
    )

    joint_state_broadcaster_spawner: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    arm_controller_spawner: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    gripper_controller_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )])

    return LaunchDescription(
        [
            is_sim_arg,
            model_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
