from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    moveit_config = (MoveItConfigsBuilder(
        "bot", package_name="robot_moveit"
    ).robot_description(file_path=os.path.join(get_package_share_directory("robot_description"), "urdf", "bot.urdf.xacro"))
    .robot_description_semantic(file_path="config/robot.srdf")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .moveit_cpp(file_path="config/planning_python_api.yaml")
    .to_moveit_configs())

    simple_move_interface  = Node(
        package="py_example",
        executable="simple_moveit_interface",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_true": True}]
    )

    return LaunchDescription(
       [simple_move_interface]
    )