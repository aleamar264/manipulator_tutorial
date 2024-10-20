from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description()->LaunchDescription:
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bot_controller"),
                     "launch",
                     "controller.launch.py"),
        launch_arguments={"is_sim": "True"}.items()
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_moveit"),
            "launch",
            "moveit.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )


    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_remote"),
            "launch",
            "remote_interface.launch.py"
        ),
        )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface
    ])