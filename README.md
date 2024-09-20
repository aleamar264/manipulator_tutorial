 ```shell
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-*
sudo apt-get install ros-humble-xacro 
sudo apt-get install ros-humble-gazebo-ros
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-ros2-controllers
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt-get install ros-humble-moveit
```

## DDS
Secure and reliable communications

![[Pasted image 20240312144842.png]]

## URDF

```shell
ros2 launch urdf_tutorial display.launch.py model:=/full_path/bot.urdf.xacro 
```

## Parameters
```shell
ros2 param set <node_name> <param_name> <value>
ros2 run <node_name> --ros-args -p <param_name>:=value>

```


### Visualization

💀⚡NOTE: Always source the workspace before do anything, this help with the autocompletion of topics, params, nodes, etc... also help to find the packages created in the workspace

*Terminal 1*
```shell
ros2 run robot_state_publisher robot_state_publisher  --ros-args -p robot_description:="$(xacro </path/model/of/urdf.xacro>)"
```
*Terminal 2*
```shell
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
*Terminal 3*
```shell
ros2 run rviz2 rviz2 
```


### Launch

```python
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import shlex


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

```

## ROS2 Control



## MoveIt2

