import rclpy
from rclpy.logging import get_logger
import numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def move_robot():
    bot = MoveItPy(node_name=" moveit_py")
    bot_arm = bot.get_planning_component("arm")
    bot_gripper = bot.get_planning_component("gripper")


    arm_state = RobotState(bot.get_robot_model())
    gripper_state = RobotState(bot.get_robot_model())


    arm_state.set_joint_group_positions("arm", np.array([1.57, 0.0, 0.0]))
    gripper_state.set_joint_group_positions("gripper", np.array([-0.7, 0.7]))


    bot_arm.set_start_state_to_current_state()
    bot_gripper.set_start_state_to_current_state()

    bot_arm.set_goal_state(robot_state=arm_state)
    bot_gripper.set_goal_state(robot_state=gripper_state)

    arm_plan_result = bot_arm.plan()
    grippe_plan_result = bot_gripper.plan()

    if arm_plan_result and grippe_plan_result:
        bot.execute(arm_plan_result.trajectory, controllers=[])
        bot.execute(grippe_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("One or more planners failed")


def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()

if __name__=="__main__":
    main()