#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from robot_msgs.action import RobotTask
import numpy as np
import time
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.action_server: ActionServer = ActionServer(self, RobotTask, 'task_server', self.goalCallback)
        self.__bot = MoveItPy(node_name="moveit_py")
        self.bot_arm = self.__bot.get_planning_component("arm")
        self.bot_gripper = self.__bot.get_planning_component("geipper")

    def goalCallback(self, goal_handle: ServerGoalHandle)-> RobotTask.Result | None:
        self.get_logger().info(f"Received goal request with task number {goal_handle.request.task_number}")
        feedback_msg = RobotTask.Feedback()
        arm_state = RobotState(self.__bot.get_robot_model())
        gripper_state = RobotState(self.__bot.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = [] 
    
        match goal_handle.request.task_number:
            case 0:
                arm_joint_goal = np.array([0.0, 0.0, 0.0])
                gripper_joint_goal = np.array([-0.7, 0.7]) 
            case 1:
                arm_joint_goal = np.array([-1.14, -0.35, -0.06])
                gripper_joint_goal = np.array([0.0, 0.0])
            case 2:
                arm_joint_goal = np.array([-1.57, -0.13, -1.0])
                gripper_joint_goal = np.array([0.0, 0.0])
            case _:
                self.get_logger().error("Invalid task number")
                return
        
        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("arm", gripper_joint_goal)

        self.bot_arm.set_start_state_to_current_state()
        self.bot_gripper.set_start_state_to_current_state()

        self.bot_arm.set_goal_state(robot_state=arm_state)
        self.bot_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result = self.bot_arm.plan()
        gripper_plan_result = self.bot_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.__bot.execute(arm_plan_result.trajectory, controllers=[])
            self.__bot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().info("One or more planner failed.")
        
        goal_handle.succeed()
        result = RobotTask.Result()
        result.success = True

        return result


def main():
    rclpy.init()
    task_server = TaskServer()
    rclpy.spin(task_server)
    task_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()