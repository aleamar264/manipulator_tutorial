#include <moveit_core/moveit/utils/moveit_error_code.h>
#include <moveit_ros_planning_interface/moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <robot_msgs/action/robot_task.hpp>
#include <rclcpp_action/rclcpp_action/server_goal_handle.hpp>
#include <thread>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace robot_remote
{
  class TaskServer : public rclcpp::Node
  {
  public:
    explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("task_server", options)
    {
      action_server_ = rclcpp_action::create_server<robot_msgs::action::RobotTask>(
          this,
          "task_server",
          std::bind(&TaskServer::goalCallback, this, _1, _2),
          std::bind(&TaskServer::cancelCallback, this, _1),
          std::bind(&TaskServer::acceptedCallback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Starting the Action server");
    }

  private:
    rclcpp_action::Server<robot_msgs::action::RobotTask>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const robot_msgs::action::RobotTask::Goal> goal)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with id:" << goal->task_number);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msgs::action::RobotTask>> goal_handle)
    {
      std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    };

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msgs::action::RobotTask>> goal_handle)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

      std::vector<double> arm_joint_goal;
      std::vector<double> gripper_joint_goal;
      
      if(goal_handle->get_goal()->task_number == 0){
        arm_joint_goal = {0.0, 0.0, 0.0};
        gripper_joint_goal = {-0.7, 0.7};
      }
      else if(goal_handle->get_goal()->task_number == 1){
          arm_joint_goal = {-1.14, -0.35, -0.06};
        gripper_joint_goal = {0.0, 0.0};
      }
      else if(goal_handle->get_goal()->task_number == 2){
          arm_joint_goal = {-1.57, -0.13, -1.0};
        gripper_joint_goal = {-0.0, 0.0};
      }
      else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid task Number");
        return;
      }

      bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
      bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
    
      if(!arm_within_bounds | !gripper_within_bounds){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target position were outside limits");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

      arm_move_group.setPlanningTime(10.0);
      gripper_move_group.setPlanningTime(10.0);

      auto arm_plan_result = arm_move_group.plan(arm_plan);

      bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
      bool gripeer_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "arm_plan " << arm_plan_success << " Arm bounds " << arm_within_bounds << " Arm plan code: " << arm_plan_result.val << " planning time " << arm_move_group.getPlanningTime());
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "gripper_plan " << gripeer_plan_success << " Gripper bounds " << gripper_within_bounds);


      if(arm_plan_success && gripeer_plan_success){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succedd, moving the arm and the gripper");
        arm_move_group.move();
        gripper_move_group.move();
      }
      else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planner failed");
        return;
      }
    auto result = std::make_shared<robot_msgs::action::RobotTask::Result>();
    result->success = true;

    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
    };

    rclcpp_action::CancelResponse cancelCallback(std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msgs::action::RobotTask>> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel the goal");
      auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
      auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
      //   (void)goal_handle;
      arm_move_group.stop();
      gripper_move_group.stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    };
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_remote::TaskServer)