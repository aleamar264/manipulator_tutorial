#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_msgs/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>

using namespace std::chrono_literals;
using Fibonnacci = robot_msgs::action::Fibonacci;
using namespace std::placeholders;

namespace cpp_example
{

	class SimpleActionClient : public rclcpp::Node
	{
	public:
		explicit SimpleActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("simple_action_clietn", options)
		{
			client_ = rclcpp_action::create_client<Fibonnacci>(this, "fibonacci");
			timer_ = create_wall_timer(1s, std::bind(&SimpleActionClient::timerCallback, this));
		}

	private:
		rclcpp_action::Client<Fibonnacci>::SharedPtr client_;
		rclcpp::TimerBase::SharedPtr timer_;

		void timerCallback()
		{
			timer_->cancel();
			if (!client_->wait_for_action_server())
			{
				RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
				rclcpp::shutdown();
			}

			auto goal_msgs = Fibonnacci::Goal();
			goal_msgs.order = 10;
			RCLCPP_INFO(this->get_logger(), "Sending Goal");
			auto send_goal_options = rclcpp_action::Client<Fibonnacci>::SendGoalOptions();
			send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goalCallback, this, _1);
			send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);
			send_goal_options.result_callback = std::bind(&SimpleActionClient::resultCallback, this, _1);

			client_->async_send_goal(goal_msgs, send_goal_options);
		}

		void goalCallback(const rclcpp_action::ClientGoalHandle<Fibonnacci>::SharedPtr goal_handle)
		{
			if (!goal_handle)
			{
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "Goal accpeted by the server, waiting for result");
			}
		};

		void feedbackCallback(const rclcpp_action::ClientGoalHandle<Fibonnacci>::SharedPtr goal_handle, const std::shared_ptr<const Fibonnacci::Feedback> feedback)
		{
			std::stringstream ss;
			ss << "Next number in sequence received is:";
			for(auto number: feedback->partial_sequence)
			{
				ss << number << " ";
			}
			RCLCPP_INFO(this->get_logger(), ss.str().c_str());
		};

	void resultCallback(const rclcpp_action::ClientGoalHandle<Fibonnacci>::WrappedResult& result)
	{
		switch (result.code)
		{
		case rclcpp_action::ResultCode::SUCCEEDED:
			break;
		case rclcpp_action::ResultCode::ABORTED :
			RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
		return;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
			return;
		default:
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			return;
		}

			std::stringstream ss;
			ss << "Next number in sequence received is:";
			for(auto number: result.result->sequence)
			{
				ss << number << " ";
			}
			RCLCPP_INFO(this->get_logger(), ss.str().c_str());
			rclcpp::shutdown();
	};


	};


}

RCLCPP_COMPONENTS_REGISTER_NODE(cpp_example::SimpleActionClient)