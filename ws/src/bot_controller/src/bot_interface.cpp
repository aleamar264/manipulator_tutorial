#include "bot_controller/bot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace bot_controller
{

	std::string componsateZeros(int values)
	{
		std::string compensate_zeros = "";
		if (values < 10)
		{
			compensate_zeros = "00";
		}
		else if (values < 100)
		{
			compensate_zeros = "0";
		}
		return compensate_zeros;
	}
	BotInterface::BotInterface()
	{
	}

	BotInterface::~BotInterface()
	{
		if (bot_.IsOpen())
		{
			try
			{
				bot_.Close();
			}
			catch (...)
			{
				RCLCPP_FATAL_STREAM(rclcpp::get_logger("BotInterface"), "Something went wrong while closing the connection with port: " << port_);
			}
		}
	}

	CallbackReturn BotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
	{
		CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

		if (result != CallbackReturn::SUCCESS)
		{
			return result;
		}
		try
		{
			port_ = info_.hardware_parameters.at("port");
		}
		catch (const std::out_of_range &e)
		{
			RCLCPP_FATAL(rclcpp::get_logger("BotInterface"), "No serial Port provided! Aborting");
			return CallbackReturn::FAILURE;
		}
		position_commands_.reserve(info_.joints.size());
		position_state_.reserve(info_.joints.size());
		prev_position_commands_.reserve(info_.joints.size());

		return CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> BotInterface::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;
		for (size_t i = 0; i < info_.joints.size() ; i++)
		{
			state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state_[i]));
		}
		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> BotInterface::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		for (size_t i = 0; i < info_.joints.size() ; i++)
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
		}
		return command_interfaces;
	}

	CallbackReturn BotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
	{
		RCLCPP_INFO(rclcpp::get_logger("BotInterface"), "Starting the robot hardware ...");
		position_commands_ = {0.0, 0.0, 0.0, 0.0};
		prev_position_commands_ = {0.0, 0.0, 0.0, 0.0};
		position_state_ = {0.0, 0.0, 0.0, 0.0};
		try
		{
			bot_.Open(port_);
			bot_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
		}
		catch (...)
		{
			RCLCPP_FATAL_STREAM(rclcpp::get_logger("BotInterface"), "Something went wrong while interacting with the port " << port_);
			return CallbackReturn::FAILURE;
		}

		RCLCPP_INFO(rclcpp::get_logger("BotInterface"), "Starting started, ready to take commands");
		return CallbackReturn::SUCCESS;
	};
	CallbackReturn BotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
	{
		RCLCPP_INFO(rclcpp::get_logger("BotInterface"), "Stoppping the robot hardware ...");
		if (bot_.IsOpen())
		{
			try
			{
				bot_.Close();
			}
			catch (...)
			{
				RCLCPP_FATAL_STREAM(rclcpp::get_logger("BotInterface"), "Something went wrong while closing with the port " << port_);
				return CallbackReturn::FAILURE;
			}
			RCLCPP_INFO(rclcpp::get_logger("BotInterface"), "Hardware stop");
			return CallbackReturn::SUCCESS;
		}
	};

	hardware_interface::return_type BotInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
	{
		position_state_ = position_commands_;
		return hardware_interface::return_type::OK;
	};
	hardware_interface::return_type BotInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
	{
		if (position_commands_ == prev_position_commands_)
		{
			return hardware_interface::return_type::OK;
		}
		// serial communication for arduino
		// b43, s92,e30, g0
		// that means base:43 degree, shoulder: 92: degree, elbow: 30 degree, gripper: 0 degree
		std::string msg;
		int base = static_cast<int>(position_commands_.at(0) + (M_PI_2) * 180 / M_PI);
		msg.append("b");
		msg.append(componsateZeros(base));
		msg.append(std::to_string(base));
		msg.append(",");
		int shoulder = 180 - static_cast<int>(((position_commands_.at(1) + M_PI_2) * 180) / M_PI);
		msg.append("s");
		msg.append(componsateZeros(shoulder));
		msg.append(std::to_string(shoulder));
		msg.append(",");
		int elbow = static_cast<int>((position_commands_.at(2) + M_PI_2 * 180)/M_PI);
		msg.append("e");
		msg.append(componsateZeros(elbow));
		msg.append(std::to_string(elbow));
		msg.append(",");
		int gripper = (-position_commands_.at(3) * 180)/ M_PI_2;
		msg.append("g");
		msg.append(componsateZeros(gripper));
		msg.append(std::to_string(gripper));
		msg.append(",");
		try
		{
			bot_.Write(msg);
		}
		catch(...)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("BotInterface"), "Something went wrong while sending the message "<< msg << "to the port " << port_);
			return hardware_interface::return_type::ERROR;
		}
		prev_position_commands_ = position_commands_;
		return hardware_interface::return_type::OK;
		
	};

}

PLUGINLIB_EXPORT_CLASS(bot_controller::BotInterface, hardware_interface::SystemInterface);