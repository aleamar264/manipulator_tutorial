#include <functional>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node {
public:
  SimpleSerialTransmitter() : Node("simple_serial_transmitter") {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    std::string port_ = get_parameter("port").as_string();
    sub_ = create_subscription<std_msgs::msg::String>(
        "serial_transmitter", 10,
        std::bind(&SimpleSerialTransmitter::msgCallback, this, _1));

    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }

  ~SimpleSerialTransmitter() { arduino_.Close(); };

  void msgCallback(const std_msgs::msg::String &msg) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "New message received, publishing on serial port"
                           << msg.data);
    arduino_.Write(msg.data);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  LibSerial::SerialPort arduino_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialTransmitter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}