#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <robot_msgs/srv/euler_to_quaternion.hpp>
#include <robot_msgs/srv/quaternion_to_euler.hpp>
#include <tf2/utils.h>


using namespace std::placeholders;

class AngleConverter : public rclcpp::Node
{
public:
  AngleConverter() : Node("simple_service_server")
  {
    quaternion_to_euler = create_service<robot_msgs::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AngleConverter::quaternionToEuler, this, _1, _2));
    euler_to_quaternion = create_service<robot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AngleConverter::eulerToQuaternion, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "AngleConverter Services is Ready");
  };

private:
  rclcpp::Service<robot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler;
  rclcpp::Service<robot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion;

  void quaternionToEuler(const std::shared_ptr<robot_msgs::srv::QuaternionToEuler::Request> req,
                       const std::shared_ptr<robot_msgs::srv::QuaternionToEuler::Response> res)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Requested to convert quaternion Quaternion x:" << req->x << ",y" << req->y  << ",z" <<req->z  << "and w" << req->w);
    tf2::Quaternion q(req->x, req->y, req->z, req->w);
    tf2::Matrix3x3 m(q);
    m.getRPY(res->roll, res->pitch, res->yaw);
    RCLCPP_INFO_STREAM(this->get_logger(), "Corresponding Euler roll: "<< res->roll << ", pitch:"  <<res->pitch  <<" and yaw:" <<res->yaw);
  }

  void eulerToQuaternion(const std::shared_ptr<robot_msgs::srv::EulerToQuaternion::Request> req,
                       const std::shared_ptr<robot_msgs::srv::EulerToQuaternion::Response> res)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Requested to convert euler angles roll:" << req->roll << "pitch" << req->pitch  << "and yaw" << req->yaw);
    tf2::Quaternion q;
    q.setRPY(req->roll, req->pitch, req->pitch);
    res->x = q.getX();
    res->y = q.getY();
    res->z = q.getZ();
    res->w = q.getW();
    RCLCPP_INFO_STREAM(this->get_logger(), "Corresponding Quaternion x: "<< res->x << ", y:"  <<res->y  <<", z:" <<res->z  << "and w:"  <<res->w );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AngleConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};