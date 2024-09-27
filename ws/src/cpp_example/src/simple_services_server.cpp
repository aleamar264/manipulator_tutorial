#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <robot_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class AngleConverter : public rclcpp::Node
{
public:
    AngleConverter() : Node("simple_service_server")
    {
        service_ = create_service<robot_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&AngleConverter::serviceCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service add_two_ints is Ready");
    };

private:
    rclcpp::Service<robot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(const std::shared_ptr<robot_msgs::srv::AddTwoInts::Request> req, 
                        const std::shared_ptr<robot_msgs::srv::AddTwoInts::Response> res)
    {        
        RCLCPP_INFO_STREAM(this->get_logger(), "New Request Received a: " << req->a << "b:" << req->b);
        res->sum = req->a + req->b;
        RCLCPP_INFO_STREAM(this->get_logger(), "Returning sum: " << res->sum);
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