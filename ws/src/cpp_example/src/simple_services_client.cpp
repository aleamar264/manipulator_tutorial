#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <robot_msgs/srv/add_two_ints.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient : public rclcpp:: Node{
    public:
        SimpleServiceClient(int a, int b) : Node("simple_service_client")
        {
            client_ = create_client<robot_msgs::srv::AddTwoInts>("add_two_ints");
            auto request_ = std::make_shared<robot_msgs::srv::AddTwoInts::Request>();
            request_->a = a;
            request_->b = b;

            while(!client_->wait_for_service(1s)){
                RCLCPP_ERROR(this->get_logger(), "Service not available, waiting more thime ...");
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                    return ;
                }
            }
            auto result_ = client_->async_send_request(request_, std::bind(&SimpleServiceClient::responseCallback, this, _1));

        }
    private:
        rclcpp::Client<robot_msgs::srv::AddTwoInts>::SharedPtr client_;
        void responseCallback(rclcpp::Client<robot_msgs::srv::AddTwoInts>::SharedFuture future){
            if(future.valid()){
                RCLCPP_INFO_STREAM(this->get_logger(), "Service Response " << future.get()->sum);
            }else{
                RCLCPP_ERROR(this->get_logger(), "Service Failure");
            }
        };
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    if(argc != 3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments! Usage: simple_service client A B");
        return 1;
    }
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}