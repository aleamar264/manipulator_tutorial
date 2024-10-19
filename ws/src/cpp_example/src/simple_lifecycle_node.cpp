#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using lifecycle = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using std::placeholders::_1;
using namespace std::chrono_literals;

class SimpleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit SimpleLifecycleNode(const std::string &node_name, bool intra_process_comms = false)
        : LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
    }
    lifecycle::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleLifecycleNode::msgCallback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Lifecycle node on_configure() called");
        return lifecycle::CallbackReturn::SUCCESS;
    }

    lifecycle::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle node on_shutdown() called");
        return lifecycle::CallbackReturn::SUCCESS;
    }

    lifecycle::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Lifecycle node on_cleanup() called");
        return lifecycle::CallbackReturn::SUCCESS;
    }


    lifecycle::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);
        RCLCPP_INFO(this->get_logger(), "Lifecycle node on_activate() called");
        std::this_thread::sleep_for(2s);
        return lifecycle::CallbackReturn::SUCCESS;
    }
    
    lifecycle::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);
        RCLCPP_INFO(this->get_logger(), "Lifecycle node on_deactivate() called");
        return lifecycle::CallbackReturn::SUCCESS;
    }
    void msgCallback(const std_msgs::msg::String &msg) {
        auto state = get_current_state();
        if(state.label()=="activate"){
            RCLCPP_INFO_STREAM(this->get_logger(), "Lifecycle node heard: " << msg.data.c_str());
        }
        
            };

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor ste;
    auto simple_lifecycle_node = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node");
    ste.add_node(simple_lifecycle_node->get_node_base_interface());
    ste.spin();
    rclcpp::shutdown();
    return 0;
}