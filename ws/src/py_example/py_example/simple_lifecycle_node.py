import rclpy
import rclpy.executors
from rclpy.lifecycle import Node
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String
import time

class SimpleLifecycleNode(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
    

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.__sub = self.create_subscription(String, "chatter", self.msgCallback, 10)
        self.get_logger().info("Lifecycle Node on_configure() called.")
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle Node on_activate() called")
        time.sleep(2)
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle Node on_activate() called")
        return super().on_deactivate(state)
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_subscription(self.__sub)
        self.get_logger().info("Lifecycle Node on_shutdown() destroyed.")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_subscription(self.__sub)
        self.get_logger().info("Lifecycle Node on_cleanup() called.")
        return TransitionCallbackReturn.SUCCESS
    

    def msgCallback(self, msg: String):
        if (current_state := self._state_machine.current_state)[1] == "active":
            self.get_logger().info(f"I heard {msg.data}")


def main():
    rclpy.init()
    executor =  rclpy.executors.SingleThreadedExecutor()
    simple_lifecycle_node = SimpleLifecycleNode("simple_life_cycle_node")
    executor.add_node(simple_lifecycle_node)
    try:
        executor.spin()
    except( KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        simple_lifecycle_node.destroy_node()
if __name__=="__main__":
    main()
