import rclpy
from rclpy.node import Node
from robot_msgs.srv import AddTwoInts
from rclpy.task import Future
import sys

class SimpleServiceClient(Node):
    def __init__(self, a:int, b:int):
        super().__init__(node_name="simple_service_client")

        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting more...")
        
        self.req_: AddTwoInts.Request = AddTwoInts.Request()
        self.req_.a = a
        self.req_.b = b
        self.future_: Future = self.client_.call_async(self.req_)
        self.future_.add_done_callback(self.responseCallback)

    
    def responseCallback(self, future: Future):
        self.get_logger().info(f"Service Response {future.result().sum}")

def main():
    rclpy.init()
    if len(sys.argv)!= 3:
        print("Wrong number of arguments! Usage: simple_services_client A B")
        return -1
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()