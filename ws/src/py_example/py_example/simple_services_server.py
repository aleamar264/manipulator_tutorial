import rclpy
from rclpy.node import Node
from robot_msgs.srv import AddTwoInts

class SimpleServicesServer(Node):
    def __init__(self):
        super().__init__(node_name="simple_servives_server")
        self.service = self.create_service(
            AddTwoInts, "add_two_ints", self.serviceCallback
        )
        self.get_logger().info("Service add_two_ints Ready")

    def serviceCallback(
        self, req: AddTwoInts.Request, res: AddTwoInts.Response
    ) -> AddTwoInts.Response:
        self.get_logger().info(f"New message received {req.a}, {req.b}")
        res.sum = req.a + req.b
        self.get_logger().info(f"Returning sum: {res.sum}")
        return res


def main():
    rclpy.init()
    simple_services_server = SimpleServicesServer()
    rclpy.spin(simple_services_server)
    simple_services_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()