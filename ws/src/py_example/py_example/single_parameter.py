import rclpy  # type:ignore
from rclpy.node import Node  # type:ignore
from rcl_interfaces.msg import SetParametersResult  # type:ignore
from rclpy.parameter import Parameter  # type:ignore


class SingleParameter(Node):
    def __init__(self):
        super().__init__("single_parameter")  # type:ignore
        self.declare_parameter("simple_int_param", 20)
        self.declare_parameter("simple_str_param", "")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params: list[Parameter]) -> SetParametersResult:
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("simple_int_param changed to {}".format(param.value))
                result.successful = True
            elif param.name == "simple_str_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("simple_str_param changed to {}".format(param.value))
                result.successful = True
        return result


def main(args=None):
    rclpy.init(args=args)
    simple_parameter = SingleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
