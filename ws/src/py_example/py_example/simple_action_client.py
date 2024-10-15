from rclpy.node import Node
import rclpy
from rclpy.action import ActionClient
from robot_msgs.action import Fibonacci
from rclpy.action.client import Future, ClientGoalHandle

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__("simple_action_server")
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")

        self.action_client.wait_for_server()
        self.goal = Fibonacci.Goal()
        self.goal.order = 10

        self.future: Future = self.action_client.send_goal_async(self.goal, self.feedbackCallback)
        self.future.add_done_callback(self.responseCallback)

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info(f"Received Feedback: {feedback_msg.feedback.partial_sequence}")

    def responseCallback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal Rejected ")
            return
        self.get_logger().info("Goal Accepted")
        self.future = (goal_handle.get_result_async().
                       add_done_callback(self.resultCallback))
        
    def resultCallback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f"Result {result.sequence}")
        rclpy.shutdown()

            


def main():
    rclpy.init()
    simple_action_client = SimpleActionClient()
    rclpy.spin(simple_action_client)
    # rclpy.shutdown()

if __name__== "__main__":
    main()