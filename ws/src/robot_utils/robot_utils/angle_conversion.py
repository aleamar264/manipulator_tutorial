#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.srv import EulerToQuaternion, QuaternionToEuler # type: ignore
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AnglesConverter(Node):
    def __init__(self):
        super().__init__(node_name="angle_conversion_service_server")
        self.euler_to_quaternion = self.create_service(
            EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback
        )
        self.quaternion_to_euler = self.create_service(
            QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback
        )
        
        self.get_logger().info("Angle conversion services are ready")

    def eulerToQuaternionCallback(
        self, req: EulerToQuaternion.Request, res: EulerToQuaternion.Response
    )->EulerToQuaternion:
        self.get_logger().info(f"Requested to convert euler angles roll: {req.roll}, pitch {req.pitch} and yaw {req.yaw}") 
        res.x, res.y, res.z, res.w = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info(f"Corresponding Quaternion x:{res.x}, y:{res.y}, z:{res.z} and w:{res.w}")
        return res

    def quaternionToEulerCallback(
        self, req: QuaternionToEuler.Request, res: QuaternionToEuler.Response
    )->QuaternionToEuler.Response:
        self.get_logger().info(f"Requested to convert quaternion Quaternion x:{req.x}, y:{req.y}, z:{req.z} and w:{req.w}")
        res.roll, res.pitch, res.yaw = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info(f"Corresponding Euler roll:{res.roll}, pitch:{res.pitch} and yaw:{res.yaw}")
        return res

def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()