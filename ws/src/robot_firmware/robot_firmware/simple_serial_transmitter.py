#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.sub = self.create_subscription(String, 'serial_transmitter', self.msgCallback, 10)
        
        self.port = self.get_parameter("port").value
        self.baud_rate: int = self.get_parameter("baud_rate").value
        self.arduino_ = serial.Serial(port = self.port, baudrate=self.baud_rate, timeout=0.1)


    def msgCallback(self, msg: String):
        self.get_logger().info(f"New message received, publishing on serial {self.arduino_.name}")
        self.arduino_.write(msg.data.encode())

def main():
    rclpy.init()
    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()