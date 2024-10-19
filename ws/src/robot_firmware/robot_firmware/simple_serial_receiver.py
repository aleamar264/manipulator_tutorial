import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)       

        self.__port = self.get_parameter("port").value
        self.__baud_rate: int = self.get_parameter("baud_rate").value

        self.pub_ = self.create_publisher(String, 'serial_receiver', 10)
        self.__frecuency = 0.01
        self.__arduino = serial.Serial(port=self.__port, baudrate=self.__baud_rate)
        self.__timer = self.create_timer(self.__frecuency, self.timerCallback)


    def timerCallback(self):
        if rclpy.ok() and self.__arduino.is_open:
            data = self.__arduino.readline()
            try:
                data.decode()
            except Exception as err:
                return
            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)

def main():
    rclpy.init()
    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()