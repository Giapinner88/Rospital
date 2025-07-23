import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class MotorStatusNode(Node):
    def __init__(self):
        super().__init__('motor_status_node')
        self.publisher = self.create_publisher(Float32MultiArray, '/motor_status', 10)
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.timer = self.create_timer(1.0, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            # Expected: "I0:1.23,I1:1.27"
            if line.startswith("I0"):
                values = line.split(',')
                i0 = float(values[0].split(':')[1])
                i1 = float(values[1].split(':')[1])
                msg = Float32MultiArray()
                msg.data = [i0, i1]
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()