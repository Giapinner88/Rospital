import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import serial

class BatteryStatusNode(Node):
    def __init__(self):
        super().__init__('battery_status')
        self.publisher = self.create_publisher(BatteryState, '/battery_status', 10)

        # Serial kết nối với Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

        self.timer = self.create_timer(1.0, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("Vbat"):
                # Vbat:35.4,Iload:1.20
                parts = line.split(',')
                vbat = float(parts[0].split(':')[1])
                iload = float(parts[1].split(':')[1])

                msg = BatteryState()
                msg.voltage = vbat
                msg.current = iload
                msg.percentage = min(vbat / 42.0, 1.0)  # Giả định pin 36V max = 42V
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

                self.publisher.publish(msg)
                self.get_logger().info(f'Vbat: {vbat:.2f}V | Iload: {iload:.2f}A')

        except Exception as e:
            self.get_logger().warn(f'Lỗi đọc serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
