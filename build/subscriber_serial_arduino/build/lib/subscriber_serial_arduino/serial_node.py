import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoSerialSubscriber(Node):
    def __init__(self):
        super().__init__('arduino_serial_subscriber')

        #################### Set parameters for serial connection ####################
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {port} at {baud} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return

        self.subscription = self.create_subscription(
            String,
            'servo_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        if command in ['S1', 'S2', 'O', 'C', 'STOP', 'END']:
            self.serial_conn.write((command + '\n').encode())
            self.get_logger().info(f'Sent: {command}')
            time.sleep(0.1)
            while self.serial_conn.in_waiting:
                response = self.serial_conn.readline().decode().strip()
                self.get_logger().info(f'Received: {response}')
        else:
            self.get_logger().warn(f'Invalid command: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

