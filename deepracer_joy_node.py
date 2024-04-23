import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import yaml
import os
import csv
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

class DeepracerJoyNode(Node):
    def __init__(self):
        super().__init__('deepracer_joy_node')
        self.get_logger().info('DeepracerJoyNode started.')

        # Load YAML configuration
        config_path = os.path.join(
            get_package_share_directory('deepracer_joy'),
            'config',
            'logitech_dual_action.yaml'
        )
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Setup CSV file for recording data
        self.csv_file_path = os.path.expanduser('~/deepracer_ws/deepracer_joy/drive_data.csv')  # Simplified path
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'angle', 'throttle'])  # Column headers

        self.get_logger().info(f"Loaded configuration: {self.config}")

        # Initialize the publisher
        self.manual_drive_publisher = self.create_publisher(
            ServoCtrlMsg,
            '/webserver_pkg/manual_drive',
            10
        )

        # Initialize the service client
        self.enable_state_client = self.create_client(SetBool, '/ctrl_pkg/enable_state')

        # Subscribe to Joy messages
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )


    def joy_callback(self, msg):
        if msg.buttons[self.config['teleop']['drive']['deadman_buttons'][0]] == 1:
            servo_msg = ServoCtrlMsg()
            servo_msg.angle = msg.axes[3] * 1.0
            servo_msg.throttle = msg.axes[1] * 0.5

            # Publish the message
            self.manual_drive_publisher.publish(servo_msg)
            self.get_logger().info(f"Publishing: {servo_msg}")

            # Write to CSV
            self.csv_writer.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), servo_msg.angle, servo_msg.throttle])

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = DeepracerJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
