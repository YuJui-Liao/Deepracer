import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from deepracer_interfaces_pkg.msg import ServoCtrlMsg  # Ensure this matches your message structure
import yaml
import os
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

        self.get_logger().info(f"Loaded configuration: {self.config}")

        # Initialize the publisher
        self.manual_drive_publisher = self.create_publisher(
            ServoCtrlMsg,
            '/webserver_pkg/manual_drive',
            10
        )

        # Initialize the service client (assuming it's needed based on your YAML)
        # Note: Ensure the service type and structure matches your actual service
        self.enable_state_client = self.create_client(SetBool, '/ctrl_pkg/enable_state')

        # Subscribe to Joy messages
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Check if the deadman button is pressed; adjust the index as needed
        if msg.buttons[self.config['teleop']['drive']['deadman_buttons'][0]] == 1:
            # Create a new ServoCtrlMsg based on joystick input
            servo_msg = ServoCtrlMsg()
            servo_msg.angle = msg.axes[3] * 1.0  # Adjust axis index and scale as necessary
            servo_msg.throttle = msg.axes[1] * 0.5  # Inverting the throttle as per your example

            # Publish the message
            self.manual_drive_publisher.publish(servo_msg)
            self.get_logger().info(f"Publishing: {servo_msg}")

            # Here you can also call the service if needed
            # You'll need to implement logic to call the enable_state service as appropriate

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
