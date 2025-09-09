import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import yaml

class VRArmControl(Node):
    def __init__(self):
        super().__init__('vr_arm_control_node')
        self.get_logger().info('\033[93mVR Arm Control Node started\033[0m')

        # Declare the parameter for the config file path
        self.declare_parameter('config_file', '/home/hrc/koch_robot_arm/ros2_ws/src/koch_ros2_wrapper/config/single_follower.yaml')
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value

        # Load configuration from the specified YAML file
        try:
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
                self.get_logger().info(f'Loaded configuration from {config_file_path}')
        except Exception as e:
            self.get_logger().error(f"\033[91mFailed to load configuration file: {e}\033[0m")
            return

        # Initialize publishers and subscribers from config
        self.joint_state_subscribers = {}
        self.joint_state_publishers = {}

        for arm_config in config['follower_arms']:
            arm_name = arm_config['name']

            # Create a subscriber for the VR input data
            vr_input_topic = f'{arm_name}/joint_states_control_vr'
            self.joint_state_subscribers[arm_name] = self.create_subscription(
                JointState,
                vr_input_topic,
                lambda msg, arm_name=arm_name: self.cb_joint_state_vr(msg, arm_name),
                10
            )

            # Create a publisher for the corrected joint data
            control_output_topic = f'{arm_name}/joint_states_control'
            self.joint_state_publishers[arm_name] = self.create_publisher(JointState, control_output_topic, 10)

            self.get_logger().info(f"Initialized VR input topic '{vr_input_topic}' and control output topic '{control_output_topic}' for arm '{arm_name}'")

    def cb_joint_state_vr(self, msg, arm_name):
        """
        Callback for VR input data. Adjusts positions of joints 1 and 2 by +90 degrees (π/2 radians)
        and publishes corrected data to the control topic.
        """
        try:
            # Convert incoming positions to numpy array for easy manipulation
            joint_positions = np.array(msg.position)

            # Add 90 degrees (π/2 radians) to joints 1 and 2
            joint_positions[1] += np.pi / 2
            joint_positions[2] += np.pi / 2

            # Create and populate corrected JointState message
            corrected_msg = JointState()
            corrected_msg.header = msg.header  # Copy header from input message
            corrected_msg.name = msg.name      # Copy joint names
            corrected_msg.position = joint_positions.tolist()  # Updated positions
            corrected_msg.velocity = []  # Copy velocity (if any)
            corrected_msg.effort = []      # Copy effort (if any)

            # Publish the corrected message
            self.joint_state_publishers[arm_name].publish(corrected_msg)
            self.get_logger().info(f"Published corrected joint positions for {arm_name}")

        except Exception as e:
            self.get_logger().error(f"\033[91mError in processing VR input for {arm_name}: {e}\033[0m")

def main(args=None):
    rclpy.init(args=args)
    vr_arm_control_node = VRArmControl()

    try:
        rclpy.spin(vr_arm_control_node)
    except KeyboardInterrupt:
        vr_arm_control_node.get_logger().info("Keyboard Interrupt (Ctrl+C) received, shutting down.")
    finally:
        vr_arm_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
