import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot

import numpy as np
import yaml

class KochCalibration(Node):
    def __init__(self):
        super().__init__('motor_calibration_node')
        self.get_logger().info('\033[93mMotor Calibration Node started\033[0m')

        # Declare the parameter for the config file path
        self.declare_parameter('config_file', '/home/hrc/koch_robot_arm/ros2_ws/src/koch_ros2_wrapper/config/dual_arm.yaml')
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value

        # Load configuration from the specified YAML file
        try:
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
                self.get_logger().info(f'Loaded configuration from {config_file_path}')
        except Exception as e:
            self.get_logger().error(f"\033[91mFailed to load configuration file: {e}\033[0m")
            return
        
        # Initialize arms based on configuration
        self.leader_arms = {}
        self.follower_arms = {}

        # Initialize leader arms
        for arm_config in config.get('leader_arms', []):
            arm_name = arm_config['name']
            port = arm_config['port']
            motors = arm_config['motors']
            self.get_logger().info(f'Initializing leader arm {arm_name} on port {port}')

            # Initialize Dynamixel motor bus for the leader arm
            self.leader_arms[arm_name] = DynamixelMotorsBus(
                port=port,
                motors={name: tuple(spec) for name, spec in motors.items()},
            )

        # Initialize follower arms
        for arm_config in config.get('follower_arms', []):
            arm_name = arm_config['name']
            port = arm_config['port']
            motors = arm_config['motors']
            self.get_logger().info(f'Initializing follower arm {arm_name} on port {port}')

            # Initialize Dynamixel motor bus for the follower arm
            self.follower_arms[arm_name] = DynamixelMotorsBus(
                port=port,
                motors={name: tuple(spec) for name, spec in motors.items()},
            )

        try:
            self.robot = ManipulatorRobot(
                robot_type="koch",
                leader_arms=self.leader_arms,
                follower_arms=self.follower_arms,
                calibration_dir="/home/hrc/koch_robot_arm/calibration/koch",
            )
            self.robot.connect()
            self.get_logger().info('\033[92mRobot successfully connected\033[0m')
        except Exception as e:
            self.get_logger().error(f'\033[91mError during robot initialization or connection: {e}\033[0m')
            
    # Override the destroy_node function of ROS 2 to disconnect the robot first
    def destroy_node(self):
        # 1. Disable torque on all arms before disconnecting
        self.get_logger().info('Disabling torque on all arms...')
        
        # Disable torque for leader arms
        if hasattr(self, 'leader_arms') and self.leader_arms:
            for arm_name, arm in self.leader_arms.items():
                try:
                    for motor_name in arm.motors.keys():
                        arm.write("Torque_Enable", 0, motor_name)  # Disable torque
                    self.get_logger().info(f'Torque disabled for leader arm {arm_name}.')
                except Exception as e:
                    self.get_logger().error(f'\033[91mError disabling torque for leader arm {arm_name}: {e}\033[0m')
        
        # Disable torque for follower arms
        if hasattr(self, 'follower_arms') and self.follower_arms:
            for arm_name, arm in self.follower_arms.items():
                try:
                    for motor_name in arm.motors.keys():
                        arm.write("Torque_Enable", 0, motor_name)  # Disable torque
                    self.get_logger().info(f'Torque disabled for follower arm {arm_name}.')
                except Exception as e:
                    self.get_logger().error(f'\033[91mError disabling torque for follower arm {arm_name}: {e}\033[0m')
        
        # 2. Disconnect the robot
        self.get_logger().info('Shutting down the node and disconnecting the robot...')
        if hasattr(self, 'robot') and self.robot:
            try:
                self.robot.disconnect()
                self.get_logger().info('\033[92mRobot successfully disconnected.\033[0m')
            except Exception as e:
                self.get_logger().error(f'\033[91mError disconnecting robot: {e}\033[0m')
        
        super().destroy_node()  # Call parent class destroy_node()


def main(args=None):
    rclpy.init(args=args)
    calibration_node = KochCalibration()
    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
