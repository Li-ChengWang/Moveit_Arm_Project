import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot

import numpy as np
import yaml

class KochLeaderFollower(Node):
    def __init__(self):
        super().__init__('leader_follower_control_node')
        self.get_logger().info('\033[93mLeader-Follower Control Node started\033[0m')

        # Declare the parameter for the config file path
        self.declare_parameter('config_file', '/home/hrc/koch_robot_arm/ros2_ws/src/koch_ros2_wrapper/config/single_leader_follower.yaml')
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value

        # Load configuration from the specified YAML file
        try:
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
                self.get_logger().info(f'Loaded configuration from {config_file_path}')
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration file: {e}")
            return

        # Initialize leader and follower arms
        self.leader_arms = {}
        self.follower_arms = {}
        self.joint_state_publishers = {}
        self.joint_state_subscribers = {}

        # Initialize leader arms (publish-only)
        for arm_config in config['leader_arms']:
            arm_name = arm_config['name']
            port = arm_config['port']
            motors = arm_config['motors']
            self.get_logger().info(f'Initializing leader arm {arm_name} on port {port}')

            self.leader_arms[arm_name] = DynamixelMotorsBus(
                port=port,
                motors={name: tuple(spec) for name, spec in motors.items()},
            )

            # Create publisher for leader arm joint states
            joint_state_topic = f'{arm_name}/joint_states'
            self.joint_state_publishers[arm_name] = self.create_publisher(JointState, joint_state_topic, 10)

        # Initialize follower arms (publish and subscribe)
        for arm_config in config['follower_arms']:
            arm_name = arm_config['name']
            port = arm_config['port']
            motors = arm_config['motors']
            self.get_logger().info(f'Initializing follower arm {arm_name} on port {port}')

            self.follower_arms[arm_name] = DynamixelMotorsBus(
                port=port,
                motors={name: tuple(spec) for name, spec in motors.items()},
            )

            # Create publisher and subscriber for follower arm
            joint_state_topic = f'{arm_name}/joint_states'
            joint_state_control_topic = f'{arm_name}/joint_states_control'
            self.joint_state_publishers[arm_name] = self.create_publisher(JointState, joint_state_topic, 10)
            self.joint_state_subscribers[arm_name] = self.create_subscription(
                JointState, joint_state_control_topic, 
                lambda msg, arm_name=arm_name: self.cb_joint_state(msg, arm_name),
                10
            )

        self.get_logger().info('Leader and Follower arms initialized')
        self.get_logger().info(f'Leader Arms: {self.leader_arms}')
        self.get_logger().info(f'Follower Arms: {self.follower_arms}')

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
            self.get_logger().error(f'\033[93mError during robot initialization or connection: {e}\033[0m')

        # Timer to publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_joint_states)

    def publish_joint_states(self):
        # Publish leader arm joint states
        for arm_name, arm in self.robot.leader_arms.items():
            try:
                pos_deg = np.array(arm.read("Present_Position"))
                pos_rad = np.radians(pos_deg)

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [f"{arm_name}_{joint}" for joint in arm.motors.keys()]
                joint_state_msg.position = pos_rad.tolist()

                self.joint_state_publishers[arm_name].publish(joint_state_msg)
            except:
                self.get_logger().warning(f'\033[93m Unable to read position data of leader.\033[0m')

        # Publish follower arm joint states
        for arm_name, arm in self.robot.follower_arms.items():
            try:
                pos_deg = np.array(arm.read("Present_Position"))
                pos_rad = np.radians(pos_deg)

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [f"{arm_name}_{joint}" for joint in arm.motors.keys()]
                joint_state_msg.position = pos_rad.tolist()

                self.joint_state_publishers[arm_name].publish(joint_state_msg)
            except:
                self.get_logger().warning(f'\033[93m Unable to read position data of follower.\033[0m')


    def cb_joint_state(self, msg, arm_name):
        # Callback for follower arm joint states (subscribe)
        unity_data_rad = np.array(msg.position)
        unity_data_deg = np.degrees(unity_data_rad)

        # Write the target position to the specified arm
        self.robot.follower_arms[arm_name].write("Goal_Position", unity_data_deg)

    #override the distory_node function of ros2 for disconnect robot first
    def destroy_node(self):
        #1. Disable torque on all arms before disconnecting
        self.get_logger().info('Disabling torque on all arms...')
        if hasattr(self, 'follower_arms') and self.follower_arms:
            for arm_name, arm in self.follower_arms.items():
                try:
                    for motor_name in arm.motors.keys():
                        arm.write("Torque_Enable", 0, motor_name)  # Disable torque
                    self.get_logger().info(f'Torque disabled for arm {arm_name}.')
                except Exception as e:
                    self.get_logger().error(f'\033[91mError disabling torque for arm {arm_name}: {e}\033[0m')
                    
        #2. Disconnect the robot
        self.get_logger().info('Shutting down the node and disconnecting the robot...')
        if hasattr(self, 'robot') and self.robot:
            try:
                self.robot.disconnect()
                self.get_logger().info('Robot successfully disconnected.')
            except Exception as e:
                self.get_logger().error(f'\033[91mError disconnecting robot: {e}\033[0m')
        super().destroy_node()  # Call parent class destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = KochLeaderFollower()
    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        motor_controller_node.destroy_node()
        motor_controller_node.get_logger().info("Keyboard Interrupt (Ctrl+C) received, shutting down.")
    finally:
        rclpy.shutdown()
if __name__ == '__main__':
    main()
