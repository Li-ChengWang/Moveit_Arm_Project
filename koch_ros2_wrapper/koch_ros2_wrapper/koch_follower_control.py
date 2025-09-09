import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot

import numpy as np
import yaml
import os

class KochROSWrapper(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('\033[93mMotor Controller Node started\033[0m')

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
        
        # Initialize follower arms based on config file
        self.follower_arms = {}
        self.joint_state_publishers = {}
        self.joint_state_subscribers = {}
        self.reset_zero_position_services = {}

        for arm_config in config['follower_arms']:
            arm_name = arm_config['name']
            port = arm_config['port']
            motors = arm_config['motors']
            self.get_logger().info(f'Initializing leader arm {arm_name} on port {port}')

            # Initialize Dynamixel motor bus for the arm
            self.follower_arms[arm_name] = DynamixelMotorsBus(
                port=port,
                motors={name: tuple(spec) for name, spec in motors.items()},
            )

            # Create a separate publisher and subscriber for each arm
            joint_state_topic = f'{arm_name}/joint_states'
            joint_state_control_topic = f'{arm_name}/joint_states_control'
            self.joint_state_publishers[arm_name] = self.create_publisher(JointState, joint_state_topic, 10)
            self.joint_state_subscribers[arm_name] = self.create_subscription(
                JointState, joint_state_control_topic, 
                lambda msg, arm_name=arm_name: self.cb_joint_state(msg, arm_name),
                10
            )

            # Create return zero position service for each arm
            service_name = f'{arm_name}_reset_position_to_zero'
            self.reset_zero_position_services[arm_name] = self.create_service(
                Trigger,
                service_name,
                lambda request, response, arm_name=arm_name: self.reset_position_to_zero_callback(request, response, arm_name)
            )


        # self.get_logger().info(f'Follower arms initialized: {self.follower_arms}')
        self.get_logger().info(f'load motor finish')

        try:
            self.robot = ManipulatorRobot(
                robot_type="koch",
                follower_arms=self.follower_arms,
                calibration_dir="/home/hrc/koch_robot_arm/calibration/koch",
            )
            # self.get_logger().info('Robot instance created, attempting to connect...')
            self.robot.connect()
            self.get_logger().info('\033[92mRobot successfully connected, you can start planning now\033[0m')

        except Exception as e:
            self.get_logger().error(f'\033[91mError during robot initialization or connection: {e}\033[0m')

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        # Iterate over each arm and publish its joint states independently
        try:
            for arm_name, arm in self.robot.follower_arms.items():
                pos_deg = np.array(arm.read("Present_Position"))
                pos_rad = np.radians(pos_deg)

                # Create JointState message for this arm
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [f"{arm_name}_{joint}" for joint in arm.motors.keys()]
                joint_state_msg.position = pos_rad.tolist()

                # Publish the JointState message for this arm
                self.joint_state_publishers[arm_name].publish(joint_state_msg)
        except:
            self.get_logger().warning(f'\033[93m Unable to read position data of follower.\033[0m')

    def cb_joint_state(self, msg, arm_name):
        # Convert received joint position data from radians to degrees
        unity_data_rad = np.array(msg.position)
        unity_data_deg = np.degrees(unity_data_rad)

        # Write the target position to the specified arm
        self.robot.follower_arms[arm_name].write("Goal_Position", unity_data_deg)

    def reset_position_to_zero_callback(self, request, response, arm_name):
        """Callback to set the specified arm's joint positions to zero."""
        try:
            arm = self.follower_arms[arm_name]
            zero_position = [0] * len(arm.motors)  # Create a zero position for each motor
            arm.write("Goal_Position", zero_position)  # Send zero positions to the arm
            self.get_logger().info(f'Set zero position for arm {arm_name}.')
            response.success = True
            response.message = f"{arm_name} arm position set to zero."
        except Exception as e:
            self.get_logger().error(f"\033[91mFailed to set zero position for {arm_name}: {e}\033[0m")
            response.success = False
            response.message = f"Error setting zero position for {arm_name}: {e}"
        return response
    

    #override the distory_node function of ros2 for disconnect robot first
    def destroy_node(self): 
        # #1. Call each arm's reset-to-zero service
        # self.get_logger().info("Setting all arm positions to zero before shutdown...")

        # for arm_name in self.follower_arms.keys():
        #     service_name = f'{arm_name}_reset_position_to_zero'
        #     reset_client = self.create_client(Trigger, service_name)
            
        #     if reset_client.wait_for_service(timeout_sec=3.0):
        #         future = reset_client.call_async(Trigger.Request())
        #         rclpy.spin_until_future_complete(self, future)
        #         if future.result() is not None and future.result().success:
        #             self.get_logger().info(f"Service call succeeded for {arm_name}: {future.result().message}")
        #         else:
        #             self.get_logger().error(f"\033[91mService call failed for {arm_name} or did not succeed.\033[0m")
        #     else:
        #         self.get_logger().error(f"\033[91mService '{service_name}' not available.\033[0m")

        # #2. Disable torque on all arms before disconnecting
        # self.get_logger().info('Disabling torque on all arms...')
        # if hasattr(self, 'follower_arms') and self.follower_arms:
        #     for arm_name, arm in self.follower_arms.items():
        #         try:
        #             for motor_name in arm.motors.keys():
        #                 arm.write("Torque_Enable", 0, motor_name)  # Disable torque
        #             self.get_logger().info(f'Torque disabled for arm {arm_name}.')
        #         except Exception as e:
        #             self.get_logger().error(f'\033[91mError disabling torque for arm {arm_name}: {e}\033[0m')
                    
        #3. Disconnect the robot
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
    motor_controller_node = KochROSWrapper()
    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        motor_controller_node.destroy_node()
        motor_controller_node.get_logger().info("Keyboard Interrupt (Ctrl+C) received, shutting down.")
    finally:
        rclpy.shutdown()
if __name__ == '__main__':
    main()
