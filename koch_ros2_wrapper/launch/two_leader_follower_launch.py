from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: koch_leader_follower
        Node(
            package='koch_ros2_wrapper',
            executable='koch_leader_follower',
            name='koch_leader_follower',
            output='screen',
            parameters=[{
                'config_file': '/home/hrc/koch_robot_arm/ros2_ws/src/koch_ros2_wrapper/config/two_leader_follower.yaml'
            }]
        ),

        # Node 2: Relay left joint states
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_left',
            output='log',
            arguments=['/left_leader/joint_states', '/left_follower/joint_states_control']
        ),

        # Node 3: Relay right joint states
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_right',
            output='log',
            arguments=['/right_leader/joint_states', '/right_follower/joint_states_control']
        ),
    ])
