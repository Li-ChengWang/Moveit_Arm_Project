from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mp_to_moveit_driver',
            executable='mp_to_moveit_driver_node',
            name='mp_to_moveit',
            output='screen',
            parameters=[{
                'group_name': 'arm',
                'end_effector_link': 'gripper_static_1',  # 不確定可留空 ""
                'planning_frame': 'base_link',
                'target_pose_topic': '/target_pose',
                'min_goal_translation_delta': 0.005,
                'min_goal_rotation_delta_deg': 3.0,
                'planning_time': 1.5,
                'max_velocity_scaling': 0.2,
                'max_acceleration_scaling': 0.2,
                'allow_execute': False,  # 先只 plan，安全
            }]
        )
    ])
