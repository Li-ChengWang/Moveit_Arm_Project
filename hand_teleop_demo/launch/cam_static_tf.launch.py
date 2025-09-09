from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    x = LaunchConfiguration('x'); y = LaunchConfiguration('y'); z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll'); pitch = LaunchConfiguration('pitch'); yaw = LaunchConfiguration('yaw')
    parent = LaunchConfiguration('parent'); child = LaunchConfiguration('child')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.40'),
        DeclareLaunchArgument('y', default_value='0.00'),
        DeclareLaunchArgument('z', default_value='0.20'),
        DeclareLaunchArgument('roll', default_value='0.0'),     # 弧度
        DeclareLaunchArgument('pitch', default_value='-0.26'),  # ≈ -15°
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('parent', default_value='base_link'),
        DeclareLaunchArgument('child', default_value='camera_link'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cam_extrinsics',
            arguments=[
                '--x', x, '--y', y, '--z', z,
                '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                '--frame-id', parent, '--child-frame-id', child
            ]
        )
    ])