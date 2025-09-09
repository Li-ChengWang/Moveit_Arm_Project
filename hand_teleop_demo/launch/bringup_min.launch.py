from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('koch_moveit_config')

    # ↓↓↓ 這個檔名依你的實際檔案改：是 v1_1 還是 v1.1
    urdf_xacro = PathJoinSubstitution([pkg, 'config', 'koch_v1_1.urdf.xacro'])
    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    # 用假硬體跑（沒有實機也能執行）
    robot_description = {'robot_description': Command(['xacro ', urdf_xacro, ' use_fake_hardware:=true'])}

    return LaunchDescription([
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[robot_description, controllers_yaml], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),
    ])
