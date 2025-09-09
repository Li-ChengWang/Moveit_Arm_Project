from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RealSense 節點（開對齊、可改 profile）
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        parameters=[{
            'camera_name': 'camera',
            'align_depth.enable': True,         # 關鍵：深度對齊彩色
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
        }]
    )

    # base_link → camera_link 的靜態外參（先給一個可用的估值）
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x','0.40','--y','0.00','--z','0.20',
            '--roll','0.0','--pitch','-0.26','--yaw','0.0',
            '--frame-id','base_link','--child-frame-id','camera_link'
        ],
        name='cam_extrinsics'
    )

    return LaunchDescription([realsense, static_tf])
