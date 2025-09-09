#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = 'mediapipe_pose_pub'
    pkg_share = get_package_share_directory(pkg)
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    # 1) RealSense 相機
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'rgb_camera.color_profile': '1280x720x30',
            'depth_module.depth_profile': '1280x720x30',
        }.items()
    )

    # 2) 靜態 TF（base_link → camera_link，camera_link → camera_color_optical_frame）
    base_to_cam = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_cam',
        # x y z yaw pitch roll frame_id child_frame_id
        arguments=['0.30', '0.00', '0.50', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )
    cam_to_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_cam_to_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_color_optical_frame'],
        output='screen'
    )

    # 3) MoveIt bringup（/move_group、RSP、ros2_control、RViz 由 koch_moveit_config 帶起）
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('koch_moveit_config'),
                         'launch', 'demo_ros2_control.launch.py')),
        launch_arguments={
            # 如果不想讓這個檔案自動開 RViz，改成：'use_rviz': 'false'
        }.items()
    )

    # 4) MediaPipe 節點
    mp_node = Node(
        package=pkg, executable='mediapipe_pose_publisher',
        name='mediapipe_pose_publisher',
        parameters=[params],
        output='screen'
    )

    # 5) 跟隨節點（只給自己的 params.yaml，不要再塞 moveit_cpp.yaml / wildcard）
    follower = Node(
        package=pkg, executable='moveit_pose_follower',
        name='moveit_pose_follower',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='🚀 Launching as Normal ROS Node'),
        realsense_launch,
        base_to_cam,
        cam_to_optical,
        moveit_demo,     # 先帶起 /move_group
        mp_node,
        follower,        # 再啟 follower（程式內會把 pipeline 參數塞進 /moveit_py）
    ])
