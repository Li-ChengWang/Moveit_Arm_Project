#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    pkg = 'mediapipe_pose_pub'
    share = get_package_share_directory(pkg)

    params_main = os.path.join(share, 'config', 'params.yaml')

    # 1) RealSense（維持你原本設定）
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

    # 2) 靜態 TF（維持你原本設定）
    base_to_cam = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_cam',
        arguments=['0.30','0.00','0.50','0','0','0','base_link','camera_link'],
        output='screen'
    )
    cam_to_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_cam_to_optical',
        arguments=['0','0','0','-1.5708','0','-1.5708','camera_link','camera_color_optical_frame'],
        output='screen'
    )

    # 3) MoveIt bringup（維持你原本設定）
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('koch_moveit_config'),
                         'launch', 'demo_ros2_control.launch.py'))
    )

    # 4) Mediapipe publisher（維持你原本設定與參數）
    mp_node = Node(
        package=pkg, executable='mediapipe_pose_publisher',
        name='mediapipe_pose_publisher',
        parameters=[params_main],
        output='screen'
    )

    # 5) **新方法**：C++ MoveIt driver（取代舊的 moveit_py follower）
    cpp_driver = Node(
        package='mp_to_moveit_driver',
        executable='mp_to_moveit_driver_node',
        name='mp_to_moveit',
        output='screen',
        parameters=[{
            'group_name': 'arm',
            'end_effector_link': 'gripper_static_1',   # 不確定可留空 ""
            'planning_frame': 'base_link',
            'target_pose_topic': '/target_pose',
            'min_goal_translation_delta': 0.005,
            'min_goal_rotation_delta_deg': 3.0,
            'planning_time': 1.5,
            'max_velocity_scaling': 0.2,
            'max_acceleration_scaling': 0.2,
            'allow_execute': False,                    # 先只規劃，安全
        }]
    )

    # （選配）影像監看
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='🚀 Launching Mediapipe + MoveIt (C++ driver)'),
        realsense_launch,
        base_to_cam,
        cam_to_optical,
        moveit_demo,     # 先起 /move_group 與 robot_description
        mp_node,
        cpp_driver,      # 用 C++ driver 取代舊 follower
        image_view,
    ])
