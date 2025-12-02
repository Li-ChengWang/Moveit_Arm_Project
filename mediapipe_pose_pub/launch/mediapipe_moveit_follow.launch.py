#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from os.path import join

def generate_launch_description():
    pkg = 'mediapipe_pose_pub'
    share = get_package_share_directory(pkg)

    params_main = join(share, 'config', 'params.yaml')

    # ① MoveIt 設定：給 driver 用（提供 robot_description / semantic）
    moveit_config = (
        MoveItConfigsBuilder("koch_v1_1", package_name="koch_moveit_config")
        .robot_description(file_path="config/koch_v1_1.urdf.xacro")
        .robot_description_semantic(file_path="config/koch_v1_1.srdf")
        .planning_pipelines(pipelines=["ompl","stomp","chomp","pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )


    # ② RealSense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'rgb_camera.color_profile': '1280x720x30',
            'depth_module.depth_profile': '1280x720x30',
        }.items()
    )

    # ③ 靜態 TF
    base_to_cam = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_cam',
        arguments=['0.0', '0.8', '0.0', '-1.5708', '0', '0', 'low_cost_robot/base_link', 'camera_link'],
        output='screen'
    )
    
    cam_to_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_cam_to_optical',
        arguments=['0','0','0','-1.5708','0','-1.5708','camera_link','camera_color_optical_frame'],
        output='screen'
    )
        # ③-2 把舊的 low_cost_robot/base_link 接回來（避免 MoveIt 抱怨兩棵樹）
    low_cost_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_low_cost_to_base',
        # 0 位移 / 0 角度：直接當成同一個點
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'low_cost_robot/base_link'],
        output='screen'
    )


    # ④ MoveIt bringup（這裡面已包含 move_group 與控制器）
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('koch_moveit_config'), 'launch', 'demo_ros2_control.launch.py')
        )
    )

    # ⑤ Mediapipe Publisher
    mp_node = Node(
        package=pkg, executable='mediapipe_pose_publisher',
        name='mediapipe_pose_publisher',
        parameters=[params_main],
        output='screen'
    )

    # ⑥ C++ MoveIt Driver（唯一一個；把 moveit_config 帶進來）
    mp_to_moveit = Node(
        package='mp_to_moveit_driver',
        executable='mp_to_moveit_driver_node',
        name='mp_to_moveit',
        output='screen',
        parameters=[
            moveit_config.to_dict(),            # ★ 關鍵：提供 robot_description(_semantic)
            {
                'use_sim_time': False,
            },
            {
                'group_name': 'arm',
                # 若 driver 有需要再加，沒有就先不要塞錯鍵
                'end_effector_link': 'gripper_static_1',
                'plan_frame': 'base_link',      # ★ 注意鍵名：plan_frame（非 planning_frame）
                'target_topic': '/target_pose', # ★ 注意鍵名：target_topic（非 target_pose_topic）
                'planning_time': 1.5,
                'max_velocity_scaling': 0.2,
                'max_acceleration_scaling': 0.2,
                'allow_execute': True,         # 先規劃不執行
                'min_goal_translation_delta': 0.005,
                'min_goal_rotation_delta_deg': 3.0,

            }
        ]
    )

    # （選配）影像監看
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen'
    )

    # （可選）等 move_group 起來再啟 driver，降低參數還沒就緒的風險
    delayed_driver = TimerAction(period=2.0, actions=[mp_to_moveit])

    return LaunchDescription([
        LogInfo(msg='🚀 Launching Mediapipe + MoveIt (C++ driver)'),
        realsense_launch,
        base_to_cam,
        cam_to_optical,
        low_cost_to_base,
        moveit_demo,       # 只保留這個，別再另外手動起 move_group
        mp_node,
        delayed_driver,    # 稍微延遲 2 秒讓 move_group 先穩定
        image_view,
    ])
