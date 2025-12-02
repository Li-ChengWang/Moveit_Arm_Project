from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory # ★ 補上這個
from os.path import join  # ★ 補上這個
import os

def generate_launch_description():
    # 載入 MoveIt 設定
    mc = MoveItConfigsBuilder("koch_v1_1", package_name="koch_moveit_config") \
            .to_moveit_configs()

    # 1. Robot State Publisher (RSP)

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[mc.robot_description],
    )

    # 3-2. Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            mc.to_dict(),
            mc.trajectory_execution,
            mc.planning_pipelines,
            mc.robot_description_kinematics, # 這裡通常也建議加上
        ],
    )

    # 3-3. RViz (視覺化)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", join(get_package_share_directory('koch_moveit_config'), "config", "moveit.rviz")],
        parameters=[
            mc.robot_description,
            mc.robot_description_semantic,
            mc.planning_pipelines,
            mc.robot_description_kinematics, # ★ [修正] 原本寫 .kinematics 導致錯誤
            mc.joint_limits,
            mc.trajectory_execution,
        ],
        output="screen",
    )

    return LaunchDescription([rsp_node, move_group_node, rviz_node])