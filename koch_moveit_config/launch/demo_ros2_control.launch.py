from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    mc = MoveItConfigsBuilder("koch_v1_1", package_name="koch_moveit_config") \
            .to_moveit_configs()

    # 讀既有的 ros2_control 控制器設定
    controllers_yaml = os.path.join(mc.package_path, "config", "ros2_controllers.yaml")

    # 1) ros2_control_node（/controller_manager）+ RSP
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[mc.robot_description, controllers_yaml],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[mc.robot_description],
        output="screen",
    )

    # 2) Spawners（加延遲，確保 manager 已就緒）
    spawner_jsc = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        )]
    )
    spawner_arm = TimerAction(
        period=3.5,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["arm_controller", "-c", "/controller_manager"],  # 名稱要與 YAML 一致
            output="screen",
        )]
    )

    # 3) move_group（明確指定使用 ros2_control；固定節點名，避免命名空間混淆）
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace="",
        output="screen",
        parameters=[
            mc.to_dict(),
            {"moveit_controller_manager": "moveit_ros_control_interface/Ros2ControlManager"},  # ← 改這行
            {"ros2_control_namespace": ""},  # 若 manager 在 /koch/controller_manager，這裡要設 "/koch"
    ],
)


    # 4) RViz（帶齊 MoveIt 參數）
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(mc.package_path, "config", "moveit.rviz")],
        parameters=[
            mc.robot_description,
            mc.robot_description_semantic,
            mc.robot_description_kinematics,
            mc.planning_pipelines,
            mc.joint_limits,
            mc.trajectory_execution,
        ],
        output="screen",
    )

    return LaunchDescription([ros2_control, rsp, spawner_jsc, spawner_arm, move_group, rviz])
