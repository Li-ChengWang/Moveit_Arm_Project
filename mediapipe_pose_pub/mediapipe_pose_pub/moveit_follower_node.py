#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# 新版 MoveIt 2 Python API
from moveit.planning import MoveItPy

from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import (
    ParameterType,
    Parameter,
    ParameterValue,
)
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory


def quat_angle_deg(q1, q2) -> float:
    dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
    dot = max(min(dot, 1.0), -1.0)
    return math.degrees(2.0 * math.acos(abs(dot)))

def pose_distance(p1, p2) -> float:
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # latched
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

def bridge_move_group_descriptions(node) -> bool:
    """從 /move_group 讀 URDF/SRDF，轉發到 latched topic，讓 MoveItPy 能吃到。"""
    client = node.create_client(GetParameters, '/move_group/get_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('等待 /move_group/get_parameters 超時（5s）')
        return False

    req = GetParameters.Request()
    req.names = ['robot_description', 'robot_description_semantic']
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        node.get_logger().warn('呼叫 /move_group/get_parameters 失敗')
        return False

    vals = {}
    for name, val in zip(req.names, future.result().values):
        if val.type == ParameterType.PARAMETER_STRING and val.string_value:
            vals[name] = val.string_value

    if 'robot_description' in vals and 'robot_description_semantic' in vals:
        qos = _latched_qos()
        node._desc_pub = node.create_publisher(String, 'robot_description', qos)
        node._srdf_pub = node.create_publisher(String, 'robot_description_semantic', qos)
        node._desc_pub.publish(String(data=vals['robot_description']))
        node._srdf_pub.publish(String(data=vals['robot_description_semantic']))
        node.get_logger().info('已轉發 URDF/SRDF（latched topic）。')
        return True

    node.get_logger().warn('從 /move_group 取得 URDF/SRDF 內容失敗。')
    return False


class MoveItPoseFollower(Node):
    def __init__(self):
        super().__init__('moveit_pose_follower')

        # 讀參數
        self.declare_parameters('', [
            ('target_pose_topic', '/target_pose'),
            ('group_name', 'arm'),
            ('base_frame', 'base_link'),
            ('end_effector_link', ''),  # 空字串代表用 group 預設 EE link
            ('plan_execute_rate_hz', 5.0),
            ('planning_time', 1.5),
            ('position_tolerance', 0.01),
            ('orientation_tolerance', 0.05),
            ('goal_joint_tolerance', 0.005),
            ('max_velocity_scaling', 0.2),
            ('max_acceleration_scaling', 0.2),
            ('min_goal_translation_delta', 0.01),
            ('min_goal_rotation_delta_deg', 3.0),
        ])

        self.target_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        self.group_name   = self.get_parameter('group_name').get_parameter_value().string_value
        self.base_frame   = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_link      = self.get_parameter('end_effector_link').get_parameter_value().string_value
        self.rate_hz      = self.get_parameter('plan_execute_rate_hz').get_parameter_value().double_value
        self.min_dtrans   = self.get_parameter('min_goal_translation_delta').get_parameter_value().double_value
        self.min_drot_deg = self.get_parameter('min_goal_rotation_delta_deg').get_parameter_value().double_value

        # 初始化 MoveItPy（會自帶一個 MoveIt 內部用的 rclcpp/rclpy node）
        bridge_move_group_descriptions(self)
        # moveit_follower_node.py
        # 構造 MoveItPy 參數檔路徑（供內部 C++ 節點載入）
        pkg_share = get_package_share_directory('mediapipe_pose_pub')
        moveitpy_yaml = f"{pkg_share}/config/moveit_py_params.yaml"
        moveitpy_yaml_node = f"{pkg_share}/config/moveit_py_params_node.yaml"

        self.moveit = MoveItPy(
            node_name='moveit_py',
            launch_params_filepaths=[moveitpy_yaml_node],
            config_dict=None  # 以檔案提供參數，避開 node_name 不匹配問題
        )


        # 先注入 pipeline 參數（自動尋找內部節點）
        #if not self._set_moveitpy_pipeline_params():
         #   raise RuntimeError('無法設定 MoveItPy 規劃管線參數')

        self.arm = self.moveit.get_planning_component(self.group_name)

        # 訂閱 pose 目標
        self.sub = self.create_subscription(PoseStamped, self.target_topic, self.on_pose, 10)

        # 以 timer 週期性觸發規劃執行（避免每次目標更新都 replan）
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_timer)

        self._latest_goal: Optional[PoseStamped] = None
        self._last_planned_goal: Optional[PoseStamped] = None

        self.get_logger().info(f'MoveIt follower ready. group={self.group_name}, topic={self.target_topic}')

    # ---------- 內部工具：找到 MoveItPy 的內部節點並注入參數 ----------


    # ---------- 一般邏輯 ----------
    def on_pose(self, msg: PoseStamped):
        # 僅接受 base_frame 的目標（若不是，這裡也可以自行 tf 轉換）
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self.get_logger().warn_once(
                f'/target_pose 的 frame 是 {msg.header.frame_id}，預期 {self.base_frame}；請確認上游輸出')
        self._latest_goal = msg

    def _need_replan(self, a: PoseStamped, b: PoseStamped) -> bool:
        dt = pose_distance(a.pose.position, b.pose.position)
        class Q: pass
        qa, qb = Q(), Q()
        qa.x, qa.y, qa.z, qa.w = a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w
        qb.x, qb.y, qb.z, qb.w = b.pose.orientation.x, b.pose.orientation.y, b.pose.orientation.z, b.pose.orientation.w
        dr = quat_angle_deg(qa, qb)
        return (dt >= self.min_dtrans) or (dr >= self.min_drot_deg)

    def on_timer(self):
        if self._latest_goal is None:
            return

        if (self._last_planned_goal is None) or self._need_replan(self._latest_goal, self._last_planned_goal):
            goal = self._latest_goal

            # 設定起點：目前實際狀態
            self.arm.set_start_state_to_current_state()

            # 設定目標：以 PoseStamped + 指定末端連桿
            pose_link = self.ee_link if self.ee_link else None
            try:
                self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=pose_link)
            except Exception as e:
                self.get_logger().error(f'set_goal_state 失敗：{e}')
                return

            # 規劃
            plan_result = self.arm.plan()
            if not plan_result or not plan_result.trajectory:
                self.get_logger().warn('規劃失敗（plan_result 空）')
                return

            # 執行（交由 MoveIt 控制器）
            try:
                self.moveit.execute(plan_result.trajectory, controllers=[])
                self._last_planned_goal = goal
                self.get_logger().info('已執行一段新軌跡')
            except Exception as e:
                self.get_logger().error(f'執行失敗：{e}')


def main():
    # 由 launch 傳入一般節點參數（params.yaml）。
    # MoveItPy 內部參數由程式內部以 launch_params_filepaths + config_dict 設定。
    rclpy.init()


    node = MoveItPoseFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
