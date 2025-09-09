#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# 新版 MoveIt 2 Python API
from moveit.planning import MoveItPy

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

def quat_angle_deg(q1, q2) -> float:
    # 以四元數內積估角度（度）
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
    """
    從 /move_group 讀 robot_description 與 robot_description_semantic，
    然後用 TRANSIENT_LOCAL（latched）topic 轉發出來，給 MoveItPy 使用。
    """
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

        # 初始化 MoveItPy（會自帶一個 MoveIt 內部用的 rclpy node）
        bridge_move_group_descriptions(self)
        self.moveit = MoveItPy(node_name='moveit_py')

        if not self._set_moveitpy_pipeline_params():
            raise RuntimeError('無法設定 /moveit_py 規劃管線參數')
        
        self.arm = self.moveit.get_planning_component(self.group_name)

        # 可選：設定規劃時間、速度比例等（由 move_group 讀參數也可以）
        # 這些可透過 ROS 參數或 Pipeline 設定調；這裡保持簡化，專注於 pose 目標追隨

        # 訂閱 pose 目標
        self.sub = self.create_subscription(PoseStamped, self.target_topic, self.on_pose, 10)

        # 以 timer 週期性觸發規劃執行（避免每次目標更新都 replan）
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_timer)

        self._latest_goal: Optional[PoseStamped] = None
        self._last_planned_goal: Optional[PoseStamped] = None

        self.get_logger().info(f'MoveIt follower ready. group={self.group_name}, topic={self.target_topic}')

    def on_pose(self, msg: PoseStamped):
        # 僅接受 base_frame 的目標（若不是，這裡也可以自行 tf 轉換）
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self.get_logger().warn_once(
                f'/target_pose 的 frame 是 {msg.header.frame_id}，預期 {self.base_frame}；請確認上游輸出')
        self._latest_goal = msg

    def _need_replan(self, a: PoseStamped, b: PoseStamped) -> bool:
        # 平移門檻
        dt = pose_distance(a.pose.position, b.pose.position)
        # 旋轉門檻
        class Q: pass
        qa, qb = Q(), Q()
        qa.x, qa.y, qa.z, qa.w = a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w
        qb.x, qb.y, qb.z, qb.w = b.pose.orientation.x, b.pose.orientation.y, b.pose.orientation.z, b.pose.orientation.w
        dr = quat_angle_deg(qa, qb)
        return (dt >= self.min_dtrans) or (dr >= self.min_drot_deg)

    def on_timer(self):
        if self._latest_goal is None:
            return

        # 沒有「上一個已規劃目標」，或差異超過門檻 → 觸發規劃
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
    
    def _set_moveitpy_pipeline_params(self) -> bool:
    # 等 /moveit_py 的 set_parameters 服務
    cli = self.create_client(SetParameters, '/moveit_py/set_parameters')
    if not cli.wait_for_service(timeout_sec=5.0):
        self.get_logger().error('等不到 /moveit_py/set_parameters 服務（5s）。')
        return False

    req = SetParameters.Request()
    req.parameters = [
        # 告訴 MoveItCpp 要啟用哪些 pipeline（至少要有一個）
        Parameter(
            name='moveit_cpp.planning_pipelines',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                string_array_value=['ompl']
            )
        ),
        # 預設用哪個 pipeline
        Parameter(
            name='moveit_cpp.plan_request_params.planning_pipeline',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='ompl'
            )
        ),
        # OMPL 插件類別
        Parameter(
            name='ompl.plugin',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='ompl_interface/OMPLPlanner'
            )
        ),
        # request_adapters（用空白分隔的字串即可）
        Parameter(
            name='ompl.request_adapters',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value=(
                    'default_planning_request_adapters/ResolveConstraintFrames '
                    'default_planning_request_adapters/ValidateWorkspaceBounds '
                    'default_planning_request_adapters/CheckStartStateBounds '
                    'default_planning_request_adapters/CheckStartStateCollision '
                    'default_planning_response_adapters/AddTimeOptimalParameterization'
                )
            )
        ),
        Parameter(
            name='ompl.start_state_max_bounds_error',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=0.1
            )
        ),
    ]

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
    if not fut.done() or fut.result() is None:
        self.get_logger().error('設定 /moveit_py 參數失敗（呼叫失敗）。')
        return False

    # 檢查每個參數是否設定成功
    results = fut.result().results
    ok = all(res.successful for res in results)
    if not ok:
        self.get_logger().error('設定 /moveit_py 參數失敗（有項目不成功）。')
    else:
        self.get_logger().info('已設定 /moveit_py 規劃管線參數。')
    return ok


def main():
    rclpy.init()
    node = MoveItPoseFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
