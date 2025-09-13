import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import cv2
import math

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# MediaPipe
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

class MediaPipePosePublisher(Node):
    def __init__(self):
        super().__init__('mediapipe_pose_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('color_topic', '/camera/color/image_raw'),
                ('depth_topic', '/camera/aligned_depth_to_color/image_raw'),
                ('camera_info_topic', '/camera/color/camera_info'),
                ('depth_scale', 0.001),
                ('camera_frame', 'camera_color_optical_frame'),
                ('base_frame', 'base_link'),
                ('target_pose_topic', '/target_pose'),
                ('use_solution', 'hands'),   # 'hands' or 'pose'
                ('landmark_index', 8),
                ('publish_rate_hz', 15.0),
                ('min_translation_delta', 0.01),
                ('min_rotation_delta_deg', 3.0),
                ('fixed_orientation_xyzw', [0.0, 0.0, 0.0, 1.0]),
                ('publish_debug_image', True),
                ('debug_image_topic', '/mediapipe/annotated'),
                ('draw_skeleton', True),
            ]
        )

        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.target_pose_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        self.use_solution = self.get_parameter('use_solution').get_parameter_value().string_value
        self.landmark_index = self.get_parameter('landmark_index').get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.min_translation_delta = self.get_parameter('min_translation_delta').get_parameter_value().double_value
        self.min_rotation_delta_deg = self.get_parameter('min_rotation_delta_deg').get_parameter_value().double_value
        self.fixed_orientation_xyzw = self.get_parameter('fixed_orientation_xyzw').get_parameter_value().double_array_value
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.draw_skeleton = self.get_parameter('draw_skeleton').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.cam_info = None

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        
        # Message filters 同步
        self.color_sub = Subscriber(self, Image, self.color_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.info_sub  = Subscriber(self, CameraInfo, self.camera_info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.sync_callback)

        # MediaPipe 初始化
        if self.use_solution == 'hands':
            self.mp_solution = mp.solutions.hands.Hands(
                model_complexity=1, max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5)
        elif self.use_solution == 'pose':
            self.mp_solution = mp.solutions.pose.Pose(model_complexity=1, enable_segmentation=False)
        else:
            raise ValueError("use_solution must be 'hands' or 'pose'.")

        self.last_pose_base = None

        self.get_logger().info("MediaPipePosePublisher ready.")

    def sync_callback(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # 1) 取影像
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')  # 16UC1 或 32FC1
        self.cam_info = info_msg

        h, w, _ = color.shape
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        annotated = color.copy() if self.publish_debug_image else None
        # 2) MediaPipe 偵測
        if self.use_solution == 'hands':
            res = self.mp_solution.process(rgb)
            if not res.multi_hand_landmarks:
                return
            lm = res.multi_hand_landmarks[0].landmark
            if self.landmark_index >= len(lm):
                return
            u = int(lm[self.landmark_index].x * w)
            v = int(lm[self.landmark_index].y * h)
            if self.publish_debug_image and self.draw_skeleton:
                mp_drawing.draw_landmarks(
                    annotated,
                    hand_landmarks,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    mp_styles.get_default_hand_landmarks_style(),
                    mp_styles.get_default_hand_connections_style(),
            )
        else:  # pose（全身）
            res = self.mp_solution.process(rgb)
            if not res.pose_landmarks:
                return
            lm = res.pose_landmarks.landmark
            if self.landmark_index >= len(lm):
                return
            u = int(lm[self.landmark_index].x * w)
            v = int(lm[self.landmark_index].y * h)
            if self.publish_debug_image and self.draw_skeleton:
                mp_drawing.draw_landmarks(
                    annotated,
                    pose_landmarks,
                    mp.solutions.pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_styles.get_default_pose_landmarks_style(),
                )

        # 邊界檢查
        if u < 0 or u >= w or v < 0 or v >= h:
            return

        # 3) 取深度，轉 3D（camera frame）
        z = depth[v, u]
        if z == 0 or np.isnan(z):
            return

        # depth 單位換算
        if depth.dtype == np.uint16:
            z = float(z) * self.depth_scale  # mm→m
        else:
            z = float(z)  # 已是 m（32FC1）

        # Pinhole 反投影
        fx = self.cam_info.k[0]; fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]; cy = self.cam_info.k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        pose_cam = PoseStamped()
        pose_cam.header = color_msg.header
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.pose.position.x = x
        pose_cam.pose.position.y = y
        pose_cam.pose.position.z = z

        # 這裡先給固定 orientation（之後你可依手指方向估姿態）
        qx, qy, qz, qw = self.fixed_orientation_xyzw
        pose_cam.pose.orientation.x = float(qx)
        pose_cam.pose.orientation.y = float(qy)
        pose_cam.pose.orientation.z = float(qz)
        pose_cam.pose.orientation.w = float(qw)

        if self.publish_debug_image:
            cv2.circle(annotated, (u, v), 6, (0, 255, 255), -1)

        # 4) 轉到 base_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time())
            pose_base = do_transform_pose(pose_cam, tf)
        except Exception as e:
            self.get_logger().throttle_log(2000, f"TF not ready from {self.camera_frame} to {self.base_frame}: {e}")
            return

        # 5) 去抖（避免太頻繁觸發）
        if self.last_pose_base is not None:
            dt = self._pose_delta(self.last_pose_base.pose, pose_base.pose)
            if dt['trans'] < self.min_translation_delta and dt['rot_deg'] < self.min_rotation_delta_deg:
                return

        self.last_pose_base = pose_base
        pose_base.header.stamp = self.get_clock().now().to_msg()
        pose_base.header.frame_id = self.base_frame
        self.pub.publish(pose_base)
        if self.publish_debug_image:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

    @staticmethod
    def _pose_delta(a, b):
        at = np.array([a.position.x, a.position.y, a.position.z])
        bt = np.array([b.position.x, b.position.y, b.position.z])
        trans = float(np.linalg.norm(bt - at))

        aq = np.array([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w])
        bq = np.array([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w])
        # 角度差（簡化）：使用內積求夾角
        dot = float(np.clip(np.dot(aq, bq), -1.0, 1.0))
        rot = 2.0 * math.degrees(math.acos(abs(dot)))
        return {'trans': trans, 'rot_deg': rot}

def main():
    rclpy.init()
    node = MediaPipePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
