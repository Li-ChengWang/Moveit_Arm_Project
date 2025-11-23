#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class MpToMoveIt : public rclcpp::Node {
public:
  MpToMoveIt();

private:
  // 回呼
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 保留函式，但不再使用（position-only）
  static double quatAngleDeg(const geometry_msgs::msg::Quaternion& a,
                             const geometry_msgs::msg::Quaternion& b) {
    const double dot = std::clamp(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w, -1.0, 1.0);
    return 2.0 * 180.0/M_PI * std::acos(std::abs(dot));
  }

  // 參數與成員
  std::string group_name_, eef_link_, plan_frame_, target_topic_;
  double min_trans_, min_rot_deg_, planning_time_, vel_scale_, acc_scale_, pos_tol_;
  bool allow_exec_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  geometry_msgs::msg::PoseStamped prev_;
  bool has_prev_{false};

  // Teleop 相關
  bool calibrated_{false};
  geometry_msgs::msg::Point hand_ref_;          // 手的參考點（base_link）
  geometry_msgs::msg::Pose gripper_ref_pose_;   // 機械手臂 EEF 初始 Pose（base_link）

  // 縮放比例：Δhand → Δrobot
  double scale_x_, scale_y_, scale_z_;
  // 限制機械手臂「目標點」的最大距離（安全球殼半徑）
  double max_robot_dist_;
};

// ===================== ctor =====================

MpToMoveIt::MpToMoveIt()
: Node("mp_to_moveit")
{
  // 參數
  group_name_    = this->declare_parameter<std::string>("group_name", "arm");
  eef_link_      = this->declare_parameter<std::string>("end_effector_link", "");
  plan_frame_    = this->declare_parameter<std::string>("planning_frame", "base_link");
  target_topic_  = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
  min_trans_     = this->declare_parameter<double>("min_goal_translation_delta", 0.005);
  // 旋轉去抖參數保留，但不再使用（position-only 不看旋轉）
  min_rot_deg_   = this->declare_parameter<double>("min_goal_rotation_delta_deg", 3.0);
  planning_time_ = this->declare_parameter<double>("planning_time", 1.5);
  vel_scale_     = this->declare_parameter<double>("max_velocity_scaling", 0.2);
  acc_scale_     = this->declare_parameter<double>("max_acceleration_scaling", 0.2);
  allow_exec_    = this->declare_parameter<bool>("allow_execute", false);
  pos_tol_       = this->declare_parameter<double>("goal_position_tolerance", 0.05); // 位置容忍度

  // Teleop 相關參數
  scale_x_        = this->declare_parameter<double>("teleop_scale_x", 0.5);
  scale_y_        = this->declare_parameter<double>("teleop_scale_y", 0.5); // 先關掉 Y 軸控制
  scale_z_        = this->declare_parameter<double>("teleop_scale_z", 0.5);
  max_robot_dist_ = this->declare_parameter<double>("max_robot_distance", 0.7); // 與你原本的 0.7m 一致

  // TF
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // MoveIt 介面延後初始化（確保 shared_from_this 可用）
  init_timer_ = this->create_wall_timer(
    100ms,
    [this]()
    {
      init_timer_->cancel();

      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), group_name_);

      if (!eef_link_.empty()) {
        move_group_->setEndEffectorLink(eef_link_);
      }

      move_group_->setPlanningTime(planning_time_);
      move_group_->setMaxVelocityScalingFactor(vel_scale_);
      move_group_->setMaxAccelerationScalingFactor(acc_scale_);

      // 關鍵：規劃參考座標系 + 位置容忍度
      move_group_->setPoseReferenceFrame(plan_frame_);
      move_group_->setGoalPositionTolerance(pos_tol_);

      RCLCPP_INFO(this->get_logger(),
                  "MoveGroupInterface initialized. group=%s, eef=%s, frame=%s",
                  group_name_.c_str(),
                  eef_link_.empty() ? "<group tip>" : eef_link_.c_str(),
                  plan_frame_.c_str());
      RCLCPP_INFO(this->get_logger(), "EEF from MoveGroup: %s",
                  move_group_->getEndEffectorLink().c_str());
      RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s",
                  move_group_->getPoseReferenceFrame().c_str());
    });

  // 訂閱目標位姿（Mediapipe 可發 PoseStamped；我們只用 position）
  auto qos = rclcpp::SensorDataQoS();
  sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic_, qos,
    std::bind(&MpToMoveIt::poseCb, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "Driver ready. group=%s, topic=%s, plan_frame=%s",
              group_name_.c_str(), target_topic_.c_str(), plan_frame_.c_str());
}

// ===================== callback =====================

void MpToMoveIt::poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!move_group_) return; // 還沒初始化好

  if (msg->header.frame_id != "camera_color_optical_frame" &&
      msg->header.frame_id != plan_frame_) { // plan_frame_ 通常是 "base_link"
    RCLCPP_ERROR(this->get_logger(),
      "Unexpected frame %s. Expected camera_color_optical_frame or %s.",
      msg->header.frame_id.c_str(), plan_frame_.c_str());
    return;
  }
/*
  RCLCPP_WARN(this->get_logger(),
    "[IN]  frame=%s  p=(%.3f, %.3f, %.3f)",
    msg->header.frame_id.c_str(),
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
*/
  // 1) 對齊到規劃座標系（通常是 base_link）
  geometry_msgs::msg::PoseStamped target = *msg;
  if (msg->header.frame_id != plan_frame_) {
    try {
      auto tf = tf_buffer_->lookupTransform(
        plan_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, target, tf);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "TF %s->%s not ready: %s",
                  msg->header.frame_id.c_str(), plan_frame_.c_str(), e.what());
      return;
    }
  }
/*
  RCLCPP_WARN(this->get_logger(),
    "[OUT] frame=%s  p=(%.3f, %.3f, %.3f)",
    plan_frame_.c_str(),
    target.pose.position.x, target.pose.position.y, target.pose.position.z);
*/
  // 手的位置（在 base_link / plan_frame_）
  const auto& hand_cur = target.pose.position;

  // 先計算手相對 base_link 原點的距離（防呆：手整個在超遠的地方就不用管）
  auto hand_dist = std::sqrt(
    hand_cur.x*hand_cur.x + hand_cur.y*hand_cur.y + hand_cur.z*hand_cur.z);

  if (hand_dist > 2.0) {  // 這裡是「感覺很怪就直接丟掉」，用比較寬的範圍
    RCLCPP_WARN(this->get_logger(),
      "[DROP][HAND_DIST] hand_dist=%.2f m too far, skip.", hand_dist);
    return;
  }

  // ====== 第一次有效輸入：建立參考點（calibration）======
  if (!calibrated_) {
    hand_ref_ = hand_cur;   // 記住這一刻「手」的位置

    const std::string eef = eef_link_.empty()
      ? move_group_->getEndEffectorLink()
      : eef_link_;

    auto cur_pose = move_group_->getCurrentPose(eef);
    gripper_ref_pose_ = cur_pose.pose;  // 記住這一刻「機械手臂 EEF」的位置與姿態

    calibrated_ = true;
    RCLCPP_INFO(this->get_logger(),
      "[CALIB] Teleop reference set. hand_ref=(%.3f, %.3f, %.3f), "
      "gripper_ref=(%.3f, %.3f, %.3f)",
      hand_ref_.x, hand_ref_.y, hand_ref_.z,
      gripper_ref_pose_.position.x,
      gripper_ref_pose_.position.y,
      gripper_ref_pose_.position.z);
    // 第一次只做標定，不馬上叫手臂動
    return;
  }
  // =====================================================

  // 已經 calibrate 過：算 Δhand（手相對起始位置的位移）
  double dx_hand = hand_cur.x - hand_ref_.x;
  double dy_hand = hand_cur.y - hand_ref_.y;
  double dz_hand = hand_cur.z - hand_ref_.z;

  // 把 Δhand 映射到 Δrobot（可縮放）
  double dx_robot = scale_x_ * dx_hand;
  double dy_robot = scale_y_ * dy_hand;
  double dz_robot = scale_z_ * dz_hand;

  // 機械手臂的目標位置 = gripper_ref + Δrobot
  geometry_msgs::msg::Point P_target;
  P_target.x = gripper_ref_pose_.position.x + dx_robot;
  P_target.y = gripper_ref_pose_.position.y + dy_robot;
  P_target.z = gripper_ref_pose_.position.z + dz_robot;

  // 檢查機械手臂目標點離 base_link 的距離
  auto robot_dist = std::sqrt(
    P_target.x*P_target.x + P_target.y*P_target.y + P_target.z*P_target.z);

  if (robot_dist > max_robot_dist_) {
    RCLCPP_WARN(this->get_logger(),
      "[REJECT][ROBOT_DIST] target %.2fm out of robot range (max=%.2f).",
      robot_dist, max_robot_dist_);
    return;
  }

  RCLCPP_WARN(this->get_logger(),
    "[TARGET] gripper_target=(%.3f, %.3f, %.3f), Δhand=(%.3f, %.3f, %.3f)",
    P_target.x, P_target.y, P_target.z,
    dx_hand, dy_hand, dz_hand);

  // 2) 去抖（看「機械手目標點」的變化）
  if (has_prev_) {
    const auto& p  = P_target;
    const auto& pp = prev_.pose.position;
    const double dtrans = std::hypot(std::hypot(p.x-pp.x, p.y-pp.y), p.z-pp.z);
    if (dtrans < min_trans_) {
      // 變化太小就不規劃
      return;
    }
    // 也可以加一個「大跳步拒絕」：例如 dtrans > 0.20 時視為鬼影
    // if (dtrans > 0.20) { ... return; }
  }
  prev_.pose.position = P_target;
  has_prev_ = true;

  // 3) 僅設定「位置目標」
  move_group_->clearPoseTargets();
  move_group_->setStartStateToCurrentState();
  move_group_->setGoalPositionTolerance(pos_tol_);

  const std::string eef = eef_link_.empty()
    ? move_group_->getEndEffectorLink()
    : eef_link_;
  move_group_->setPositionTarget(P_target.x, P_target.y, P_target.z, eef);
  move_group_->setGoalOrientationTolerance(M_PI);

  // 4) 規劃 + （可選）執行
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(this->get_logger(),
      "Plan failed to position-only target (%.3f, %.3f, %.3f).",
      P_target.x, P_target.y, P_target.z);
    return;
  }

  if (allow_exec_) {
    auto ex = move_group_->execute(plan);
    if (ex != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Execute failed.");
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Executed position-only target (%.3f, %.3f, %.3f).",
        P_target.x, P_target.y, P_target.z);
    }
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Planned (dry-run) to position-only target (%.3f, %.3f, %.3f).",
      P_target.x, P_target.y, P_target.z);
  }
}

// ===================== main =====================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpToMoveIt>());
  rclcpp::shutdown();
  return 0;
}
