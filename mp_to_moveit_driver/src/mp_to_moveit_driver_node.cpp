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
  MpToMoveIt() : Node("mp_to_moveit") {
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
    pos_tol_       = this->declare_parameter<double>("goal_position_tolerance", 0.05); // 新增：位置容忍度（預設1cm）

    // TF
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // MoveIt 介面延後初始化（確保 shared_from_this 可用）
    init_timer_ = this->create_wall_timer(100ms, [this]() {
      init_timer_->cancel();

      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), group_name_);

      if (!eef_link_.empty())
        move_group_->setEndEffectorLink(eef_link_);

      move_group_->setPlanningTime(planning_time_);
      move_group_->setMaxVelocityScalingFactor(vel_scale_);
      move_group_->setMaxAccelerationScalingFactor(acc_scale_);

      // 關鍵：規劃參考座標系 + 位置容忍度
      move_group_->setPoseReferenceFrame(plan_frame_);
      move_group_->setGoalPositionTolerance(pos_tol_);

      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized. group=%s, eef=%s, frame=%s",
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

    RCLCPP_INFO(this->get_logger(), "Driver ready. group=%s, topic=%s, plan_frame=%s",
                group_name_.c_str(), target_topic_.c_str(), plan_frame_.c_str());
  }

private:
  // 保留函式，但不再使用（position-only）
  static double quatAngleDeg(const geometry_msgs::msg::Quaternion& a,
                             const geometry_msgs::msg::Quaternion& b) {
    const double dot = std::clamp(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w, -1.0, 1.0);
    return 2.0 * 180.0/M_PI * std::acos(std::abs(dot));
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!move_group_) return; // 還沒初始化好

    if (msg->header.frame_id != "camera_color_optical_frame" &&
        msg->header.frame_id != plan_frame_) { // plan_frame_ 通常是 "base_link"
      RCLCPP_ERROR(this->get_logger(),
        "Unexpected frame %s. Expected camera_color_optical_frame or %s.",
        msg->header.frame_id.c_str(), plan_frame_.c_str());
      return;
    } 


    RCLCPP_WARN(this->get_logger(), 
      "[IN]  frame=%s  p=(%.3f, %.3f, %.3f)",
      msg->header.frame_id.c_str(),
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

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

    RCLCPP_WARN(this->get_logger(),
      "[OUT] frame=%s  p=(%.3f, %.3f, %.3f)",
      plan_frame_.c_str(),
      target.pose.position.x, target.pose.position.y, target.pose.position.z);

    auto dist = std::sqrt(
      target.pose.position.x*target.pose.position.x +
      target.pose.position.y*target.pose.position.y +
      target.pose.position.z*target.pose.position.z);

    if (dist > 0.7) {
      RCLCPP_ERROR(this->get_logger(),
        "[REJECT] target %.2fm out of reach. Check units/TF/back-projection.",
        dist);
      return;
    }

    
    // 2) 去抖（只看位移，不看旋轉）
    if (has_prev_) {
      const auto& p  = target.pose.position;
      const auto& pp = prev_.pose.position;
      const double dtrans = std::hypot(std::hypot(p.x-pp.x, p.y-pp.y), p.z-pp.z);
      if (dtrans < min_trans_) return;
    }
    prev_ = target; has_prev_ = true;

    // 3) 僅設定「位置目標」——關鍵一行（完全不設定 orientation）
    move_group_->clearPoseTargets();                    // ← 清掉任何殘留目標（超關鍵）
    move_group_->setStartStateToCurrentState();

    move_group_->setGoalPositionTolerance(pos_tol_);        // ← 先放寬到 5 cm 試跑（確定流程 OK 再收緊）
    const auto& P = target.pose.position; 
    const std::string eef = eef_link_.empty()
      ? move_group_->getEndEffectorLink()
      : eef_link_;
    move_group_->setPositionTarget(P.x, P.y, P.z, eef); // ← 確保用 eef，不是 frame
    // （可選保險，避免任何意外 orientation 約束干擾）
    move_group_->setGoalOrientationTolerance(M_PI);

    // 4) 規劃 + （可選）執行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Plan failed to position-only target (%.3f, %.3f, %.3f).",
                  P.x, P.y, P.z);
      return;
    }

    if (allow_exec_) {
      auto ex = move_group_->execute(plan);
      if (ex != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Execute failed.");
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Executed position-only target (%.3f, %.3f, %.3f).", P.x, P.y, P.z);
      }
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Planned (dry-run) to position-only target (%.3f, %.3f, %.3f).", P.x, P.y, P.z);
    }
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
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpToMoveIt>());
  rclcpp::shutdown();
  return 0;
}
