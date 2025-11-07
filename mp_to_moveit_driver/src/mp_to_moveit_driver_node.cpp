#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>   // 用 .hpp 版本
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
    group_name_   = this->declare_parameter<std::string>("group_name", "arm");
    eef_link_     = this->declare_parameter<std::string>("end_effector_link", "");
    plan_frame_   = this->declare_parameter<std::string>("planning_frame", "base_link");
    target_topic_ = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    min_trans_    = this->declare_parameter<double>("min_goal_translation_delta", 0.005);
    min_rot_deg_  = this->declare_parameter<double>("min_goal_rotation_delta_deg", 3.0);
    planning_time_= this->declare_parameter<double>("planning_time", 1.5);
    vel_scale_    = this->declare_parameter<double>("max_velocity_scaling", 0.2);
    acc_scale_    = this->declare_parameter<double>("max_acceleration_scaling", 0.2);
    allow_exec_   = this->declare_parameter<bool>("allow_execute", false);

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // MoveIt 介面
    // 建構子內
    init_timer_ = this->create_wall_timer(100ms, [this]() {
    // 只跑一次
    init_timer_->cancel();
    // 這裡 shared_from_this() 已經安全
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        this->shared_from_this(), group_name_);

    if (!eef_link_.empty())
        move_group_->setEndEffectorLink(eef_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(acc_scale_);

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    });


    // 訂閱目標位姿（用 SensorData QoS）
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, qos,
      std::bind(&MpToMoveIt::poseCb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Driver ready. group=%s, topic=%s, plan_frame=%s",
                group_name_.c_str(), target_topic_.c_str(), plan_frame_.c_str());
  }

private:
  static double quatAngleDeg(const geometry_msgs::msg::Quaternion& a,
                             const geometry_msgs::msg::Quaternion& b) {
    const double dot = std::clamp(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w, -1.0, 1.0);
    return 2.0 * 180.0/M_PI * std::acos(std::abs(dot));
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!move_group_) {
        // 還沒初始化好，先不規劃
        return;
    }
  geometry_msgs::msg::PoseStamped target = *msg;

    // 對齊到規劃座標系
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

    // 去抖
    if (has_prev_) {
      const auto& p  = target.pose.position;
      const auto& q  = target.pose.orientation;
      const auto& pp = prev_.pose.position;
      const auto& pq = prev_.pose.orientation;
      const double dtrans = std::hypot(std::hypot(p.x-pp.x, p.y-pp.y), p.z-pp.z);
      const double drot   = quatAngleDeg(q, pq);
      if (dtrans < min_trans_ && drot < min_rot_deg_) return;
    }
    prev_ = target; has_prev_ = true;

    // 規劃與（可選）執行
    move_group_->setStartStateToCurrentState();
    if (!eef_link_.empty())
      move_group_->setPoseTarget(target.pose, eef_link_);
    else
      move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Plan failed.");
      return;
    }

    if (allow_exec_) {
      auto ex = move_group_->execute(plan);
      if (ex != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Execute failed.");
      }
    }
  }

  // 參數與成員
  std::string group_name_, eef_link_, plan_frame_, target_topic_;
  double min_trans_, min_rot_deg_, planning_time_, vel_scale_, acc_scale_;
  bool allow_exec_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;


  geometry_msgs::msg::PoseStamped prev_;
  bool has_prev_{false};
};  // ← 一定要有這個分號結束類別

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpToMoveIt>());
  rclcpp::shutdown();
  return 0;
}
