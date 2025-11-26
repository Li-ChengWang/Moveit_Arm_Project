#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <map>
#include <thread>

using namespace std::chrono_literals;

class MpToMoveIt : public rclcpp::Node {
public:
  MpToMoveIt();

private:
  // 回呼
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 參數與成員
  std::string group_name_, eef_link_, plan_frame_, target_topic_;
  double min_trans_, min_rot_deg_, planning_time_, vel_scale_, acc_scale_, pos_tol_;
  bool allow_exec_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // 夾爪控制相關
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  bool last_gripper_close_cmd_{false}; 
  geometry_msgs::msg::PoseStamped prev_;
  bool has_prev_{false};

  // 安全啟動與 Homing
  bool control_active_ = false;       
  rclcpp::Time sync_start_time_;      
  bool is_syncing_ = false;
  double max_robot_dist_;

  // =================================================================
  // [NEW] 實體手臂驅動相關
  // =================================================================
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_control_pub_;
  
  // 用來儲存所有關節的當前值，確保發送給 Driver 時永遠是完整的 6 軸數據
  std::map<std::string, double> current_joint_state_;

  // Python Driver 需要的嚴格順序 (對應 YAML)
  const std::vector<std::string> DRIVER_JOINT_ORDER = {
      "shoulder_pan", 
      "shoulder_lift", 
      "elbow_flex", 
      "wrist_flex", 
      "wrist_roll", 
      "gripper"
  };

  // MoveIt Joint Name 到 Driver Joint Name 的映射
  // 請確認左邊的名字與您的 URDF/SRDF 一致
  std::map<std::string, std::string> moveit_to_driver_map_ = {
      {"joint1", "shoulder_pan"},
      {"joint2", "shoulder_lift"},
      {"joint3", "elbow_flex"},
      {"joint4", "wrist_flex"},
      {"joint5", "wrist_roll"},
      {"gripper_joint", "gripper"} 
  };

  // 手動執行軌跡的函式
  void playTrajectoryWithFollower(const trajectory_msgs::msg::JointTrajectory & traj);

  // 工具函式：限制數值
  double clamp(double v, double v_min, double v_max);
};


// ===================== 實作 =====================

MpToMoveIt::MpToMoveIt()
: Node("mp_to_moveit")
{
  // 參數宣告
  group_name_    = this->declare_parameter<std::string>("group_name", "arm");
  eef_link_      = this->declare_parameter<std::string>("end_effector_link", "");
  plan_frame_    = this->declare_parameter<std::string>("planning_frame", "base_link");
  target_topic_  = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
  min_trans_     = this->declare_parameter<double>("min_goal_translation_delta", 0.01);
  planning_time_ = this->declare_parameter<double>("planning_time", 1.5);
  vel_scale_     = this->declare_parameter<double>("max_velocity_scaling", 0.2);
  acc_scale_     = this->declare_parameter<double>("max_acceleration_scaling", 0.2);
  allow_exec_    = this->declare_parameter<bool>("allow_execute", false);
  pos_tol_       = this->declare_parameter<double>("goal_position_tolerance", 0.05); 
  max_robot_dist_ = this->declare_parameter<double>("max_robot_distance", 0.7);

  // 初始化發布者
  js_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/left_follower/joint_states_control", 10);

  // 初始化 current_joint_state_
  for(const auto& name : DRIVER_JOINT_ORDER) {
      current_joint_state_[name] = 0.0;
  }

  // TF Buffer
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // MoveIt 初始化 Timer
  init_timer_ = this->create_wall_timer(
    100ms,
    [this]()
    {
      init_timer_->cancel();

      // 1. 初始化 Arm Group
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), group_name_);
      
      if (!eef_link_.empty()) {
        move_group_->setEndEffectorLink(eef_link_);
      }
      move_group_->setPlanningTime(planning_time_);
      move_group_->setMaxVelocityScalingFactor(vel_scale_);
      move_group_->setMaxAccelerationScalingFactor(acc_scale_);
      move_group_->setPoseReferenceFrame(plan_frame_);
      move_group_->setGoalPositionTolerance(pos_tol_);
      move_group_->setGoalOrientationTolerance(3.14); 

      // 2. 初始化 Gripper Group
      try {
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "gripper");
        gripper_group_->setMaxVelocityScalingFactor(1.0);
        RCLCPP_INFO(this->get_logger(), "Gripper MoveGroup initialized.");
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to init gripper group: %s", e.what());
      }

      RCLCPP_INFO(this->get_logger(), "MoveIt Groups Ready!");
    });

  // 訂閱目標
  auto qos = rclcpp::SensorDataQoS();
  sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic_, qos,
    std::bind(&MpToMoveIt::poseCb, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MpToMoveIt Node Started.");
}

// ===================== [CORE] 手動執行軌跡 =====================
void MpToMoveIt::playTrajectoryWithFollower(const trajectory_msgs::msg::JointTrajectory & traj)
{
  if (traj.points.empty()) return;

  // 確保沒有同時有別的指令在跑 (簡單處理)
  // 實務上這裡可能需要 mutex，但若是單線程 callback 則沒差

  rclcpp::Duration prev_time(0, 0);

  // 遍歷每一個軌跡點
  for (size_t i = 0; i < traj.points.size(); ++i)
  {
    const auto & pt = traj.points[i];

    // [FIX] 修正時間計算錯誤
    // pt.time_from_start 是 builtin_interfaces::msg::Duration
    // 我們將它轉成 rclcpp::Duration 後再做減法
    rclcpp::Duration current_time(pt.time_from_start);
    rclcpp::Duration dt = current_time - prev_time;
    
    if (dt.seconds() > 0.0) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(dt.nanoseconds()));
    }
    prev_time = current_time;

    // 更新本地記憶的關節狀態
    for (size_t j = 0; j < traj.joint_names.size(); ++j)
    {
        std::string moveit_name = traj.joint_names[j];
        if (moveit_to_driver_map_.count(moveit_name)) {
            std::string driver_name = moveit_to_driver_map_[moveit_name];
            current_joint_state_[driver_name] = pt.positions[j];
        }
    }

    // 打包成 Driver 需要的格式
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    
    for (const auto& name : DRIVER_JOINT_ORDER) {
        double val = 0.0;
        if (current_joint_state_.count(name)) {
            val = current_joint_state_[name];
        }
        js.position.push_back(val);
        js.name.push_back(name);
    }

    js_control_pub_->publish(js);
  }
}

double MpToMoveIt::clamp(double v, double v_min, double v_max) {
    return std::max(v_min, std::min(v, v_max));
}

// ===================== Pose Callback =====================
void MpToMoveIt::poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!move_group_) return;

  // 1. 座標轉換
  geometry_msgs::msg::PoseStamped target = *msg;
  if (msg->header.frame_id != plan_frame_) {
    try {
      auto tf = tf_buffer_->lookupTransform(
        plan_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, target, tf);
    } catch (const std::exception& e) {
      return;
    }
  }

  geometry_msgs::msg::Point target_pos = target.pose.position;

  // 2. 安全啟動邏輯 (Homing)
  if (!control_active_) {
      std::string eef_name = eef_link_.empty() ? move_group_->getEndEffectorLink() : eef_link_;
      if (eef_name.empty()) eef_name = "gripper"; 

      try {
          auto tf_robot = tf_buffer_->lookupTransform(
              plan_frame_, eef_name, tf2::TimePointZero);
          
          double dx = target_pos.x - tf_robot.transform.translation.x;
          double dy = target_pos.y - tf_robot.transform.translation.y;
          double dz = target_pos.z - tf_robot.transform.translation.z;
          double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

          if (dist < 0.15) { 
              if (!is_syncing_) {
                  sync_start_time_ = this->now();
                  is_syncing_ = true;
                  RCLCPP_INFO(this->get_logger(), "🟡 Detected! Hold still...");
              } else if ((this->now() - sync_start_time_).seconds() > 1.0) {
                  control_active_ = true;
                  RCLCPP_INFO(this->get_logger(), "🟢 LOCKED ON! Control Active!");
              }
          } else {
              is_syncing_ = false;
          }
      } catch (...) {}
      return; 
  }

  // 3. 安全範圍限制
  target_pos.x = clamp(target_pos.x, -0.30, 0.30);
  target_pos.y = clamp(target_pos.y,  0.10, 0.25);
  target_pos.z = clamp(target_pos.z,  0.05, 0.40);

  // 4. [Gripper]
  if (gripper_group_) { 
      bool want_close = (target.pose.orientation.w > 0.5);
      if (want_close != last_gripper_close_cmd_) {
          RCLCPP_INFO(this->get_logger(), want_close ? "✊ Gripper CLOSE" : "🖐 Gripper OPEN");
          
          gripper_group_->setNamedTarget(want_close ? "close" : "open");
          
          moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
          if (gripper_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              if (allow_exec_) {
                  // [FIX] 修正 struct 成員名稱
                  playTrajectoryWithFollower(gripper_plan.trajectory.joint_trajectory);
              }
          }
          last_gripper_close_cmd_ = want_close;
      }
  }

  // 5. [Arm]
  if (has_prev_) {
      double d = std::abs(target_pos.x - prev_.pose.position.x) + 
                 std::abs(target_pos.y - prev_.pose.position.y) + 
                 std::abs(target_pos.z - prev_.pose.position.z);
      if (d < min_trans_) return;
  }
  prev_.pose.position = target_pos;
  has_prev_ = true;

  move_group_->setStartStateToCurrentState();
  const std::string eef = eef_link_.empty() ? move_group_->getEndEffectorLink() : eef_link_;
  move_group_->setPositionTarget(target_pos.x, target_pos.y, target_pos.z, eef);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      if (allow_exec_) {
          // [FIX] 修正 struct 成員名稱
          playTrajectoryWithFollower(plan.trajectory.joint_trajectory);
      } else {
          RCLCPP_INFO(this->get_logger(), "Planning SUCCESS (Dry Run)");
      }
  } else {
      RCLCPP_WARN(this->get_logger(), "Planning FAILED");
  }
}

// ===================== main =====================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MpToMoveIt>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}



/*
  // ====== [NEW Part 1] 夾爪控制邏輯 ======
  // 檢查 w 是否 > 0.5 (代表捏合/Close)
  if (gripper_group_) { 
      bool want_close = (target.pose.orientation.w > 0.5);

      if (want_close != last_gripper_close_cmd_) {
          if (want_close) {
              RCLCPP_INFO(this->get_logger(), "✊ Gripper CLOSE");
              gripper_group_->setNamedTarget("close"); // SRDF 裡的 target name
              gripper_group_->asyncMove(); 
          } else {
              RCLCPP_INFO(this->get_logger(), "🖐 Gripper OPEN");
              gripper_group_->setNamedTarget("open");  // SRDF 裡的 target name
              gripper_group_->asyncMove();
          }
          last_gripper_close_cmd_ = want_close;
      }
  }

  // ====== [NEW Part 2] 手臂控制邏輯 (含面向限制) ======
  move_group_->clearPoseTargets();
  move_group_->setStartStateToCurrentState();
  move_group_->setGoalPositionTolerance(pos_tol_);

  const std::string eef = eef_link_.empty()
    ? move_group_->getEndEffectorLink()
    : eef_link_;

  // A. 設定位置目標 (原本的邏輯)
  move_group_->setPositionTarget(P_target.x, P_target.y, P_target.z, eef);

  // B. [修改] 設定面向目標 (Orientation Constraint) 解決 5-DOF 亂飄
  // 設定目標為：夾爪朝前 (Quaternion: 0, 0, 0, 1)
  // 如果你的手臂初始狀態不是朝前，請根據實際情況修改這裡的 quaternion
  //move_group_->setOrientationTarget(0.0, 0.0, 0.7071, 0.7071, eef);

  // C. [修改] 設定面向容忍度
  // 5-DOF 手臂無法完美對齊，必須給予較大寬容度 (例如 1.0 rad ≈ 57度)
  // 讓它盡量朝前，但允許因為機構限制而傾斜
  move_group_->setGoalOrientationTolerance(M_PI); 

  // 4) 規劃與執行
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      if (allow_exec_) {
          // 使用 asyncExecute 讓動作更平滑連續
          move_group_->execute(plan); 
      } else {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Plan SUCCESS (Dry Run) -> (%.3f, %.3f, %.3f)", P_target.x, P_target.y, P_target.z);
      }
  } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Plan FAILED (Constraint too strict?) -> (%.3f, %.3f, %.3f)", P_target.x, P_target.y, P_target.z);
  }
*/