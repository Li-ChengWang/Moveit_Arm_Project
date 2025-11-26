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

  bool control_active_ = false;       // 是否已經開始控制
  rclcpp::Time sync_start_time_;      //用來計算手停住的時間
  bool is_syncing_ = false;           // 是否正在倒數計時

 /* 
  // Teleop 相關
  bool calibrated_{false};
  geometry_msgs::msg::Point hand_ref_;          // 手的參考點（base_link）
  geometry_msgs::msg::Pose gripper_ref_pose_;   // 機械手臂 EEF 初始 Pose（base_link）


  // 縮放比例：Δhand → Δrobot
  double scale_x_, scale_y_, scale_z_;
*/

  // 限制機械手臂「目標點」的最大距離（安全球殼半徑）
  double max_robot_dist_;

  
  // joint_state 監控
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Time last_joint_state_stamp_;
  bool has_joint_state_{false};

  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool waitForRecentJointState(double timeout_sec);

};

/*
class MpToMoveItDriver : public rclcpp::Node
{
public:
  MpToMoveItDriver(const rclcpp::NodeOptions & options)
  : Node("mp_to_moveit_driver", options)
  , move_group_(shared_from_this(), "arm")  // 群組名稱自己改
  {
    js_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/left_follower/joint_states_control", 10);

    // MoveIt joint name -> follower joint name
    moveit_to_follower_ = {
      {"joint1",  "shoulder_pan"},
      {"joint2", "shoulder_lift"},
      {"joint3",    "elbow_flex"},
      {"joint4",    "wrist_flex"},
      {"joint5",    "wrist_roll"},
      // 如果還有 gripper 也可以加
    };
  }

private:

void playTrajectoryWithFollower(const trajectory_msgs::msg::JointTrajectory & traj)
{
  if (traj.points.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Empty trajectory, nothing to execute.");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Executing trajectory with %zu points via follower...",
              traj.points.size());

  // 這是第一個點的 time_from_start，通常是 0
  rclcpp::Duration prev_time = traj.points.front().time_from_start;

  for (size_t i = 0; i < traj.points.size(); ++i)
  {
    const auto & pt = traj.points[i];

    // 1) 先算這個點跟前一點的時間差
    rclcpp::Duration dt = pt.time_from_start - prev_time;
    if (dt.nanoseconds() < 0)
    {
      // 保護一下，理論上不會 < 0
      dt = rclcpp::Duration(0, 0);
    }

    // 2) 先 sleep 到該時間點再送
    if (i > 0)  // 第一點通常是 t=0，可以直接送
    {
      auto sleep_ns = std::chrono::nanoseconds(dt.nanoseconds());
      rclcpp::sleep_for(sleep_ns);
    }

    // 3) 把這個 JointTrajectoryPoint 轉成 JointState
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();

    if (pt.positions.size() != traj.joint_names.size())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Point[%zu] positions size (%zu) != joint_names size (%zu)",
                  i, pt.positions.size(), traj.joint_names.size());
      continue;
    }

    for (size_t j = 0; j < traj.joint_names.size(); ++j)
    {
      const std::string & moveit_name = traj.joint_names[j];

      auto it = moveit_to_follower_.find(moveit_name);
      if (it == moveit_to_follower_.end())
      {
        RCLCPP_WARN(this->get_logger(),
                    "No follower mapping for MoveIt joint '%s'",
                    moveit_name.c_str());
        continue;
      }

      const std::string & follower_name = it->second;

      js.name.push_back(follower_name);
      js.position.push_back(pt.positions[j]);  // rad → rad，follower 裡會轉成 deg
    }

    if (js.name.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Point[%zu] has no mapped joints, skip.", i);
      continue;
    }

    js_control_pub_->publish(js);
    RCLCPP_DEBUG(this->get_logger(), "Published JointState point[%zu] with %zu joints.",
                 i, js.name.size());

    // 4) 更新 prev_time，用來算下一個點的 dt
    prev_time = pt.time_from_start;
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory execution via follower finished.");
}

  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_control_pub_;

  std::map<std::string, std::string> moveit_to_follower_;
  
};
*/

// ===================== ctor =====================

MpToMoveIt::MpToMoveIt()
: Node("mp_to_moveit")
{
  // 參數
  group_name_    = this->declare_parameter<std::string>("group_name", "arm");
  eef_link_      = this->declare_parameter<std::string>("end_effector_link", "");
  plan_frame_    = this->declare_parameter<std::string>("planning_frame", "base_link");
  target_topic_  = this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
  min_trans_     = this->declare_parameter<double>("min_goal_translation_delta", 0.01);
  // 旋轉去抖參數保留，但不再使用（position-only 不看旋轉）
  min_rot_deg_   = this->declare_parameter<double>("min_goal_rotation_delta_deg", 3.0);
  planning_time_ = this->declare_parameter<double>("planning_time", 1.5);
  vel_scale_     = this->declare_parameter<double>("max_velocity_scaling", 0.2);
  acc_scale_     = this->declare_parameter<double>("max_acceleration_scaling", 0.2);
  allow_exec_    = this->declare_parameter<bool>("allow_execute", false);
  pos_tol_       = this->declare_parameter<double>("goal_position_tolerance", 0.05); // 位置容忍度

  // Teleop 相關參數
  /*
  scale_x_        = this->declare_parameter<double>("teleop_scale_x", 0.8);
  scale_y_        = this->declare_parameter<double>("teleop_scale_y", 0.8); // 先關掉 Y 軸控制
  scale_z_        = this->declare_parameter<double>("teleop_scale_z", 0.8);
  */
  max_robot_dist_ = this->declare_parameter<double>("max_robot_distance", 0.7); // 與你原本的 0.7m 一致

  // TF
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  /*
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  "joint_states", 10,
  std::bind(&MpToMoveIt::jointStateCb, this, std::placeholders::_1));
  */

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
      RCLCPP_INFO(this->get_logger(), "E_trans_EF from MoveGroup: %s",
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
  // ===================== 工具函式 =====================

  constexpr double X_MIN = -0.30;
  constexpr double X_MAX = 0.30;
  constexpr double Y_MIN = 0.10;
  constexpr double Y_MAX =  0.25;
  constexpr double Z_MIN = 0.05;
  constexpr double Z_MAX = 0.40;

  inline double clamp(
    double v,
    double v_min,
    double v_max,
    const rclcpp::Logger & logger)
  {
    if (v < v_min) {
      RCLCPP_WARN(
        logger,"[CLAMP] value %.3f below min %.3f, clamped.",v, v_min);
      return v_min;
    }
    if (v > v_max) {
      RCLCPP_WARN(
        logger,"[CLAMP] value %.3f above max %.3f, clamped.",v, v_max);
      return v_max;
    }
    return v;
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

  geometry_msgs::msg::Point target_pos = target.pose.position; // 假設你前面已經轉好 target 了

// ====== [NEW] 安全啟動邏輯 (Homing) - TF 版 ======
    if (!control_active_) {
        
        // [修改] 不用 move_group_->getCurrentPose()，改用 TF 查「最新」位置
        // 這樣可以避開 "Failed to fetch current robot state" 的時間同步問題
        
        // 1. 確認 End-Effector 名字 (防呆)
        std::string eef_name = eef_link_;
        if (eef_name.empty()) {
            eef_name = move_group_->getEndEffectorLink();
        }
        // 如果還是空的，就預設一個 (請改成你 URDF 裡真正的夾爪 link)
        if (eef_name.empty()) eef_name = "gripper_static_1"; 

        geometry_msgs::msg::Point current_robot_pos;
        bool tf_success = false;

        try {
            // ★ 關鍵：使用 tf2::TimePointZero 拿「最新」的一筆資料
            auto tf_robot = tf_buffer_->lookupTransform(
                plan_frame_,      // Target frame (base_link)
                eef_name,         // Source frame (gripper)
                tf2::TimePointZero 
            );
            
            current_robot_pos.x = tf_robot.transform.translation.x;
            current_robot_pos.y = tf_robot.transform.translation.y;
            current_robot_pos.z = tf_robot.transform.translation.z;
            tf_success = true;

        } catch (const tf2::TransformException & ex) {
            // 如果 TF 還沒準備好，就先跳過這一次
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Waiting for robot TF (%s -> %s): %s", 
                plan_frame_.c_str(), eef_name.c_str(), ex.what());
            return;
        }

        if (tf_success) {
            // B. 計算距離 (手 vs 機械手臂)
            double dist = std::sqrt(
              std::pow(target_pos.x - current_robot_pos.x, 2) +
              std::pow(target_pos.y - current_robot_pos.y, 2) +
              std::pow(target_pos.z - current_robot_pos.z, 2)
            );

            // C. 設定閾值 (例如 15 公分 - 放寬一點比較好對準)
            double sync_threshold = 0.05; 

            if (dist < sync_threshold) {
              if (!is_syncing_) {
                sync_start_time_ = this->now();
                is_syncing_ = true;
                RCLCPP_INFO(this->get_logger(), "🟡 Detected! Hold still to activate...");
              } else {
                auto duration = this->now() - sync_start_time_;
                if (duration.seconds() > 1.5) {
                  control_active_ = true;
                  RCLCPP_INFO(this->get_logger(), "🟢 LOCKED ON! Control Active!");
                }
              }
            } else {
              is_syncing_ = false;
              // 印出座標來除錯
              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🔴 Safety Lock: Dist: %.3fm\n"
                "   👉 Hand : (%.3f, %.3f, %.3f)\n"
                "   🤖 Robot: (%.3f, %.3f, %.3f) [%s]", 
                dist, 
                target_pos.x, target_pos.y, target_pos.z,
                current_robot_pos.x, current_robot_pos.y, current_robot_pos.z,
                eef_name.c_str());
            }
        }
        return;
    }
/*
  RCLCPP_WARN(this->get_logger(),
    "[OUT] frame=%s  p=(%.3f, %.3f, %.3f)",
    plan_frame_.c_str(),
    target.pose.position.x, target.pose.position.y, target.pose.position.z);
*/
  /*
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
  */
  /*
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

  //hand_ref_ = hand_cur; // 更新參考點（類似積分器）

  // 把 Δhand 映射到 Δrobot（可縮放）
  double dx_robot = scale_x_ * dx_hand;
  double dy_robot = scale_y_ * dy_hand;
  double dz_robot = scale_z_ * dz_hand;

  // 機械手臂的目標位置 = gripper_ref + Δrobot
  geometry_msgs::msg::Point P_target;
  P_target.x = gripper_ref_pose_.position.x - dy_robot;
  P_target.y = gripper_ref_pose_.position.y + dx_robot;
  P_target.z = gripper_ref_pose_.position.z + dz_robot;
*/
  // [PLUS] 加入這段新的邏輯

  // 1. 直接取得轉換後的絕對座標
  // target 是已經經過 TF 轉換，變成了 base_link 座標系下的手部位置
  // 所以 target.pose.position.x 就是機器人前方的 X 座標
  geometry_msgs::msg::Point P_target = target.pose.position;

  // 2. (可選) 座標修正
  // 有時候雖然 TF 轉過來了，但如果覺得手的位置直接對應 End-Effector 太危險，
  // 可以在這裡加一點 offset (偏移量)。
  // 例如：希望手在相機前，但機器人 End-Effector 保持在手前方 10cm 處，不要碰到手
  // P_target.x += 0.0; 
  // P_target.y += 0.0;
  // P_target.z += 0.0;

  // 3. [非常重要] 範圍限制 (Clamp) 維持開啟
  // 絕對模式下最怕 TF 算錯，手跑到 3 公尺外，手臂會被拉斷。
  // 必須強制把座標鎖在安全工作區內。
  P_target.x = clamp(P_target.x, X_MIN, X_MAX, this->get_logger());
  P_target.y = clamp(P_target.y, Y_MIN, Y_MAX, this->get_logger());
  P_target.z = clamp(P_target.z, Z_MIN, Z_MAX, this->get_logger());

  // Log 方便除錯
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[ABSOLUTE] Hand(base_frame): (%.3f, %.3f, %.3f) -> Clamped: (%.3f, %.3f, %.3f)",
    target.pose.position.x, target.pose.position.y, target.pose.position.z,
    P_target.x, P_target.y, P_target.z);



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
    "[TARGET] gripper_target=(%.3f, %.3f, %.3f),",
    P_target.x, P_target.y, P_target.z);

  // 2) 去抖（看「機械手目標點」的變化）
  if (has_prev_) {
    const auto& p  = P_target;
    const auto& pp = prev_.pose.position;
    if (p.x-pp.x < min_trans_ && 
        p.y-pp.y < min_trans_ &&
        p.z-pp.z < min_trans_) {
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

/*
  if (result == moveit::core::MoveItErrorCode::SUCCESS){
    const auto & traj = plan.trajectory_.joint_trajectory;
    playTrajectoryWithFollower(traj);
  }
  else{
    RCLCPP_WARN(this->get_logger(), "Planning failed, skip execution.");
  }
*/

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
