// =============================================================================
//  precision_land_node —— AprilTag 視覺精準降落
//
//  對外只有一個 action：<ns>/precision_land
//
//  ⚠️ 交接協議：呼叫方在 goal 被接受後「必須停止」發布 trajectory_setpoint 與
//     offboard_control_mode。兩邊同時發 = PX4 收到交錯的矛盾指令 = 飛機抽搐，
//     而且從 log 完全看不出原因（看起來只像控制器沒調好）。
//
//  座標轉換鏈已由 test/t2_pose_check.py 在 Gazebo 裡實測驗證過（7 個已知位置
//  + 2 個航向，水平誤差 ≤ 0.4 cm）。這裡的公式和那支測試用的是同一套，
//  camera_offset / camera_rpy 兩個參數也是同一組值 —— 改了要兩邊一起改，
//  T2 會擋下不一致。
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "drone_apriltag_landing/action/precision_land.hpp"

using namespace std::chrono_literals;
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;

namespace
{

// 相機「光學座標系」→「link 座標系」的固定旋轉。
//   光學（ROS 慣例）  ：x 右、y 下、z 沿光軸向前
//   link（Gazebo 慣例）：x 沿光軸向前、y 左、z 上
// apriltag_ros 算出來的位姿是光學系，但 TF 上掛的 frame_id 是 link 的名字
// （camera_info 的 frame_id 來自 gz_frame_id）—— 這個落差是最常見的錯誤來源。
constexpr Mat3 kOpticalToLink = {{
  {{0.0, 0.0, 1.0}},
  {{-1.0, 0.0, 0.0}},
  {{0.0, -1.0, 0.0}},
}};

Mat3 rotRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  return {{
    {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}},
    {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}},
    {{-sp, cp * sr, cp * cr}},
  }};
}

Vec3 matVec(const Mat3 & m, const Vec3 & v)
{
  Vec3 o{};
  for (int i = 0; i < 3; ++i) {
    o[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
  }
  return o;
}

Mat3 matMul(const Mat3 & a, const Mat3 & b)
{
  Mat3 o{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
    }
  }
  return o;
}

// 從四元數取繞 z 軸的偏航角。
// 刻意自己算而不用 tf2::getYaw：這條公式要和 test/t2_pose_check.py 的
// quat_to_yaw 完全一致，T2 驗過的正負才算數。用不同的實作就等於沒驗過。
double quatToYaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

double clampAbs(double v, double lim)
{
  return std::max(-lim, std::min(lim, v));
}

}  // namespace


class PrecisionLandNode : public rclcpp::Node
{
public:
  using PrecisionLand = drone_apriltag_landing::action::PrecisionLand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PrecisionLand>;

  enum class State {IDLE, SEARCHING, ALIGNING, DESCENDING, HANDOFF};

  PrecisionLandNode()
  : rclcpp::Node("precision_land_node")
  {
    declareParameters();
    loadParameters();

    // 光學系 → 機體系（SDF 慣例：x 前、y 左、z 上）的合成旋轉。
    // 相機的安裝角度來自參數，不是寫死 —— 換機體只改 yaml。
    r_optical_to_body_ = matMul(
      rotRpy(camera_rpy_[0], camera_rpy_[1], camera_rpy_[2]), kOpticalToLink);

    // PX4 -> ROS 的 topic 一律 best effort；用預設的 reliable 會完全收不到，
    // 而且不會有任何錯誤訊息，只是安靜地沒有資料。
    rclcpp::QoS sub_qos(rclcpp::KeepLast(5));
    sub_qos.best_effort().durability_volatile();
    // ROS -> PX4 的指令用預設 reliable，相容性最好（沿用 drone_control 的做法）
    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));

    const std::string ns = "/" + namespace_;

    // 訊息版本化：MESSAGE_VERSION = 1 的 topic 才帶 _v1 後綴。
    // VehicleLocalPosition 與 VehicleStatus 是 1，VehicleLandDetected 是 0。
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ns + "/fmu/out/vehicle_local_position_v1", sub_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr m) {
        local_position_ = *m;
        has_local_position_ = true;
      });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      ns + "/fmu/out/vehicle_status_v1", sub_qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr m) {vehicle_status_ = *m;});
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      ns + "/fmu/out/vehicle_land_detected", sub_qos,
      [this](px4_msgs::msg::VehicleLandDetected::UniquePtr m) {land_detected_ = *m;});

    // detections 和 /tf 兩邊都要訂，缺一不可：
    //   detections 有 hamming / decision_margin，但沒有位姿
    //   /tf        有位姿，但沒有品質欄位
    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "detections", rclcpp::SensorDataQoS(),
      std::bind(&PrecisionLandNode::onDetections, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      ns + "/fmu/in/offboard_control_mode", pub_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      ns + "/fmu/in/trajectory_setpoint", pub_qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      ns + "/fmu/in/vehicle_command", pub_qos);
    // 即使跑 controller 模式也照發，可以直接 ros2 topic echo 看誤差怎麼收斂
    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "landing_error", 10);

    action_server_ = rclcpp_action::create_server<PrecisionLand>(
      this, "precision_land",
      std::bind(&PrecisionLandNode::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&PrecisionLandNode::handleCancel, this, std::placeholders::_1),
      std::bind(&PrecisionLandNode::handleAccepted, this, std::placeholders::_1));

    // 20 Hz。PX4 要求 offboard setpoint 串流 > 2 Hz，20 Hz 留了十倍餘裕，
    // 又不會像 100 Hz 那樣把 DDS 塞滿（多機時三台共用同一條網路）。
    timer_ = create_wall_timer(50ms, std::bind(&PrecisionLandNode::loop, this));

    logStartup();
  }

private:
  // ===========================================================================
  //  參數
  // ===========================================================================
  void declareParameters()
  {
    declare_parameter<std::string>("namespace", "MAV1");
    declare_parameter<int>("target_system", 1);
    declare_parameter<std::string>("camera_frame", "camera_down_link");
    declare_parameter<int>("tag_id", 0);
    declare_parameter<double>("tag_size", 0.8);

    declare_parameter<std::vector<double>>("camera_offset", {0.0, 0.0, 0.10});
    declare_parameter<std::vector<double>>("camera_rpy", {0.0, 1.5707, 0.0});

    declare_parameter<double>("min_decision_margin", 30.0);
    declare_parameter<int>("lock_frames", 5);
    declare_parameter<int>("lost_frames", 15);

    declare_parameter<double>("xy_tolerance", 0.15);
    declare_parameter<double>("yaw_tolerance_deg", 5.0);

    declare_parameter<double>("kp_xy", 0.5);
    declare_parameter<double>("kp_z", 0.6);
    declare_parameter<double>("kp_yaw", 0.8);
    declare_parameter<double>("max_speed_xy", 0.8);
    declare_parameter<double>("max_yawspeed", 0.5);
    declare_parameter<double>("descend_speed", 0.3);

    declare_parameter<double>("handoff_altitude", 1.0);
    declare_parameter<double>("search_timeout_s", 30.0);
    declare_parameter<double>("total_timeout_s", 120.0);

    declare_parameter<std::string>("control_mode", "controller");
  }

  void loadParameters()
  {
    namespace_ = get_parameter("namespace").as_string();
    target_system_ = get_parameter("target_system").as_int();
    camera_frame_ = get_parameter("camera_frame").as_string();
    tag_id_ = get_parameter("tag_id").as_int();
    tag_size_ = get_parameter("tag_size").as_double();

    auto off = get_parameter("camera_offset").as_double_array();
    auto rpy = get_parameter("camera_rpy").as_double_array();
    for (size_t i = 0; i < 3; ++i) {
      camera_offset_[i] = (off.size() > i) ? off[i] : 0.0;
      camera_rpy_[i] = (rpy.size() > i) ? rpy[i] : 0.0;
    }

    min_decision_margin_ = get_parameter("min_decision_margin").as_double();
    lock_frames_ = get_parameter("lock_frames").as_int();
    lost_frames_ = get_parameter("lost_frames").as_int();
    xy_tolerance_ = get_parameter("xy_tolerance").as_double();
    yaw_tolerance_ = get_parameter("yaw_tolerance_deg").as_double() * M_PI / 180.0;
    kp_xy_ = get_parameter("kp_xy").as_double();
    kp_z_ = get_parameter("kp_z").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    max_speed_xy_ = get_parameter("max_speed_xy").as_double();
    max_yawspeed_ = get_parameter("max_yawspeed").as_double();
    descend_speed_ = get_parameter("descend_speed").as_double();
    handoff_altitude_ = get_parameter("handoff_altitude").as_double();
    search_timeout_s_ = get_parameter("search_timeout_s").as_double();
    total_timeout_s_ = get_parameter("total_timeout_s").as_double();
    control_mode_ = get_parameter("control_mode").as_string();
    advisor_ = (control_mode_ == "advisor");
  }

  void logStartup()
  {
    const std::string eff = get_effective_namespace();
    RCLCPP_INFO(get_logger(), "===== precision_land_node =====");
    RCLCPP_INFO(get_logger(), "  namespace        : %s", namespace_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_frame     : %s", camera_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  tag_id / size    : %d / %.3f m", tag_id_, tag_size_);
    RCLCPP_INFO(get_logger(), "  camera_offset    : [%.3f %.3f %.3f]",
      camera_offset_[0], camera_offset_[1], camera_offset_[2]);
    RCLCPP_INFO(get_logger(), "  camera_rpy       : [%.4f %.4f %.4f]",
      camera_rpy_[0], camera_rpy_[1], camera_rpy_[2]);
    RCLCPP_INFO(get_logger(), "  handoff_altitude : %.2f m", handoff_altitude_);
    RCLCPP_INFO(get_logger(), "  control_mode     : %s%s", control_mode_.c_str(),
      advisor_ ? "（只回報誤差，不碰飛機）" : "（會接管控制權）");
    RCLCPP_INFO(get_logger(), "  action           : %s/precision_land",
      (eff == "/") ? "" : eff.c_str());
  }

  // ===========================================================================
  //  觀測：把 tag 的位姿換算成「相對本機的 NED 位移」
  // ===========================================================================
  void onDetections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    bool good = false;
    for (const auto & d : msg->detections) {
      // 三道品質關卡，缺一不可：
      //   id      —— 只認目標，場地裡有別的 tag 也不會誤鎖
      //   hamming —— 必須 0，不接受任何位元修正（36h11 保證任兩碼差 11 bit，
      //              設 0 就不可能誤判成別的 ID）
      //   margin  —— 黑白對比夠明顯，濾掉逆光和過曝的那幾幀
      if (d.id != tag_id_) {continue;}
      if (d.hamming != 0) {continue;}
      if (d.decision_margin < min_decision_margin_) {continue;}
      good = true;
      break;
    }
    if (!good) {return;}

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        camera_frame_, tag_frame_hint_.empty() ? guessTagFrame() : tag_frame_hint_,
        tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "偵測到 tag 但查不到 TF：%s", e.what());
      return;
    }

    if (!has_local_position_) {return;}

    // --- 轉換鏈（與 t2_pose_check.py 完全相同，那支已在 Gazebo 實測過）---
    const Vec3 v_opt{tf.transform.translation.x,
      tf.transform.translation.y, tf.transform.translation.z};
    // 1) 光學系 -> 機體系（SDF 慣例 x 前 / y 左 / z 上），再加上相機的安裝位移
    Vec3 v_flu = matVec(r_optical_to_body_, v_opt);
    for (int i = 0; i < 3; ++i) {v_flu[i] += camera_offset_[i];}
    // 2) SDF 機體系 -> PX4 的 FRD（x 前 / y 右 / z 下）：後兩軸反向
    const Vec3 v_frd{v_flu[0], -v_flu[1], -v_flu[2]};
    // 3) FRD -> NED：繞「下」軸轉機頭角
    const double h = local_position_.heading;
    tag_offset_ned_[0] = v_frd[0] * std::cos(h) - v_frd[1] * std::sin(h);
    tag_offset_ned_[1] = v_frd[0] * std::sin(h) + v_frd[1] * std::cos(h);
    tag_offset_ned_[2] = v_frd[2];

    // 4) 航向誤差 = 「機頭還要轉多少」。
    //
    //    ⚠️ 這個符號踩過一次坑，記錄清楚：
    //    T2 實測的是「探針 ENU yaw +40 度 -> 量到的 tag 角度 +40 度」，同向。
    //    但 ENU yaw（從東起算、逆時針為正）和 PX4 的 heading（從北起算、
    //    順時針為正）旋轉方向是「相反」的 —— 同一個實體轉動，兩者符號相反。
    //
    //    所以換成 heading 的語言：量到的角度 = -heading + 常數。
    //    要讓量到的角度歸零，heading 必須「增加」量到的那個值：
    //        目標 heading = heading + tag_yaw_in_cam
    //    寫成 -tag_yaw_in_cam 的話控制器會把機頭推離目標，
    //    最後穩定在差 180 度的地方 —— 水平位置照樣收斂得很漂亮，
    //    只有 yaw 永遠不收斂，看起來像「對不準」而不像「符號錯」。
    const auto & q = tf.transform.rotation;
    tag_yaw_in_cam_ = quatToYaw(q.x, q.y, q.z, q.w);
    yaw_error_ = wrapPi(tag_yaw_in_cam_);

    last_detect_time_ = now();
    ++consecutive_hits_;
    consecutive_miss_ = 0;
    if (consecutive_hits_ >= lock_frames_) {tag_locked_ = true;}
  }

  std::string guessTagFrame() const
  {
    // apriltag_ros 預設的 frame 名稱格式是 "<family>:<id>"（T2 實測 = tag36h11:0）。
    // 沒有寫成參數是因為它由 apriltag 的設定決定，這裡跟著推導比較不會不同步。
    return "tag36h11:" + std::to_string(tag_id_);
  }

  // ===========================================================================
  //  Action
  // ===========================================================================
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PrecisionLand::Goal> goal)
  {
    if (goal_handle_ && goal_handle_->is_active()) {
      RCLCPP_WARN(get_logger(), "已經有一個降落在進行中，拒絕新的 goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!has_local_position_) {
      RCLCPP_ERROR(get_logger(), "還沒收到 vehicle_local_position，"
        "確認 PX4 與 uXRCE agent 都在跑、namespace 是否為 %s", namespace_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!local_position_.xy_valid || !local_position_.z_valid) {
      RCLCPP_ERROR(get_logger(), "PX4 的位置估計無效（xy_valid=%d z_valid=%d），拒絕",
        local_position_.xy_valid, local_position_.z_valid);
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 自保：偵測到別人也在發 setpoint 就不接手。
    // 硬上的話兩組指令交錯，飛機會抽搐，而那個現象在 log 裡幾乎查不出原因，
    // 所以寧可在起點就擋下來並講清楚。
    if (!advisor_ && count_publishers(
        "/" + namespace_ + "/fmu/in/trajectory_setpoint") > 1)
    {
      RCLCPP_ERROR(get_logger(),
        "trajectory_setpoint 上還有其他發布者。呼叫方必須先停止發布再呼叫本 action");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "收到降落 goal：tag_id=%d approach_alt=%.2f align_yaw=%s",
      goal->tag_id, goal->approach_altitude, goal->align_yaw ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "收到取消請求，將停在原地並交還控制權");
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> gh)
  {
    goal_handle_ = gh;
    const auto goal = gh->get_goal();
    if (goal->tag_id >= 0) {tag_id_ = goal->tag_id;}

    // 目標高度：goal 沒指定就用當下高度。
    // NED 的 z 是「下為正」，所以離地高度 = -z。
    approach_alt_ = (goal->approach_altitude > 0.01)
      ? goal->approach_altitude : -local_position_.z;
    align_yaw_ = goal->align_yaw;

    tag_locked_ = false;
    consecutive_hits_ = 0;
    consecutive_miss_ = 0;
    cancel_requested_ = false;
    start_time_ = now();
    state_start_time_ = start_time_;
    offboard_cmd_count_ = 0;
    transitionTo(State::SEARCHING, "goal 接受，開始搜尋 tag");
  }

  // ===========================================================================
  //  狀態機
  // ===========================================================================
  void transitionTo(State s, const std::string & why)
  {
    if (state_ == s) {return;}
    RCLCPP_INFO(get_logger(), "[狀態] %s -> %s：%s（高度 %.2f m，水平誤差 %.2f m）",
      stateName(state_), stateName(s), why.c_str(),
      -local_position_.z, horizontalError());
    state_ = s;
    state_start_time_ = now();
  }

  static const char * stateName(State s)
  {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::SEARCHING: return "SEARCHING";
      case State::ALIGNING: return "ALIGNING";
      case State::DESCENDING: return "DESCENDING";
      case State::HANDOFF: return "HANDOFF";
    }
    return "?";
  }

  double horizontalError() const
  {
    return std::hypot(tag_offset_ned_[0], tag_offset_ned_[1]);
  }

  bool tagFresh() const
  {
    return tag_locked_ && consecutive_miss_ < lost_frames_;
  }

  // ===========================================================================
  //  20 Hz 主迴圈
  // ===========================================================================
  void loop()
  {
    // 沒有偵測進來就累計「連續丟失」。用計數而不是單幀判斷：
    // 設 1 的話偶爾掉一幀就退回 SEARCHING，狀態機會一直抖。
    if (state_ != State::IDLE) {
      const double since = (now() - last_detect_time_).seconds();
      if (since > 0.15) {++consecutive_miss_;}
      if (consecutive_miss_ >= lost_frames_) {consecutive_hits_ = 0;}
    }

    publishError();

    if (state_ == State::IDLE || !goal_handle_ || !goal_handle_->is_active()) {
      return;
    }

    if (cancel_requested_) {
      finish(false, PrecisionLand::Result::CODE_CANCELLED, "使用者取消");
      return;
    }
    if ((now() - start_time_).seconds() > total_timeout_s_) {
      finish(false, PrecisionLand::Result::CODE_TIMEOUT,
        "整體逾時，交還控制權讓上層決定");
      return;
    }
    if (!local_position_.xy_valid || !local_position_.z_valid) {
      finish(false, PrecisionLand::Result::CODE_BAD_POSE,
        "PX4 位置估計失效，立刻收手");
      return;
    }

    // advisor 模式只回報，不碰飛機 —— 上面的逾時與失效判斷照樣有效，
    // 這樣呼叫方仍然拿得到「找不到 tag」之類的結論。
    //
    // ⚠️ HANDOFF 之後一定要連 offboard_control_mode 和切模式指令一起停。
    //    只停 setpoint 是不夠的：ensureOffboard() 會每 0.5 秒把飛機拉回
    //    offboard，跟 PX4 的 LAND 模式互相拉扯 —— 飛機看起來確實降下去了，
    //    但落地偵測永遠穩不下來，最後以「30 秒未偵測到落地」逾時收場。
    //    這個現象完全不像模式衝突，第一眼會以為是收不到 land_detected。
    if (!advisor_ && state_ != State::HANDOFF) {
      publishOffboardControlMode();
      ensureOffboard();
    }

    switch (state_) {
      case State::SEARCHING: runSearching(); break;
      case State::ALIGNING: runAligning(); break;
      case State::DESCENDING: runDescending(); break;
      case State::HANDOFF: runHandoff(); break;
      default: break;
    }

    publishFeedback();
  }

  // --- SEARCHING：原地停住等 -------------------------------------------------
  // 刻意不繞圈也不升高：上層導航應該已經把飛機帶到降落點上方，看不到多半是
  // 還沒停穩。自作主張到處亂飛比較危險，逾時回報失敗讓上層決定要不要重試。
  void runSearching()
  {
    if (!advisor_) {holdPosition();}
    if (tagFresh()) {
      transitionTo(State::ALIGNING, "鎖定 tag");
      return;
    }
    if ((now() - state_start_time_).seconds() > search_timeout_s_) {
      finish(false, PrecisionLand::Result::CODE_NO_TAG,
        "搜尋逾時，一直沒看到 tag");
    }
  }

  // --- ALIGNING：高度不動，只修水平與航向 -------------------------------------
  void runAligning()
  {
    if (!tagFresh()) {
      transitionTo(State::SEARCHING, "tag 丟失");
      return;
    }
    if (!advisor_) {publishVelocity(alignVelocity(), holdAltitudeVz(approach_alt_));}

    const bool xy_ok = horizontalError() < xy_tolerance_;
    const bool yaw_ok = !align_yaw_ || std::fabs(yaw_error_) < yaw_tolerance_;
    if (xy_ok && yaw_ok) {
      transitionTo(State::DESCENDING, "對準完成，開始下降");
    }
  }

  // --- DESCENDING：邊降邊修 --------------------------------------------------
  void runDescending()
  {
    if (!tagFresh()) {
      transitionTo(State::SEARCHING, "下降途中 tag 丟失");
      return;
    }
    // 水平誤差變太大就先停止下降回去對準，不要歪著往下衝
    if (horizontalError() > xy_tolerance_ * 2.5) {
      transitionTo(State::ALIGNING, "水平誤差擴大，暫停下降");
      return;
    }
    if (!advisor_) {publishVelocity(alignVelocity(), descend_speed_);}

    if (-local_position_.z <= handoff_altitude_) {
      transitionTo(State::HANDOFF, "到達交接高度");
    }
  }

  // --- HANDOFF：交給 PX4 ------------------------------------------------------
  // 為什麼不自己降到底：下視相機 FOV 有限，tag 在某個高度以下會整個出框
  // （T1 對這組相機與 tag 實測是 0.5 m）。硬要用視覺降到地面，狀態機會在
  // 「下降」和「丟失」之間無限來回，永遠落不了地。
  // PX4 的降落有觸地偵測和自動 disarm，最後一段交給它最穩。
  void runHandoff()
  {
    if (advisor_) {
      finish(true, PrecisionLand::Result::CODE_SUCCESS,
        "advisor 模式：已到交接高度，控制權本來就在呼叫方");
      return;
    }
    // 只送一次；重複送會讓 PX4 反覆重新進入降落流程
    if (!land_cmd_sent_) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      land_cmd_sent_ = true;
      final_x_error_ = tag_offset_ned_[0];
      final_y_error_ = tag_offset_ned_[1];
      final_yaw_error_ = yaw_error_;
      RCLCPP_INFO(get_logger(),
        "已送出 NAV_LAND，交給 PX4 收尾。交接時殘餘誤差：北 %+.3f 東 %+.3f yaw %+.1f°",
        final_x_error_, final_y_error_, final_yaw_error_ * 180.0 / M_PI);
    }
    // 交接後不再發 offboard setpoint —— 繼續發會把 PX4 拉回 offboard，
    // 降落流程就被打斷了。
    if (land_detected_.landed) {
      finish(true, PrecisionLand::Result::CODE_SUCCESS, "已落地");
    } else if ((now() - state_start_time_).seconds() > 30.0) {
      finish(false, PrecisionLand::Result::CODE_TIMEOUT,
        "送出 NAV_LAND 後 30 秒仍未偵測到落地");
    }
  }

  // ===========================================================================
  //  控制輸出
  // ===========================================================================

  // 水平速度：誤差乘增益。不用 1.0 是因為一次修到位會過衝，然後來回震盪。
  std::array<double, 2> alignVelocity() const
  {
    return {
      clampAbs(kp_xy_ * tag_offset_ned_[0], max_speed_xy_),
      clampAbs(kp_xy_ * tag_offset_ned_[1], max_speed_xy_),
    };
  }

  // 定高用的垂直速度。NED 的 z 下為正，所以「目前高度低於目標」要給負值往上。
  double holdAltitudeVz(double target_alt) const
  {
    const double err = (-local_position_.z) - target_alt;   // 正 = 太高
    return clampAbs(kp_z_ * err, descend_speed_);
  }

  void holdPosition()
  {
    publishVelocity({0.0, 0.0}, holdAltitudeVz(approach_alt_));
  }

  void publishVelocity(const std::array<double, 2> & vne, double vd)
  {
    px4_msgs::msg::TrajectorySetpoint sp{};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // 位置全給 NaN 代表「這一層不控制」，PX4 才會走速度控制。
    sp.position = {nan, nan, nan};
    sp.velocity = {static_cast<float>(vne[0]), static_cast<float>(vne[1]),
      static_cast<float>(vd)};
    sp.acceleration = {nan, nan, nan};

    if (align_yaw_ && tagFresh()) {
      // 用絕對角度而不是角速度：角速度會累積誤差，絕對角度每一幀都是重新算的。
      // 但一次只准轉一小步，否則大角度會瞬間甩頭。
      const double step = clampAbs(kp_yaw_ * yaw_error_, max_yawspeed_ * 0.05);
      sp.yaw = static_cast<float>(wrapPi(local_position_.heading + step));
    } else {
      sp.yaw = nan;
    }
    sp.yawspeed = nan;
    sp.timestamp = nowMicros();
    setpoint_pub_->publish(sp);
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode m{};
    // 這幾個布林是「互斥的控制層級」，只能有一個為 true。
    // 我們全程用速度控制（水平靠增益、垂直靠定高或下降速度）。
    m.position = false;
    m.velocity = true;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    m.timestamp = nowMicros();
    offboard_mode_pub_->publish(m);
  }

  // 確保飛機在 offboard 模式。呼叫方停止發 setpoint 之後 PX4 會在約 0.5 秒內
  // 掉出 offboard，所以我們接手時要主動切回來。
  void ensureOffboard()
  {
    constexpr uint8_t kOffboard = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (vehicle_status_.nav_state == kOffboard) {return;}
    // 每 0.5 秒重送一次（20 Hz 迴圈 = 每 10 次）。
    // DO_SET_MODE 的 (param1=1, param2=6) 就是「切到 Offboard」。
    if (offboard_cmd_count_ % 10 == 0) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0f, 6.0f);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "送出切換 Offboard 指令，目前 nav_state=%d", vehicle_status_.nav_state);
    }
    ++offboard_cmd_count_;
  }

  void publishVehicleCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand m{};
    m.command = command;
    m.param1 = p1;
    m.param2 = p2;
    // target_system：單機 SITL 是 1，多機時第 N 台是 N+1。
    // 填錯的話指令會被安靜地忽略 —— 三機編隊最容易出錯的一格。
    m.target_system = static_cast<uint8_t>(target_system_);
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    // 一定要 true，否則 PX4 會當成內部指令而拒絕執行
    m.from_external = true;
    m.timestamp = nowMicros();
    command_pub_->publish(m);
  }

  void publishError()
  {
    geometry_msgs::msg::Vector3Stamped e;
    e.header.stamp = now();
    e.header.frame_id = "ned";
    e.vector.x = tag_offset_ned_[0];
    e.vector.y = tag_offset_ned_[1];
    e.vector.z = yaw_error_;
    error_pub_->publish(e);
  }

  void publishFeedback()
  {
    auto fb = std::make_shared<PrecisionLand::Feedback>();
    switch (state_) {
      case State::SEARCHING: fb->state = PrecisionLand::Feedback::STATE_SEARCHING; break;
      case State::ALIGNING: fb->state = PrecisionLand::Feedback::STATE_ALIGNING; break;
      case State::DESCENDING: fb->state = PrecisionLand::Feedback::STATE_DESCENDING; break;
      case State::HANDOFF: fb->state = PrecisionLand::Feedback::STATE_HANDOFF; break;
      default: fb->state = PrecisionLand::Feedback::STATE_SEARCHING; break;
    }
    fb->altitude = -local_position_.z;
    fb->xy_error = horizontalError();
    fb->yaw_error = yaw_error_;
    fb->tag_visible = tagFresh();
    goal_handle_->publish_feedback(fb);
  }

  void finish(bool success, uint8_t code, const std::string & why)
  {
    auto r = std::make_shared<PrecisionLand::Result>();
    r->success = success;
    r->result_code = code;
    r->final_x_error = land_cmd_sent_ ? final_x_error_ : tag_offset_ned_[0];
    r->final_y_error = land_cmd_sent_ ? final_y_error_ : tag_offset_ned_[1];
    r->final_yaw_error = land_cmd_sent_ ? final_yaw_error_ : yaw_error_;

    if (success) {
      RCLCPP_INFO(get_logger(), "降落完成：%s", why.c_str());
      goal_handle_->succeed(r);
    } else {
      RCLCPP_ERROR(get_logger(), "降落失敗（code=%d）：%s", code, why.c_str());
      goal_handle_->abort(r);
    }
    state_ = State::IDLE;
    land_cmd_sent_ = false;
    tag_locked_ = false;
    goal_handle_.reset();
  }

  uint64_t nowMicros() {return static_cast<uint64_t>(now().nanoseconds() / 1000);}

  // ===========================================================================
  //  成員
  // ===========================================================================
  std::string namespace_, camera_frame_, control_mode_, tag_frame_hint_;
  int target_system_{1}, tag_id_{0}, lock_frames_{5}, lost_frames_{15};
  double tag_size_{0.8}, min_decision_margin_{30.0};
  double xy_tolerance_{0.15}, yaw_tolerance_{0.09};
  double kp_xy_{0.5}, kp_z_{0.6}, kp_yaw_{0.8};
  double max_speed_xy_{0.8}, max_yawspeed_{0.5}, descend_speed_{0.3};
  double handoff_altitude_{1.0}, search_timeout_s_{30.0}, total_timeout_s_{120.0};
  Vec3 camera_offset_{}, camera_rpy_{};
  Mat3 r_optical_to_body_{};
  bool advisor_{false}, align_yaw_{true};

  State state_{State::IDLE};
  Vec3 tag_offset_ned_{};
  double tag_yaw_in_cam_{0.0}, yaw_error_{0.0}, approach_alt_{3.0};
  double final_x_error_{0.0}, final_y_error_{0.0}, final_yaw_error_{0.0};
  int consecutive_hits_{0}, consecutive_miss_{0}, offboard_cmd_count_{0};
  bool tag_locked_{false}, cancel_requested_{false}, land_cmd_sent_{false};
  bool has_local_position_{false};
  rclcpp::Time start_time_, state_start_time_, last_detect_time_{0, 0, RCL_ROS_TIME};

  px4_msgs::msg::VehicleLocalPosition local_position_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleLandDetected land_detected_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<PrecisionLand>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GoalHandle> goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrecisionLandNode>());
  rclcpp::shutdown();
  return 0;
}
