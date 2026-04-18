#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

double Clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(value, high));
}

double NormalizeAngle(const double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double DegreesToRadians(const double angle_deg) {
  return angle_deg * kPi / 180.0;
}

Eigen::Quaterniond ToEigenQuaternion(const geometry_msgs::Quaternion& quaternion_msg) {
  Eigen::Quaterniond quaternion(
      quaternion_msg.w,
      quaternion_msg.x,
      quaternion_msg.y,
      quaternion_msg.z);

  if (quaternion.norm() < 1e-6) {
    return Eigen::Quaterniond::Identity();
  }

  quaternion.normalize();
  return quaternion;
}

Eigen::Quaterniond QuaternionFromRpy(
    const double roll_rad,
    const double pitch_rad,
    const double yaw_rad) {
  Eigen::Quaterniond quaternion(
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
  quaternion.normalize();
  return quaternion;
}

struct PlanningOutput {
  geometry_msgs::PoseStamped setpoint;
  bool avoidance_active = false;
  double raw_corridor_clearance_m = std::numeric_limits<double>::infinity();
  double selected_corridor_clearance_m = std::numeric_limits<double>::infinity();
  double selected_heading_rad = 0.0;
};

class ObstacleAvoidanceNode {
 public:
  ObstacleAvoidanceNode() : nh_(), pnh_("~") {
    LoadParameters();
    SetupRosInterfaces();

    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_rate_hz_),
        &ObstacleAvoidanceNode::PublishTimerCallback,
        this);

    ROS_INFO_STREAM(
        "Obstacle avoidance ready. enable_avoidance=" << (enable_avoidance_ ? "true" : "false")
        << ", raw_pose_setpoint_topic=" << raw_pose_setpoint_topic_
        << ", safe_pose_setpoint_topic=" << safe_pose_setpoint_topic_
        << ", lidar_topic=" << lidar_topic_);
  }

 private:
  void LoadParameters() {
    pnh_.param<std::string>("raw_pose_setpoint_topic", raw_pose_setpoint_topic_, "/rmua/control/path/raw_pose_setpoint");
    pnh_.param<std::string>("safe_pose_setpoint_topic", safe_pose_setpoint_topic_, "/rmua/control/basic/pose_setpoint");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/rmua/sensors/drone_1/pose_gt");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/rmua/sensors/drone_1/imu");
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/rmua/sensors/drone_1/lidar");
    pnh_.param<std::string>("world_frame_id", world_frame_id_, "world_ned");
    pnh_.param<std::string>("debug_pose_topic", debug_pose_topic_, "/rmua/control/avoidance/target_pose");
    pnh_.param<std::string>("active_topic", active_topic_, "/rmua/control/avoidance/active");

    pnh_.param("enable_avoidance", enable_avoidance_, true);
    pnh_.param("publish_rate_hz", publish_rate_hz_, 20.0);
    pnh_.param("input_timeout_s", input_timeout_s_, 0.5);
    pnh_.param("lidar_timeout_s", lidar_timeout_s_, 0.5);
    pnh_.param("near_target_passthrough_distance_m", near_target_passthrough_distance_m_, 1.0);
    pnh_.param("stable_hold_passthrough_distance_m", stable_hold_passthrough_distance_m_, 2.0);
    pnh_.param("stable_hold_passthrough_vertical_m", stable_hold_passthrough_vertical_m_, 1.0);
    pnh_.param("min_obstacle_range_m", min_obstacle_range_m_, 0.6);
    pnh_.param("max_obstacle_range_m", max_obstacle_range_m_, 25.0);
    pnh_.param("obstacle_height_above_m", obstacle_height_above_m_, 1.0);
    pnh_.param("obstacle_height_below_m", obstacle_height_below_m_, 1.4);
    pnh_.param("obstacle_inflation_radius_m", obstacle_inflation_radius_m_, 1.0);
    pnh_.param("raw_corridor_half_angle_deg", raw_corridor_half_angle_deg_, 12.0);
    pnh_.param("raw_path_clearance_m", raw_path_clearance_m_, 6.0);
    pnh_.param("selected_path_clearance_m", selected_path_clearance_m_, 5.0);
    pnh_.param("forward_safety_buffer_m", forward_safety_buffer_m_, 1.0);
    pnh_.param("emergency_stop_distance_m", emergency_stop_distance_m_, 2.5);
    pnh_.param("avoidance_lookahead_m", avoidance_lookahead_m_, 8.0);
    pnh_.param("min_avoidance_step_m", min_avoidance_step_m_, 3.0);
    pnh_.param("max_heading_offset_deg", max_heading_offset_deg_, 80.0);
    pnh_.param("clearance_recovery_hold_s", clearance_recovery_hold_s_, 0.6);
    pnh_.param("side_switch_penalty_rad", side_switch_penalty_rad_, 0.35);
    pnh_.param("sector_count", sector_count_, 72);
    pnh_.param("lidar_offset_x_m", lidar_offset_body_m_.x(), 0.0);
    pnh_.param("lidar_offset_y_m", lidar_offset_body_m_.y(), 0.0);
    pnh_.param("lidar_offset_z_m", lidar_offset_body_m_.z(), -0.05);

    double lidar_roll_deg = 0.0;
    double lidar_pitch_deg = 0.0;
    double lidar_yaw_deg = 0.0;
    pnh_.param("lidar_roll_deg", lidar_roll_deg, 0.0);
    pnh_.param("lidar_pitch_deg", lidar_pitch_deg, 0.0);
    pnh_.param("lidar_yaw_deg", lidar_yaw_deg, 0.0);
    lidar_rotation_body_from_lidar_ = QuaternionFromRpy(
        DegreesToRadians(lidar_roll_deg),
        DegreesToRadians(lidar_pitch_deg),
        DegreesToRadians(lidar_yaw_deg));

    publish_rate_hz_ = std::max(5.0, publish_rate_hz_);
    input_timeout_s_ = std::max(0.1, input_timeout_s_);
    lidar_timeout_s_ = std::max(0.1, lidar_timeout_s_);
    near_target_passthrough_distance_m_ = std::max(0.0, near_target_passthrough_distance_m_);
    stable_hold_passthrough_distance_m_ = std::max(
        near_target_passthrough_distance_m_, stable_hold_passthrough_distance_m_);
    stable_hold_passthrough_vertical_m_ = std::max(0.0, stable_hold_passthrough_vertical_m_);
    min_obstacle_range_m_ = std::max(0.05, min_obstacle_range_m_);
    max_obstacle_range_m_ = std::max(min_obstacle_range_m_ + 0.5, max_obstacle_range_m_);
    obstacle_height_above_m_ = std::max(0.0, obstacle_height_above_m_);
    obstacle_height_below_m_ = std::max(0.0, obstacle_height_below_m_);
    obstacle_inflation_radius_m_ = std::max(0.1, obstacle_inflation_radius_m_);
    raw_corridor_half_angle_deg_ = Clamp(raw_corridor_half_angle_deg_, 1.0, 45.0);
    raw_path_clearance_m_ = std::max(1.0, raw_path_clearance_m_);
    selected_path_clearance_m_ = std::max(1.0, selected_path_clearance_m_);
    forward_safety_buffer_m_ = std::max(0.1, forward_safety_buffer_m_);
    emergency_stop_distance_m_ = std::max(0.5, emergency_stop_distance_m_);
    avoidance_lookahead_m_ = std::max(1.0, avoidance_lookahead_m_);
    min_avoidance_step_m_ = Clamp(min_avoidance_step_m_, 0.0, avoidance_lookahead_m_);
    max_heading_offset_deg_ = Clamp(max_heading_offset_deg_, 5.0, 120.0);
    clearance_recovery_hold_s_ = std::max(0.0, clearance_recovery_hold_s_);
    side_switch_penalty_rad_ = std::max(0.0, side_switch_penalty_rad_);
    sector_count_ = std::max(24, sector_count_);

    sector_width_rad_ = 2.0 * kPi / static_cast<double>(sector_count_);
    raw_corridor_half_angle_rad_ = DegreesToRadians(raw_corridor_half_angle_deg_);
    max_heading_offset_rad_ = DegreesToRadians(max_heading_offset_deg_);
    sector_clearance_m_.assign(sector_count_, std::numeric_limits<double>::infinity());
  }

  void SetupRosInterfaces() {
    raw_pose_setpoint_subscriber_ = nh_.subscribe(
        raw_pose_setpoint_topic_, 20, &ObstacleAvoidanceNode::RawPoseSetpointCallback, this);
    pose_subscriber_ = nh_.subscribe(pose_topic_, 20, &ObstacleAvoidanceNode::PoseCallback, this);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 100, &ObstacleAvoidanceNode::ImuCallback, this);
    lidar_subscriber_ = nh_.subscribe(lidar_topic_, 5, &ObstacleAvoidanceNode::LidarCallback, this);

    safe_pose_setpoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(safe_pose_setpoint_topic_, 10);
    debug_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(debug_pose_topic_, 10);
    active_publisher_ = nh_.advertise<std_msgs::Bool>(active_topic_, 10, true);
  }

  void RawPoseSetpointCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_raw_pose_setpoint_ = *message;
    latest_raw_pose_setpoint_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    has_raw_pose_setpoint_ = true;
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    current_position_world_ned_ = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    current_attitude_world_ned_ = ToEigenQuaternion(message->pose.orientation);
    has_pose_ = true;
  }

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    has_imu_ = true;
  }

  void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_lidar_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    has_lidar_model_ = BuildLidarModelLocked(*message);
  }

  void PublishTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();

    PlanningOutput planning_output;
    bool previous_avoidance_active = false;
    bool current_avoidance_active = false;
    bool has_output = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!has_raw_pose_setpoint_) {
        ROS_WARN_THROTTLE(2.0, "Obstacle avoidance is waiting for raw pose setpoints.");
        return;
      }

      previous_avoidance_active = avoidance_active_;
      planning_output = BuildPlanningOutputLocked(now);
      current_avoidance_active = planning_output.avoidance_active;
      has_output = true;
    }

    if (!has_output) {
      return;
    }

    safe_pose_setpoint_publisher_.publish(planning_output.setpoint);
    debug_pose_publisher_.publish(planning_output.setpoint);

    std_msgs::Bool active_message;
    active_message.data = planning_output.avoidance_active;
    active_publisher_.publish(active_message);

    if (!previous_avoidance_active && current_avoidance_active) {
      ROS_WARN_STREAM(
          "Obstacle avoidance engaged. raw_clearance=" << planning_output.raw_corridor_clearance_m
          << " m, selected_clearance=" << planning_output.selected_corridor_clearance_m
          << " m, selected_heading_deg="
          << (planning_output.selected_heading_rad * 180.0 / kPi));
    } else if (previous_avoidance_active && !current_avoidance_active) {
      ROS_INFO("Obstacle avoidance released and returned to raw path setpoints.");
    }
  }

  PlanningOutput BuildPlanningOutputLocked(const ros::Time& now) {
    PlanningOutput output;
    output.setpoint = latest_raw_pose_setpoint_;
    if (output.setpoint.header.stamp.isZero()) {
      output.setpoint.header.stamp = now;
    }
    if (output.setpoint.header.frame_id.empty()) {
      output.setpoint.header.frame_id = world_frame_id_;
    }

    if (!enable_avoidance_) {
      ResetAvoidanceStateLocked();
      return output;
    }

    if (!HasFreshInputsLocked(now)) {
      ResetAvoidanceStateLocked();
      ROS_WARN_THROTTLE(2.0, "Obstacle avoidance is bypassing because pose, imu, or lidar is stale.");
      return output;
    }

    const Eigen::Vector3d raw_target_position_world_ned(
        latest_raw_pose_setpoint_.pose.position.x,
        latest_raw_pose_setpoint_.pose.position.y,
        latest_raw_pose_setpoint_.pose.position.z);
    const Eigen::Vector3d raw_delta_world_ned =
        raw_target_position_world_ned - current_position_world_ned_;
    const double raw_horizontal_distance_m = std::hypot(raw_delta_world_ned.x(), raw_delta_world_ned.y());

    if (raw_horizontal_distance_m <= near_target_passthrough_distance_m_) {
      ResetAvoidanceStateLocked();
      return output;
    }

    if (IsStableHoldProtectedLocked(raw_delta_world_ned)) {
      ResetAvoidanceStateLocked();
      return output;
    }

    const Eigen::Vector3d raw_delta_body_ned = current_attitude_world_ned_.inverse() * raw_delta_world_ned;
    const Eigen::Vector2d raw_delta_body_horizontal(raw_delta_body_ned.x(), raw_delta_body_ned.y());
    if (raw_delta_body_horizontal.norm() <= 1e-6) {
      ResetAvoidanceStateLocked();
      return output;
    }

    const double desired_heading_rad = std::atan2(raw_delta_body_horizontal.y(), raw_delta_body_horizontal.x());
    const double raw_corridor_clearance_m = CorridorClearanceLocked(desired_heading_rad, raw_corridor_half_angle_rad_);
    output.raw_corridor_clearance_m = raw_corridor_clearance_m;

    const bool raw_corridor_clear = raw_corridor_clearance_m >= raw_path_clearance_m_;
    if (raw_corridor_clear) {
      if (raw_corridor_clear_since_.isZero()) {
        raw_corridor_clear_since_ = now;
      }
    } else {
      raw_corridor_clear_since_ = ros::Time();
    }

    if (avoidance_active_
        && !raw_corridor_clear_since_.isZero()
        && (now - raw_corridor_clear_since_).toSec() >= clearance_recovery_hold_s_) {
      ResetAvoidanceStateLocked();
      return output;
    }

    if (!avoidance_active_ && raw_corridor_clear) {
      return output;
    }

    double selected_heading_rad = desired_heading_rad;
    double selected_corridor_clearance_m = raw_corridor_clearance_m;
    if (!TrySelectHeadingLocked(
            desired_heading_rad,
            raw_horizontal_distance_m,
            &selected_heading_rad,
            &selected_corridor_clearance_m)) {
      selected_heading_rad = desired_heading_rad;
      selected_corridor_clearance_m = raw_corridor_clearance_m;
    }

    const double desired_step_m = std::min(raw_horizontal_distance_m, avoidance_lookahead_m_);
    const double max_safe_step_m = std::max(0.0, selected_corridor_clearance_m - forward_safety_buffer_m_);

    double selected_step_m = std::min(desired_step_m, max_safe_step_m);
    if (selected_step_m < min_avoidance_step_m_ && max_safe_step_m >= min_avoidance_step_m_) {
      selected_step_m = std::min(desired_step_m, min_avoidance_step_m_);
    }

    if (selected_step_m <= 1e-3
        && selected_corridor_clearance_m < emergency_stop_distance_m_) {
      selected_step_m = 0.0;
    }

    const Eigen::Matrix3d attitude_world_from_body = current_attitude_world_ned_.toRotationMatrix();
    Eigen::Vector3d forward_world_ned = attitude_world_from_body.col(0);
    forward_world_ned.z() = 0.0;
    if (forward_world_ned.head<2>().norm() <= 1e-6) {
      forward_world_ned = Eigen::Vector3d::UnitX();
    } else {
      forward_world_ned.normalize();
    }

    Eigen::Vector3d right_world_ned = attitude_world_from_body.col(1);
    right_world_ned.z() = 0.0;
    if (right_world_ned.head<2>().norm() <= 1e-6) {
      right_world_ned = Eigen::Vector3d(-forward_world_ned.y(), forward_world_ned.x(), 0.0);
    } else {
      right_world_ned.normalize();
    }

    const Eigen::Vector2d selected_local_offset(
        std::cos(selected_heading_rad) * selected_step_m,
        std::sin(selected_heading_rad) * selected_step_m);
    const Eigen::Vector3d selected_world_offset =
        forward_world_ned * selected_local_offset.x() + right_world_ned * selected_local_offset.y();

    output.setpoint.pose.position.x = current_position_world_ned_.x() + selected_world_offset.x();
    output.setpoint.pose.position.y = current_position_world_ned_.y() + selected_world_offset.y();
    output.setpoint.pose.position.z = raw_target_position_world_ned.z();
    output.avoidance_active = true;
    output.selected_heading_rad = selected_heading_rad;
    output.selected_corridor_clearance_m = selected_corridor_clearance_m;

    avoidance_active_ = true;
    latched_heading_rad_ = selected_heading_rad;
    return output;
  }

  bool HasFreshInputsLocked(const ros::Time& now) const {
    if (!has_pose_ || !has_imu_ || !has_lidar_model_) {
      return false;
    }

    if ((now - latest_pose_stamp_).toSec() > input_timeout_s_) {
      return false;
    }
    if ((now - latest_imu_stamp_).toSec() > input_timeout_s_) {
      return false;
    }
    if ((now - latest_lidar_stamp_).toSec() > lidar_timeout_s_) {
      return false;
    }

    return true;
  }

  bool IsStableHoldProtectedLocked(const Eigen::Vector3d& raw_delta_world_ned) const {
    const double horizontal_distance_m = std::hypot(raw_delta_world_ned.x(), raw_delta_world_ned.y());
    return horizontal_distance_m <= stable_hold_passthrough_distance_m_
        && std::abs(raw_delta_world_ned.z()) <= stable_hold_passthrough_vertical_m_;
  }

  bool BuildLidarModelLocked(const sensor_msgs::PointCloud2& message) {
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (const sensor_msgs::PointField& field : message.fields) {
      if (field.name == "x") {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y") {
        y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z") {
        z_offset = static_cast<int>(field.offset);
      }
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || message.point_step < 12) {
      ROS_WARN_THROTTLE(2.0, "Lidar message is missing x/y/z float32 fields. Avoidance stays in bypass.");
      return false;
    }

    std::fill(
        sector_clearance_m_.begin(),
        sector_clearance_m_.end(),
        std::numeric_limits<double>::infinity());

    const std::size_t point_count = static_cast<std::size_t>(message.width) * static_cast<std::size_t>(message.height);
    if (point_count == 0 || message.data.empty()) {
      return true;
    }

    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const std::size_t data_offset = point_index * static_cast<std::size_t>(message.point_step);
      if (data_offset + static_cast<std::size_t>(std::max({x_offset, y_offset, z_offset}) + 4) > message.data.size()) {
        break;
      }

      float point_x = 0.0f;
      float point_y = 0.0f;
      float point_z = 0.0f;
      std::memcpy(&point_x, &message.data[data_offset + static_cast<std::size_t>(x_offset)], sizeof(float));
      std::memcpy(&point_y, &message.data[data_offset + static_cast<std::size_t>(y_offset)], sizeof(float));
      std::memcpy(&point_z, &message.data[data_offset + static_cast<std::size_t>(z_offset)], sizeof(float));

      if (!std::isfinite(point_x) || !std::isfinite(point_y) || !std::isfinite(point_z)) {
        continue;
      }

      const Eigen::Vector3d point_lidar(point_x, point_y, point_z);
      const Eigen::Vector3d point_body = lidar_rotation_body_from_lidar_ * point_lidar + lidar_offset_body_m_;
      const double point_distance_m = point_body.norm();
      const double point_horizontal_distance_m = std::hypot(point_body.x(), point_body.y());
      if (point_distance_m < min_obstacle_range_m_ || point_distance_m > max_obstacle_range_m_) {
        continue;
      }
      if (point_horizontal_distance_m <= 1e-6) {
        continue;
      }
      if (point_body.z() < -obstacle_height_above_m_ || point_body.z() > obstacle_height_below_m_) {
        continue;
      }

      const double point_heading_rad = std::atan2(point_body.y(), point_body.x());
      const int center_sector_index = AngleToSectorIndexLocked(point_heading_rad);
      const double inflation_heading_rad = std::atan2(
          obstacle_inflation_radius_m_, std::max(point_horizontal_distance_m, 0.1));
      const int inflated_sector_span = std::max(
          0,
          static_cast<int>(std::ceil(inflation_heading_rad / sector_width_rad_)));
      for (int sector_offset = -inflated_sector_span; sector_offset <= inflated_sector_span; ++sector_offset) {
        const int wrapped_sector_index = WrapSectorIndexLocked(center_sector_index + sector_offset);
        sector_clearance_m_[wrapped_sector_index] = std::min(
            sector_clearance_m_[wrapped_sector_index], point_horizontal_distance_m);
      }
    }

    return true;
  }
  double CorridorClearanceLocked(const double heading_rad, const double half_angle_rad) const {
    if (sector_clearance_m_.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    const int center_sector_index = AngleToSectorIndexLocked(heading_rad);
    const int half_sector_span = std::max(
        0,
        static_cast<int>(std::ceil(half_angle_rad / sector_width_rad_)));
    double corridor_clearance_m = std::numeric_limits<double>::infinity();
    for (int sector_offset = -half_sector_span; sector_offset <= half_sector_span; ++sector_offset) {
      const int wrapped_sector_index = WrapSectorIndexLocked(center_sector_index + sector_offset);
      corridor_clearance_m = std::min(corridor_clearance_m, sector_clearance_m_[wrapped_sector_index]);
    }
    return corridor_clearance_m;
  }

  bool TrySelectHeadingLocked(
      const double desired_heading_rad,
      const double raw_horizontal_distance_m,
      double* selected_heading_rad,
      double* selected_corridor_clearance_m) {
    if (selected_heading_rad == nullptr || selected_corridor_clearance_m == nullptr) {
      return false;
    }

    const int max_heading_sector_span = std::max(
        1,
        static_cast<int>(std::ceil(max_heading_offset_rad_ / sector_width_rad_)));
    const double candidate_half_angle_rad = std::max(raw_corridor_half_angle_rad_ * 0.75, sector_width_rad_);
    const double required_clearance_m = std::min(raw_horizontal_distance_m, selected_path_clearance_m_);
    const double latched_heading_sign = std::abs(latched_heading_rad_) <= 1e-6 ? 0.0 : std::copysign(1.0, latched_heading_rad_);

    bool found_candidate = false;
    double best_score = std::numeric_limits<double>::infinity();
    double best_heading_rad = desired_heading_rad;
    double best_clearance_m = CorridorClearanceLocked(desired_heading_rad, candidate_half_angle_rad);

    for (int sector_offset = -max_heading_sector_span; sector_offset <= max_heading_sector_span; ++sector_offset) {
      const double candidate_heading_rad = NormalizeAngle(
          desired_heading_rad + static_cast<double>(sector_offset) * sector_width_rad_);
      const double candidate_corridor_clearance_m =
          CorridorClearanceLocked(candidate_heading_rad, candidate_half_angle_rad);
      const double heading_error_rad =
          std::abs(NormalizeAngle(candidate_heading_rad - desired_heading_rad));

      double score = heading_error_rad;
      if (candidate_corridor_clearance_m < required_clearance_m) {
        score += 2.0 * (required_clearance_m - candidate_corridor_clearance_m);
      }
      if (avoidance_active_ && latched_heading_sign != 0.0) {
        const double candidate_heading_sign =
            std::abs(candidate_heading_rad) <= 1e-6 ? latched_heading_sign : std::copysign(1.0, candidate_heading_rad);
        if (candidate_heading_sign != latched_heading_sign) {
          score += side_switch_penalty_rad_;
        }
      }

      if (!found_candidate || score < best_score) {
        found_candidate = true;
        best_score = score;
        best_heading_rad = candidate_heading_rad;
        best_clearance_m = candidate_corridor_clearance_m;
      }
    }

    *selected_heading_rad = best_heading_rad;
    *selected_corridor_clearance_m = best_clearance_m;
    return found_candidate;
  }

  int AngleToSectorIndexLocked(const double heading_rad) const {
    const double normalized_heading_rad = NormalizeAngle(heading_rad);
    const double shifted_heading_rad = normalized_heading_rad + kPi;
    int sector_index = static_cast<int>(std::floor(shifted_heading_rad / sector_width_rad_));
    if (sector_index >= sector_count_) {
      sector_index = sector_count_ - 1;
    }
    return WrapSectorIndexLocked(sector_index);
  }

  int WrapSectorIndexLocked(const int sector_index) const {
    int wrapped_sector_index = sector_index % sector_count_;
    if (wrapped_sector_index < 0) {
      wrapped_sector_index += sector_count_;
    }
    return wrapped_sector_index;
  }

  void ResetAvoidanceStateLocked() {
    avoidance_active_ = false;
    latched_heading_rad_ = 0.0;
    raw_corridor_clear_since_ = ros::Time();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber raw_pose_setpoint_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber lidar_subscriber_;
  ros::Publisher safe_pose_setpoint_publisher_;
  ros::Publisher debug_pose_publisher_;
  ros::Publisher active_publisher_;
  ros::Timer publish_timer_;

  std::string raw_pose_setpoint_topic_;
  std::string safe_pose_setpoint_topic_;
  std::string pose_topic_;
  std::string imu_topic_;
  std::string lidar_topic_;
  std::string world_frame_id_;
  std::string debug_pose_topic_;
  std::string active_topic_;

  bool enable_avoidance_ = true;
  double publish_rate_hz_ = 20.0;
  double input_timeout_s_ = 0.5;
  double lidar_timeout_s_ = 0.5;
  double near_target_passthrough_distance_m_ = 1.0;
  double stable_hold_passthrough_distance_m_ = 2.0;
  double stable_hold_passthrough_vertical_m_ = 1.0;
  double min_obstacle_range_m_ = 0.6;
  double max_obstacle_range_m_ = 25.0;
  double obstacle_height_above_m_ = 1.0;
  double obstacle_height_below_m_ = 1.4;
  double obstacle_inflation_radius_m_ = 1.0;
  double raw_corridor_half_angle_deg_ = 12.0;
  double raw_corridor_half_angle_rad_ = DegreesToRadians(12.0);
  double raw_path_clearance_m_ = 6.0;
  double selected_path_clearance_m_ = 5.0;
  double forward_safety_buffer_m_ = 1.0;
  double emergency_stop_distance_m_ = 2.5;
  double avoidance_lookahead_m_ = 8.0;
  double min_avoidance_step_m_ = 3.0;
  double max_heading_offset_deg_ = 80.0;
  double max_heading_offset_rad_ = DegreesToRadians(80.0);
  double clearance_recovery_hold_s_ = 0.6;
  double side_switch_penalty_rad_ = 0.35;
  int sector_count_ = 72;
  double sector_width_rad_ = 2.0 * kPi / 72.0;

  Eigen::Vector3d lidar_offset_body_m_ = Eigen::Vector3d(0.0, 0.0, -0.05);
  Eigen::Quaterniond lidar_rotation_body_from_lidar_ = Eigen::Quaterniond::Identity();

  bool has_raw_pose_setpoint_ = false;
  bool has_pose_ = false;
  bool has_imu_ = false;
  bool has_lidar_model_ = false;
  bool avoidance_active_ = false;
  ros::Time latest_raw_pose_setpoint_stamp_;
  ros::Time latest_pose_stamp_;
  ros::Time latest_imu_stamp_;
  ros::Time latest_lidar_stamp_;
  ros::Time raw_corridor_clear_since_;
  geometry_msgs::PoseStamped latest_raw_pose_setpoint_;
  Eigen::Vector3d current_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond current_attitude_world_ned_ = Eigen::Quaterniond::Identity();
  double latched_heading_rad_ = 0.0;
  std::vector<double> sector_clearance_m_;
  std::mutex data_mutex_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_avoidance_node");
  ObstacleAvoidanceNode node;
  ros::spin();
  return 0;
}