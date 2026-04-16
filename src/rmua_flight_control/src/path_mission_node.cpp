#include <airsim_ros/RotorPWM.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kMissionFailureExitCode = 2;
constexpr int kMissionSuccessExitCode = 0;

double Clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(value, high));
}

double HorizontalNorm(const Eigen::Vector3d& vector) {
  return std::hypot(vector.x(), vector.y());
}

double HorizontalTurnAngleRad(
    const Eigen::Vector3d& first_segment,
    const Eigen::Vector3d& second_segment) {
  const Eigen::Vector2d first_horizontal(first_segment.x(), first_segment.y());
  const Eigen::Vector2d second_horizontal(second_segment.x(), second_segment.y());
  const double first_norm = first_horizontal.norm();
  const double second_norm = second_horizontal.norm();
  if (first_norm <= 1e-6 || second_norm <= 1e-6) {
    return 0.0;
  }

  const double cosine = Clamp(
      first_horizontal.dot(second_horizontal) / (first_norm * second_norm),
      -1.0,
      1.0);
  return std::acos(cosine);
}

double LinearInterpolate(const double start, const double end, const double ratio) {
  return start + (end - start) * Clamp(ratio, 0.0, 1.0);
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

geometry_msgs::Quaternion ToRosQuaternion(const Eigen::Quaterniond& quaternion) {
  geometry_msgs::Quaternion message;
  message.w = quaternion.w();
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  return message;
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::vector<int> ParseIntegerCsv(const std::string& csv_text) {
  std::vector<int> values;
  if (csv_text.empty()) {
    return values;
  }

  std::stringstream stream(csv_text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (token.empty()) {
      continue;
    }
    values.push_back(std::stoi(token));
  }
  return values;
}

struct Waypoint {
  int csv_index = 0;
  Eigen::Vector3d position_world_ned = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_world_ned = Eigen::Quaterniond::Identity();
};

struct SampledTarget {
  Eigen::Vector3d position_world_ned = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_world_ned = Eigen::Quaterniond::Identity();
  int progress_index = 0;
  int lookahead_index = 0;
  double distance_to_final = 0.0;
  double euclidean_distance_to_final = 0.0;
  std::string pending_name;
  bool stable_hold_active = false;
  int stable_point_index = -1;
};

class PathMissionNode {
 public:
  PathMissionNode() : nh_(), pnh_("~") {
    LoadParameters();
    LoadPath();
    SetupRosInterfaces();

    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_rate_hz_),
        &PathMissionNode::PublishTimerCallback,
        this);

    ROS_INFO_STREAM("Path mission ready. path_csv=" << path_csv_
                    << ", target_waypoint_index=" << target_waypoint_index_
                    << ", lookahead_distance_m=" << lookahead_distance_m_
                    << ", stable_points=" << stable_point_indices_csv_);
  }

 private:
  void LoadParameters() {
    pnh_.param<std::string>("path_csv", path_csv_, "/home/hao/workspace-RosNoetic/rmua2026/drone_path.csv");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/rmua/sensors/drone_1/pose_gt");
    pnh_.param<std::string>("rotor_pwm_topic", rotor_pwm_topic_, "/rmua/sensors/drone_1/debug/rotor_pwm");
    pnh_.param<std::string>("pose_setpoint_topic", pose_setpoint_topic_, "/rmua/control/basic/pose_setpoint");
    pnh_.param<std::string>("enable_topic", enable_topic_, "/rmua/control/basic/enable");
    pnh_.param<std::string>("world_frame_id", world_frame_id_, "world_ned");
    pnh_.param("target_waypoint_index", target_waypoint_index_, -1);
    pnh_.param("publish_rate_hz", publish_rate_hz_, 20.0);
    pnh_.param<std::string>("stable_point_indices", stable_point_indices_csv_, "69");
    pnh_.param("stable_point_hold_time_s", stable_point_hold_time_s_, 3.0);
    pnh_.param("lookahead_distance_m", lookahead_distance_m_, 30.0);
    pnh_.param("min_lookahead_distance_m", min_lookahead_distance_m_, 0.0);
    pnh_.param("lookahead_error_reduction_gain", lookahead_error_reduction_gain_, 0.0);
    pnh_.param("turn_preview_distance_m", turn_preview_distance_m_, 0.0);
    pnh_.param("turn_min_lookahead_distance_m", turn_min_lookahead_distance_m_, 0.0);
    pnh_.param("turn_lookahead_min_scale", turn_lookahead_min_scale_, 1.0);
    pnh_.param("turn_angle_reduction_gain", turn_angle_reduction_gain_, 0.0);
    pnh_.param("turn_short_segment_ignore_distance_m", turn_short_segment_ignore_distance_m_, 0.0);
    pnh_.param("terminal_lookahead_distance_m", terminal_lookahead_distance_m_, 0.0);
    pnh_.param("terminal_slowdown_distance_m", terminal_slowdown_distance_m_, 0.0);
    pnh_.param("waypoint_acceptance_radius_m", waypoint_acceptance_radius_m_, 4.0);
    pnh_.param("final_acceptance_radius_m", final_acceptance_radius_m_, 3.0);
    pnh_.param("completion_hold_time_s", completion_hold_time_s_, 0.0);
    pnh_.param("climb_priority_threshold_m", climb_priority_threshold_m_, 1.0);
    pnh_.param("climb_horizontal_leash_m", climb_horizontal_leash_m_, 1.0);
    pnh_.param("climb_horizontal_leash_scale", climb_horizontal_leash_scale_, 0.25);
    pnh_.param("climb_horizontal_leash_max_m", climb_horizontal_leash_max_m_, 8.0);
    pnh_.param("climb_vertical_release_distance_m", climb_vertical_release_distance_m_, 10.0);
    pnh_.param("climb_vertical_step_m", climb_vertical_step_m_, 4.0);
    pnh_.param("climb_preview_waypoints", climb_preview_waypoints_, 4);
    pnh_.param("descent_release_distance_m", descent_release_distance_m_, 0.0);
    pnh_.param("enable_controller_on_start", enable_controller_on_start_, true);
    pnh_.param("failure_abort_on_detection", failure_abort_on_detection_, true);
    pnh_.param("failure_progress_stall_timeout_s", failure_progress_stall_timeout_s_, 3.0);
    pnh_.param("failure_stall_motion_radius_m", failure_stall_motion_radius_m_, 1.5);
    pnh_.param("failure_distance_improvement_m", failure_distance_improvement_m_, 1.5);
    pnh_.param("failure_min_distance_to_goal_m", failure_min_distance_to_goal_m_, 12.0);
    pnh_.param("failure_zero_pwm_timeout_s", failure_zero_pwm_timeout_s_, 1.0);
    pnh_.param("failure_zero_pwm_threshold", failure_zero_pwm_threshold_, 0.08);
    pnh_.param("failure_zero_pwm_min_progress", failure_zero_pwm_min_progress_, 8);
    pnh_.param("failure_grace_period_s", failure_grace_period_s_, 8.0);

    publish_rate_hz_ = std::max(2.0, publish_rate_hz_);
    stable_point_hold_time_s_ = std::max(0.0, stable_point_hold_time_s_);
    lookahead_distance_m_ = std::max(0.0, lookahead_distance_m_);
    min_lookahead_distance_m_ = Clamp(min_lookahead_distance_m_, 0.0, lookahead_distance_m_);
    lookahead_error_reduction_gain_ = std::max(0.0, lookahead_error_reduction_gain_);
    if (turn_preview_distance_m_ <= 0.0) {
      turn_preview_distance_m_ = lookahead_distance_m_;
    }
    turn_preview_distance_m_ = std::max(0.0, turn_preview_distance_m_);
    if (turn_min_lookahead_distance_m_ <= 0.0) {
      turn_min_lookahead_distance_m_ = min_lookahead_distance_m_;
    }
    turn_min_lookahead_distance_m_ = Clamp(turn_min_lookahead_distance_m_, 0.0, min_lookahead_distance_m_);
    turn_lookahead_min_scale_ = Clamp(turn_lookahead_min_scale_, 0.1, 1.0);
    turn_angle_reduction_gain_ = std::max(0.0, turn_angle_reduction_gain_);
    turn_short_segment_ignore_distance_m_ = std::max(0.0, turn_short_segment_ignore_distance_m_);
    terminal_lookahead_distance_m_ = Clamp(terminal_lookahead_distance_m_, 0.0, lookahead_distance_m_);
    terminal_slowdown_distance_m_ = std::max(0.0, terminal_slowdown_distance_m_);
    waypoint_acceptance_radius_m_ = std::max(0.5, waypoint_acceptance_radius_m_);
    final_acceptance_radius_m_ = std::max(0.5, final_acceptance_radius_m_);
    completion_hold_time_s_ = std::max(0.0, completion_hold_time_s_);
    climb_priority_threshold_m_ = std::max(0.0, climb_priority_threshold_m_);
    climb_horizontal_leash_m_ = std::max(0.0, climb_horizontal_leash_m_);
    climb_horizontal_leash_scale_ = Clamp(climb_horizontal_leash_scale_, 0.0, 1.0);
    climb_horizontal_leash_max_m_ = std::max(climb_horizontal_leash_m_, climb_horizontal_leash_max_m_);
    climb_vertical_release_distance_m_ = std::max(0.0, climb_vertical_release_distance_m_);
    climb_vertical_step_m_ = std::max(0.0, climb_vertical_step_m_);
    climb_preview_waypoints_ = std::max(1, climb_preview_waypoints_);
    descent_release_distance_m_ = std::max(0.0, descent_release_distance_m_);
    failure_progress_stall_timeout_s_ = std::max(0.5, failure_progress_stall_timeout_s_);
    failure_stall_motion_radius_m_ = std::max(0.1, failure_stall_motion_radius_m_);
    failure_distance_improvement_m_ = std::max(0.1, failure_distance_improvement_m_);
    failure_min_distance_to_goal_m_ = std::max(final_acceptance_radius_m_, failure_min_distance_to_goal_m_);
    failure_zero_pwm_timeout_s_ = std::max(0.1, failure_zero_pwm_timeout_s_);
    failure_zero_pwm_threshold_ = Clamp(failure_zero_pwm_threshold_, 0.0, 1.0);
    failure_zero_pwm_min_progress_ = std::max(0, failure_zero_pwm_min_progress_);
    failure_grace_period_s_ = std::max(0.0, failure_grace_period_s_);
  }

  void LoadPath() {
    std::ifstream path_stream(path_csv_);
    if (!path_stream.is_open()) {
      ROS_FATAL_STREAM("Failed to open path csv: " << path_csv_);
      ros::shutdown();
      return;
    }

    std::string line;
    bool header_skipped = false;
    while (std::getline(path_stream, line)) {
      if (line.empty()) {
        continue;
      }

      if (!header_skipped) {
        header_skipped = true;
        continue;
      }

      const std::vector<std::string> fields = SplitCsvLine(line);
      if (fields.size() < 8) {
        continue;
      }

      Waypoint waypoint;
      waypoint.csv_index = std::stoi(fields[0]);
      waypoint.position_world_ned = Eigen::Vector3d(
          std::stod(fields[1]),
          std::stod(fields[2]),
          std::stod(fields[3]));
      waypoint.attitude_world_ned = Eigen::Quaterniond(
          std::stod(fields[7]),
          std::stod(fields[4]),
          std::stod(fields[5]),
          std::stod(fields[6]));
      waypoint.attitude_world_ned.normalize();
      waypoints_.push_back(waypoint);
    }

    if (waypoints_.size() < 2) {
      ROS_FATAL_STREAM("Path csv must contain at least 2 waypoints: " << path_csv_);
      ros::shutdown();
      return;
    }

    if (target_waypoint_index_ <= 0) {
      target_waypoint_index_ = static_cast<int>(waypoints_.size()) - 1;
    } else if (target_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
      target_waypoint_index_ = static_cast<int>(waypoints_.size()) - 1;
    }

    stable_point_indices_ = ParseIntegerCsv(stable_point_indices_csv_);
    stable_point_indices_.erase(
        std::remove_if(
            stable_point_indices_.begin(),
            stable_point_indices_.end(),
            [this](const int index) {
              return index <= 0
                  || index >= target_waypoint_index_
                  || index + 1 >= static_cast<int>(waypoints_.size());
            }),
        stable_point_indices_.end());
    std::sort(stable_point_indices_.begin(), stable_point_indices_.end());
    stable_point_indices_.erase(
        std::unique(stable_point_indices_.begin(), stable_point_indices_.end()),
        stable_point_indices_.end());
    if (stable_point_indices_.empty()) {
      stable_point_indices_csv_ = "none";
    }
  }

  void SetupRosInterfaces() {
    pose_subscriber_ = nh_.subscribe(pose_topic_, 20, &PathMissionNode::PoseCallback, this);
    rotor_pwm_subscriber_ = nh_.subscribe(rotor_pwm_topic_, 20, &PathMissionNode::RotorPwmCallback, this);
    pose_setpoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_setpoint_topic_, 10);
    enable_publisher_ = nh_.advertise<std_msgs::Bool>(enable_topic_, 1, true);
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_pose_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    current_position_world_ned_ = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    current_attitude_world_ned_ = ToEigenQuaternion(message->pose.orientation);
    has_pose_ = true;
  }

  void RotorPwmCallback(const airsim_ros::RotorPWM::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_mean_rotor_pwm_ = 0.25 * (
        message->rotorPWM0 +
        message->rotorPWM1 +
        message->rotorPWM2 +
        message->rotorPWM3);
    has_rotor_pwm_ = true;
  }

  void PublishTimerCallback(const ros::TimerEvent&) {
    SampledTarget target;
    ros::Time stamp;
    bool should_enable = false;
    bool should_abort = false;
    bool should_finish = false;
    std::string failure_reason;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!has_pose_) {
        ROS_WARN_THROTTLE(2.0, "Path mission is waiting for pose data.");
        return;
      }

      stamp = current_pose_stamp_.isZero() ? ros::Time::now() : current_pose_stamp_;
      RefreshStablePointStateLocked(stamp);
      AdvanceProgressLocked();
      RefreshStablePointStateLocked(stamp);
      target = SampleLookaheadTargetLocked();
      target.progress_index = current_waypoint_index_;

      if (enable_controller_on_start_ && !controller_enabled_published_) {
        controller_enabled_published_ = true;
        should_enable = true;
      }

      if (!mission_completed_logged_ && current_waypoint_index_ >= target_waypoint_index_
            && target.euclidean_distance_to_final <= final_acceptance_radius_m_) {
        mission_completed_logged_ = true;
        ROS_INFO_STREAM("Reached target waypoint " << target_waypoint_index_
                          << " with final_distance=" << target.euclidean_distance_to_final << " m");
      }

      should_finish = DetectMissionCompletionLocked(target, stamp);

      should_abort = DetectMissionFailureLocked(target, stamp, &failure_reason);
    }

    if (should_finish) {
      ROS_INFO_STREAM("Mission completed successfully after holding final acceptance radius for "
                      << completion_hold_time_s_ << " s");
      std::exit(kMissionSuccessExitCode);
    }

    if (should_abort) {
      ROS_ERROR_STREAM("Mission failure detected: " << failure_reason
                       << ", progress=" << target.progress_index << "/" << target_waypoint_index_
                       << ", lookahead=" << target.lookahead_index
                       << ", distance_to_final=" << target.distance_to_final);
      if (failure_abort_on_detection_) {
        std::exit(kMissionFailureExitCode);
      }
      return;
    }

    geometry_msgs::PoseStamped target_message;
    target_message.header.stamp = stamp;
    target_message.header.frame_id = world_frame_id_;
    target_message.pose.position.x = target.position_world_ned.x();
    target_message.pose.position.y = target.position_world_ned.y();
    target_message.pose.position.z = target.position_world_ned.z();
    target_message.pose.orientation = ToRosQuaternion(target.attitude_world_ned);
    pose_setpoint_publisher_.publish(target_message);

    if (should_enable) {
      std_msgs::Bool enable_message;
      enable_message.data = true;
      enable_publisher_.publish(enable_message);
    }

    ROS_INFO_THROTTLE(
        1.0,
      "Mission %s progress=%d/%d lookahead=%d remaining_path=%.2f final_distance=%.2f target=(%.2f, %.2f, %.2f)%s",
        target.pending_name.c_str(),
        target.progress_index,
        target_waypoint_index_,
        target.lookahead_index,
        target.distance_to_final,
        target.euclidean_distance_to_final,
        target.position_world_ned.x(),
        target.position_world_ned.y(),
        target.position_world_ned.z(),
        target.stable_hold_active ? " [stable-hold]" : "");
  }

  void AdvanceProgressLocked() {
    if (stable_point_hold_active_) {
      return;
    }

    while (current_waypoint_index_ < target_waypoint_index_) {
      const Waypoint& segment_start_waypoint = waypoints_[current_waypoint_index_];
      const Waypoint& next_waypoint = waypoints_[current_waypoint_index_ + 1];

      if ((current_position_world_ned_ - next_waypoint.position_world_ned).norm() <= waypoint_acceptance_radius_m_) {
        ++current_waypoint_index_;
        continue;
      }

      const Eigen::Vector3d segment =
          next_waypoint.position_world_ned - segment_start_waypoint.position_world_ned;
      const double segment_length_squared = segment.squaredNorm();
      if (segment_length_squared <= 1e-6) {
        ++current_waypoint_index_;
        continue;
      }

      const double raw_projection_fraction =
          (current_position_world_ned_ - segment_start_waypoint.position_world_ned).dot(segment)
          / segment_length_squared;
      if (raw_projection_fraction >= 1.0) {
        ++current_waypoint_index_;
        continue;
      }

      break;
    }
  }

  std::string PendingNameLocked(const int pending_index) const {
    return "pending" + std::to_string(std::max(0, pending_index) + 1);
  }

  Eigen::Quaterniond ComputeStablePointAttitudeLocked(const int stable_point_index) const {
    if (stable_point_index < 0 || stable_point_index + 1 >= static_cast<int>(waypoints_.size())) {
      return Eigen::Quaterniond::Identity();
    }

    const Eigen::Vector3d delta =
        waypoints_[stable_point_index + 1].position_world_ned - waypoints_[stable_point_index].position_world_ned;
    const Eigen::Vector2d horizontal_delta(delta.x(), delta.y());
    if (horizontal_delta.norm() <= 1e-6) {
      return waypoints_[stable_point_index].attitude_world_ned;
    }

    Eigen::Quaterniond attitude(
        Eigen::AngleAxisd(std::atan2(horizontal_delta.y(), horizontal_delta.x()), Eigen::Vector3d::UnitZ()));
    attitude.normalize();
    return attitude;
  }

  void RefreshStablePointStateLocked(const ros::Time& stamp) {
    if (stable_point_hold_active_) {
      if ((stamp - stable_point_hold_start_stamp_).toSec() < stable_point_hold_time_s_) {
        return;
      }

      stable_point_hold_active_ = false;
      stable_point_hold_start_stamp_ = ros::Time();
      active_stable_point_index_ = -1;
      ++active_pending_index_;
      ++next_stable_point_cursor_;
      ROS_INFO_STREAM("Stable point switch complete. Active phase="
                      << PendingNameLocked(active_pending_index_));
      return;
    }

    if (next_stable_point_cursor_ >= stable_point_indices_.size()) {
      return;
    }

    const int stable_point_index = stable_point_indices_[next_stable_point_cursor_];
    if (stable_point_index >= target_waypoint_index_ || current_waypoint_index_ < stable_point_index) {
      return;
    }

    active_stable_point_index_ = stable_point_index;
    stable_point_hold_active_ = true;
    stable_point_hold_start_stamp_ = stamp;
    stable_point_attitude_world_ned_ = ComputeStablePointAttitudeLocked(stable_point_index);
    ROS_INFO_STREAM("Reached stable point " << stable_point_index
                    << ". Holding for " << stable_point_hold_time_s_
                    << " s to switch from " << PendingNameLocked(active_pending_index_)
                    << " to " << PendingNameLocked(active_pending_index_ + 1));
  }

  SampledTarget SampleLookaheadTargetLocked() const {
    SampledTarget sampled_target;
    const Waypoint& final_waypoint = waypoints_[target_waypoint_index_];
    sampled_target.euclidean_distance_to_final =
        (current_position_world_ned_ - final_waypoint.position_world_ned).norm();
    sampled_target.progress_index = current_waypoint_index_;
    sampled_target.lookahead_index = std::min(current_waypoint_index_ + 1, target_waypoint_index_);
    sampled_target.pending_name = PendingNameLocked(active_pending_index_);

    if (stable_point_hold_active_ && active_stable_point_index_ >= 0) {
      sampled_target.position_world_ned = waypoints_[active_stable_point_index_].position_world_ned;
      sampled_target.attitude_world_ned = stable_point_attitude_world_ned_;
      sampled_target.lookahead_index = std::min(active_stable_point_index_ + 1, target_waypoint_index_);
      sampled_target.stable_hold_active = true;
      sampled_target.stable_point_index = active_stable_point_index_;
      return sampled_target;
    }

    int segment_index = std::min(current_waypoint_index_, target_waypoint_index_ - 1);
    double start_fraction = 0.0;
    while (segment_index < target_waypoint_index_) {
      const Waypoint& segment_start_waypoint = waypoints_[segment_index];
      const Waypoint& segment_end_waypoint = waypoints_[segment_index + 1];
      const Eigen::Vector3d segment =
          segment_end_waypoint.position_world_ned - segment_start_waypoint.position_world_ned;
      const double segment_length_squared = segment.squaredNorm();
      if (segment_length_squared <= 1e-6) {
        ++segment_index;
        continue;
      }

      start_fraction = Clamp(
          (current_position_world_ned_ - segment_start_waypoint.position_world_ned).dot(segment)
              / segment_length_squared,
          0.0,
          1.0);
      break;
    }

    if (segment_index >= target_waypoint_index_) {
      sampled_target.position_world_ned = final_waypoint.position_world_ned;
      sampled_target.attitude_world_ned = final_waypoint.attitude_world_ned;
      sampled_target.lookahead_index = target_waypoint_index_;
      return sampled_target;
    }

    const Waypoint& start_segment_waypoint = waypoints_[segment_index];
    const Waypoint& start_segment_end_waypoint = waypoints_[segment_index + 1];
    const Eigen::Vector3d start_segment =
      start_segment_end_waypoint.position_world_ned - start_segment_waypoint.position_world_ned;
    double remaining_path_distance = 0.0;
    int remaining_distance_segment_index = segment_index;
    double remaining_distance_fraction = start_fraction;
    while (remaining_distance_segment_index < target_waypoint_index_) {
      const Eigen::Vector3d remaining_segment =
          waypoints_[remaining_distance_segment_index + 1].position_world_ned
          - waypoints_[remaining_distance_segment_index].position_world_ned;
      const double remaining_segment_length = remaining_segment.norm();
      if (remaining_segment_length > 1e-6) {
        remaining_path_distance += (1.0 - remaining_distance_fraction) * remaining_segment_length;
      }
      ++remaining_distance_segment_index;
      remaining_distance_fraction = 0.0;
    }
    sampled_target.distance_to_final = remaining_path_distance;
    const Eigen::Vector3d projected_position_world_ned =
      start_segment_waypoint.position_world_ned + start_fraction * start_segment;
    const Eigen::Vector2d horizontal_tracking_error(
      current_position_world_ned_.x() - projected_position_world_ned.x(),
      current_position_world_ned_.y() - projected_position_world_ned.y());
    double effective_lookahead = std::max(
      min_lookahead_distance_m_,
      lookahead_distance_m_ - lookahead_error_reduction_gain_ * horizontal_tracking_error.norm());
    effective_lookahead *= ComputeTurnLookaheadScaleLocked(segment_index, start_fraction);
    effective_lookahead = Clamp(
        effective_lookahead,
        std::max(turn_min_lookahead_distance_m_, min_lookahead_distance_m_),
        lookahead_distance_m_);
    if (terminal_slowdown_distance_m_ > final_acceptance_radius_m_
      && sampled_target.distance_to_final < terminal_slowdown_distance_m_) {
      const double terminal_ratio = Clamp(
        (sampled_target.distance_to_final - final_acceptance_radius_m_)
          / std::max(terminal_slowdown_distance_m_ - final_acceptance_radius_m_, 1e-3),
        0.0,
        1.0);
      const double terminal_lookahead_cap = LinearInterpolate(
        terminal_lookahead_distance_m_,
        lookahead_distance_m_,
        terminal_ratio);
      effective_lookahead = std::min(effective_lookahead, terminal_lookahead_cap);
    }
    if (current_waypoint_index_ >= target_waypoint_index_) {
      effective_lookahead = std::min(effective_lookahead, terminal_lookahead_distance_m_);
    }

    double remaining_lookahead = effective_lookahead;
    int sampled_segment_index = segment_index;
    double sampled_fraction = start_fraction;
    while (sampled_segment_index < target_waypoint_index_) {
      const Waypoint& segment_start_waypoint = waypoints_[sampled_segment_index];
      const Waypoint& segment_end_waypoint = waypoints_[sampled_segment_index + 1];
      const Eigen::Vector3d segment =
          segment_end_waypoint.position_world_ned - segment_start_waypoint.position_world_ned;
      const double segment_length = segment.norm();
      if (segment_length <= 1e-6) {
        ++sampled_segment_index;
        sampled_fraction = 0.0;
        continue;
      }

      const double distance_available = (1.0 - sampled_fraction) * segment_length;
      if (remaining_lookahead <= distance_available || sampled_segment_index + 1 >= target_waypoint_index_) {
        const double target_fraction = Clamp(
            sampled_fraction + remaining_lookahead / segment_length,
            0.0,
            1.0);
        sampled_target.position_world_ned =
            segment_start_waypoint.position_world_ned + target_fraction * segment;
        sampled_target.attitude_world_ned = segment_start_waypoint.attitude_world_ned.slerp(
            target_fraction,
            segment_end_waypoint.attitude_world_ned);
        sampled_target.lookahead_index = std::min(sampled_segment_index + 1, target_waypoint_index_);

        const Eigen::Vector2d next_waypoint_horizontal_delta(
            segment_end_waypoint.position_world_ned.x() - current_position_world_ned_.x(),
            segment_end_waypoint.position_world_ned.y() - current_position_world_ned_.y());
        const double next_waypoint_horizontal_distance = next_waypoint_horizontal_delta.norm();
        if (descent_release_distance_m_ > 0.0 &&
            sampled_target.position_world_ned.z() > current_position_world_ned_.z() &&
            next_waypoint_horizontal_distance > descent_release_distance_m_) {
          sampled_target.position_world_ned.z() = current_position_world_ned_.z();
        }

        if (target_fraction >= 1.0 - 1e-3) {
          sampled_target.lookahead_index = std::min(sampled_segment_index + 1, target_waypoint_index_);
        }
        return sampled_target;
      }

      remaining_lookahead -= distance_available;
      ++sampled_segment_index;
      sampled_fraction = 0.0;
    }

    sampled_target.position_world_ned = final_waypoint.position_world_ned;
    sampled_target.attitude_world_ned = final_waypoint.attitude_world_ned;
    sampled_target.lookahead_index = target_waypoint_index_;
    return sampled_target;
  }

  double ComputeTurnLookaheadScaleLocked(
      const int segment_index,
      const double start_fraction) const {
    if (turn_angle_reduction_gain_ <= 0.0 || turn_lookahead_min_scale_ >= 1.0) {
      return 1.0;
    }
    if (segment_index < 0 || segment_index + 2 > target_waypoint_index_) {
      return 1.0;
    }

    const Eigen::Vector3d current_segment =
        waypoints_[segment_index + 1].position_world_ned - waypoints_[segment_index].position_world_ned;
    const Eigen::Vector3d next_segment =
        waypoints_[segment_index + 2].position_world_ned - waypoints_[segment_index + 1].position_world_ned;
    const double current_segment_horizontal_length = HorizontalNorm(current_segment);
    const double next_segment_horizontal_length = HorizontalNorm(next_segment);
    if (current_segment_horizontal_length <= 1e-6 || next_segment_horizontal_length <= 1e-6) {
      return 1.0;
    }
    if (turn_short_segment_ignore_distance_m_ > 0.0
        && (current_segment_horizontal_length < turn_short_segment_ignore_distance_m_
            || next_segment_horizontal_length < turn_short_segment_ignore_distance_m_)) {
      return 1.0;
    }

    const double turn_angle = HorizontalTurnAngleRad(current_segment, next_segment);
    if (turn_angle <= 1e-3) {
      return 1.0;
    }

    const double distance_to_corner = (1.0 - start_fraction) * current_segment_horizontal_length;
    if (distance_to_corner >= turn_preview_distance_m_) {
      return 1.0;
    }

    const double normalized_proximity = 1.0 - Clamp(
        distance_to_corner / std::max(turn_preview_distance_m_, 1e-3),
        0.0,
        1.0);
    const double reduction = turn_angle_reduction_gain_ * (turn_angle / kPi) * normalized_proximity;
    return Clamp(1.0 - reduction, turn_lookahead_min_scale_, 1.0);
  }

  bool DetectMissionCompletionLocked(
      const SampledTarget& target,
      const ros::Time& stamp) {
    if (mission_completion_reported_) {
      return false;
    }

    const bool inside_final_acceptance =
        current_waypoint_index_ >= target_waypoint_index_
      && target.euclidean_distance_to_final <= final_acceptance_radius_m_;
    if (!inside_final_acceptance) {
      mission_completion_start_stamp_ = ros::Time();
      return false;
    }

    if (mission_completion_start_stamp_.isZero()) {
      mission_completion_start_stamp_ = stamp;
    }

    if ((stamp - mission_completion_start_stamp_).toSec() < completion_hold_time_s_) {
      return false;
    }

    mission_completion_reported_ = true;
    return true;
  }

  bool DetectMissionFailureLocked(
      const SampledTarget& target,
      const ros::Time& stamp,
      std::string* reason) {
    if (mission_failed_) {
      if (reason != nullptr) {
        *reason = mission_failure_reason_;
      }
      return true;
    }

    if (!failure_monitor_initialized_) {
      failure_monitor_initialized_ = true;
      mission_start_stamp_ = stamp;
      last_progress_advance_stamp_ = stamp;
      last_distance_improvement_stamp_ = stamp;
      failure_anchor_position_world_ned_ = current_position_world_ned_;
      best_progress_index_ = current_waypoint_index_;
      best_distance_to_final_ = target.distance_to_final;
      return false;
    }

    if (current_waypoint_index_ > best_progress_index_) {
      best_progress_index_ = current_waypoint_index_;
      last_progress_advance_stamp_ = stamp;
      failure_anchor_position_world_ned_ = current_position_world_ned_;
    }

    if (target.distance_to_final + failure_distance_improvement_m_ < best_distance_to_final_) {
      best_distance_to_final_ = target.distance_to_final;
      last_distance_improvement_stamp_ = stamp;
      failure_anchor_position_world_ned_ = current_position_world_ned_;
    }

    if (current_waypoint_index_ >= target_waypoint_index_
        && target.euclidean_distance_to_final <= final_acceptance_radius_m_) {
      return false;
    }

    if ((stamp - mission_start_stamp_).toSec() < failure_grace_period_s_) {
      return false;
    }

    if (!mission_failed_
        && has_rotor_pwm_
        && current_waypoint_index_ >= failure_zero_pwm_min_progress_
        && target.distance_to_final > failure_min_distance_to_goal_m_) {
      if (latest_mean_rotor_pwm_ <= failure_zero_pwm_threshold_) {
        if (zero_pwm_start_stamp_.isZero()) {
          zero_pwm_start_stamp_ = stamp;
        } else if ((stamp - zero_pwm_start_stamp_).toSec() >= failure_zero_pwm_timeout_s_) {
          mission_failed_ = true;
          mission_failure_reason_ = "rotor pwm dropped near zero during active mission";
        }
      } else {
        zero_pwm_start_stamp_ = ros::Time();
      }
    } else {
      zero_pwm_start_stamp_ = ros::Time();
    }

    const double since_progress = (stamp - last_progress_advance_stamp_).toSec();
    const double since_distance_improvement = (stamp - last_distance_improvement_stamp_).toSec();
    if (!mission_failed_
        && current_waypoint_index_ < target_waypoint_index_
        && target.distance_to_final > failure_min_distance_to_goal_m_
        && since_progress >= failure_progress_stall_timeout_s_
        && since_distance_improvement >= failure_progress_stall_timeout_s_) {
      const double traveled_since_anchor =
          (current_position_world_ned_ - failure_anchor_position_world_ned_).norm();
      if (traveled_since_anchor <= failure_stall_motion_radius_m_) {
        mission_failed_ = true;
        mission_failure_reason_ = "mission stalled without progress";
      }
    }

    if (mission_failed_ && reason != nullptr) {
      *reason = mission_failure_reason_;
    }
    return mission_failed_;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber rotor_pwm_subscriber_;
  ros::Publisher pose_setpoint_publisher_;
  ros::Publisher enable_publisher_;
  ros::Timer publish_timer_;

  std::string path_csv_;
  std::string pose_topic_;
  std::string rotor_pwm_topic_;
  std::string pose_setpoint_topic_;
  std::string enable_topic_;
  std::string world_frame_id_;
  int target_waypoint_index_ = 69;
  double publish_rate_hz_ = 20.0;
  std::string stable_point_indices_csv_ = "69";
  double stable_point_hold_time_s_ = 3.0;
  double lookahead_distance_m_ = 30.0;
  double min_lookahead_distance_m_ = 0.0;
  double lookahead_error_reduction_gain_ = 0.0;
  double turn_preview_distance_m_ = 0.0;
  double turn_min_lookahead_distance_m_ = 0.0;
  double turn_lookahead_min_scale_ = 1.0;
  double turn_angle_reduction_gain_ = 0.0;
  double turn_short_segment_ignore_distance_m_ = 0.0;
  double terminal_lookahead_distance_m_ = 0.0;
  double terminal_slowdown_distance_m_ = 0.0;
  double waypoint_acceptance_radius_m_ = 8.0;
  double final_acceptance_radius_m_ = 4.0;
  double completion_hold_time_s_ = 0.0;
  double climb_priority_threshold_m_ = 1.0;
  double climb_horizontal_leash_m_ = 1.0;
  double climb_horizontal_leash_scale_ = 0.25;
  double climb_horizontal_leash_max_m_ = 8.0;
  double climb_vertical_release_distance_m_ = 10.0;
  double climb_vertical_step_m_ = 4.0;
  int climb_preview_waypoints_ = 1;
  double descent_release_distance_m_ = 0.0;
  bool enable_controller_on_start_ = true;
  bool failure_abort_on_detection_ = true;
  double failure_progress_stall_timeout_s_ = 3.0;
  double failure_stall_motion_radius_m_ = 1.5;
  double failure_distance_improvement_m_ = 1.5;
  double failure_min_distance_to_goal_m_ = 12.0;
  double failure_zero_pwm_timeout_s_ = 1.0;
  double failure_zero_pwm_threshold_ = 0.08;
  int failure_zero_pwm_min_progress_ = 8;
  double failure_grace_period_s_ = 8.0;

  mutable std::mutex data_mutex_;
  std::vector<Waypoint> waypoints_;
  std::vector<int> stable_point_indices_;
  bool has_pose_ = false;
  bool controller_enabled_published_ = false;
  bool mission_completed_logged_ = false;
  bool mission_completion_reported_ = false;
  bool failure_monitor_initialized_ = false;
  bool mission_failed_ = false;
  bool has_rotor_pwm_ = false;
  bool stable_point_hold_active_ = false;
  int current_waypoint_index_ = 0;
  int best_progress_index_ = 0;
  int active_pending_index_ = 0;
  int active_stable_point_index_ = -1;
  std::size_t next_stable_point_cursor_ = 0;
  ros::Time current_pose_stamp_;
  ros::Time mission_start_stamp_;
  ros::Time mission_completion_start_stamp_;
  ros::Time stable_point_hold_start_stamp_;
  ros::Time last_progress_advance_stamp_;
  ros::Time last_distance_improvement_stamp_;
  ros::Time zero_pwm_start_stamp_;
  Eigen::Vector3d current_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d failure_anchor_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond current_attitude_world_ned_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond stable_point_attitude_world_ned_ = Eigen::Quaterniond::Identity();
  double best_distance_to_final_ = std::numeric_limits<double>::infinity();
  double latest_mean_rotor_pwm_ = 0.0;
  std::string mission_failure_reason_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_mission_node");
  PathMissionNode mission_node;
  ros::spin();
  return 0;
}