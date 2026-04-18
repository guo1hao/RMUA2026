#include <airsim_ros/RotorPWM.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

namespace {

constexpr double kSqrtHalf = 0.7071067811865476;

double Clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(value, high));
}

double NormalizeAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double LinearInterpolate(const double start, const double end, const double ratio) {
  return start + (end - start) * Clamp(ratio, 0.0, 1.0);
}

double LimitLinearCommandRate(
    const double desired_value,
    const double previous_value,
    const double max_rate,
    const double dt) {
  if (max_rate <= 0.0 || dt <= 0.0) {
    return desired_value;
  }

  const double max_delta = max_rate * dt;
  return Clamp(desired_value, previous_value - max_delta, previous_value + max_delta);
}

double LimitYawCommandRate(
    const double desired_yaw,
    const double previous_yaw,
    const double max_rate,
    const double dt) {
  if (max_rate <= 0.0 || dt <= 0.0) {
    return NormalizeAngle(desired_yaw);
  }

  const double max_delta = max_rate * dt;
  const double yaw_delta = Clamp(NormalizeAngle(desired_yaw - previous_yaw), -max_delta, max_delta);
  return NormalizeAngle(previous_yaw + yaw_delta);
}

double ComputeProximityScale(
    const double distance,
    const double slowdown_distance,
    const double minimum_scale) {
  if (slowdown_distance <= 1e-6) {
    return 1.0;
  }

  const double normalized_distance = Clamp(distance / slowdown_distance, 0.0, 1.0);
  return LinearInterpolate(minimum_scale, 1.0, normalized_distance);
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

Eigen::Matrix3d NedToFluBasis() {
  Eigen::Matrix3d basis = Eigen::Matrix3d::Identity();
  basis(1, 1) = -1.0;
  basis(2, 2) = -1.0;
  return basis;
}

Eigen::Vector3d NedToFlu(const Eigen::Vector3d& vector_ned) {
  return Eigen::Vector3d(vector_ned.x(), -vector_ned.y(), -vector_ned.z());
}

Eigen::Vector3d FluToNed(const Eigen::Vector3d& vector_flu) {
  return Eigen::Vector3d(vector_flu.x(), -vector_flu.y(), -vector_flu.z());
}

Eigen::Quaterniond WorldNedToWorldFlu(const Eigen::Quaterniond& quaternion_world_ned) {
  const Eigen::Matrix3d basis = NedToFluBasis();
  const Eigen::Matrix3d rotation_world_flu = basis * quaternion_world_ned.toRotationMatrix() * basis;
  Eigen::Quaterniond quaternion_world_flu(rotation_world_flu);
  quaternion_world_flu.normalize();
  return quaternion_world_flu;
}

Eigen::Quaterniond WorldFluToWorldNed(const Eigen::Quaterniond& quaternion_world_flu) {
  const Eigen::Matrix3d basis = NedToFluBasis();
  const Eigen::Matrix3d rotation_world_ned = basis * quaternion_world_flu.toRotationMatrix() * basis;
  Eigen::Quaterniond quaternion_world_ned(rotation_world_ned);
  quaternion_world_ned.normalize();
  return quaternion_world_ned;
}

Eigen::Matrix4d MakeTransform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

struct RpyState {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

RpyState ExtractFluRpy(const Eigen::Matrix3d& rotation) {
  RpyState rpy;
  rpy.roll = std::asin(Clamp(rotation(2, 1), -1.0, 1.0));
  const double cos_roll = std::max(1e-6, std::cos(rpy.roll));
  rpy.pitch = std::atan2(-rotation(2, 0) / cos_roll, rotation(2, 2) / cos_roll);
  rpy.yaw = std::atan2(-rotation(0, 1) / cos_roll, rotation(1, 1) / cos_roll);
  return rpy;
}

struct ControllerSnapshot {
  bool has_pose = false;
  bool has_imu = false;
  bool has_home = false;
  bool has_target_setpoint = false;
  bool external_setpoint_active = false;
  bool controller_enabled = false;
  ros::Time pose_stamp;
  ros::Time imu_stamp;
  Eigen::Vector3d position_world_ned = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world_ned = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world_flu = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_world_ned = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity_body_ned = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_rel_flu = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_body_flu = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_rel_flu = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity_body_flu = Eigen::Vector3d::Zero();
  Eigen::Vector3d home_position_world_ned = Eigen::Vector3d::Zero();
  Eigen::Vector3d home_position_world_flu = Eigen::Vector3d::Zero();
  Eigen::Quaterniond home_attitude_world_flu = Eigen::Quaterniond::Identity();
  Eigen::Vector3d target_position_rel_flu = Eigen::Vector3d::Zero();
  double target_yaw_rel_flu = 0.0;
};

class BasicPwmControllerNode {
 public:
  BasicPwmControllerNode() : nh_(), pnh_("~") {
    LoadParameters();
    InitializeController();
    SetupRosInterfaces();

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate_hz_),
        &BasicPwmControllerNode::ControlLoop,
        this);

    ROS_INFO_STREAM(
        "Basic PWM controller ready. hover_pwm_estimate=" << hover_pwm_estimate_
        << ", max_force_per_motor=" << max_force_per_motor_
        << " N, control_rate_hz=" << control_rate_hz_);
  }

 private:
  void LoadParameters() {
    pnh_.param<std::string>("pose_topic", pose_topic_, "/rmua/sensors/drone_1/pose_gt");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/rmua/sensors/drone_1/imu");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/rmua/sensors/drone_1/initial_pose");
    pnh_.param<std::string>("pose_setpoint_topic", pose_setpoint_topic_, "/rmua/control/basic/pose_setpoint");
    pnh_.param<std::string>("enable_topic", enable_topic_, "/rmua/control/basic/enable");
    pnh_.param<std::string>("pwm_command_topic", pwm_command_topic_, "/airsim_node/drone_1/rotor_pwm_cmd");
    pnh_.param<std::string>("state_topic", state_topic_, "/rmua/control/basic/state");
    pnh_.param<std::string>("target_topic", target_topic_, "/rmua/control/basic/target_pose");
    pnh_.param<std::string>("pwm_debug_topic", pwm_debug_topic_, "/rmua/control/basic/pwm_cmd");
    pnh_.param<std::string>("world_frame_id", world_frame_id_, "world_ned");
    pnh_.param<std::string>("body_frame_id", body_frame_id_, "base_link_frd");

    pnh_.param("control_rate_hz", control_rate_hz_, 100.0);
    pnh_.param("velocity_filter_alpha", velocity_filter_alpha_, 0.25);
    pnh_.param("hover_altitude_m", hover_altitude_m_, 1.0);
    pnh_.param("auto_enable", auto_enable_, false);
    pnh_.param("publish_zero_when_inactive", publish_zero_when_inactive_, true);
    pnh_.param("use_initial_pose_as_home", use_initial_pose_as_home_, true);
    pnh_.param("home_from_first_pose_if_missing", home_from_first_pose_if_missing_, true);
    pnh_.param("initial_pose_home_distance_tolerance_m", initial_pose_home_distance_tolerance_m_, 2.0);
    pnh_.param("min_active_pwm", min_active_pwm_, 0.0);
    pnh_.param("hover_pwm_estimate_override", hover_pwm_estimate_override_, -1.0);

    pnh_.param("mass", mass_, 0.9);
    pnh_.param("gravity", gravity_, 9.8);
    pnh_.param("arm_length", arm_length_, 0.18);
    pnh_.param("ixx", ixx_, 0.0046890742);
    pnh_.param("iyy", iyy_, 0.0069312);
    pnh_.param("izz", izz_, 0.010421166);
    pnh_.param("thrust_coefficient", thrust_coefficient_, 0.00036771704516278653);
    pnh_.param("torque_coefficient", torque_coefficient_, 4.888486266072161e-06);
    pnh_.param("max_rotor_rpm", max_rotor_rpm_, 11079.03);

    pnh_.param("kp_x", kp_x_, 1.0);
    pnh_.param("kd_x", kd_x_, 0.1);
    pnh_.param("kp_y", kp_y_, 1.0);
    pnh_.param("kd_y", kd_y_, 0.1);
    pnh_.param("kp_z", kp_z_, 2.0);
    pnh_.param("ki_z", ki_z_, 0.0);
    pnh_.param("kv_z", kv_z_, 4.0);
    pnh_.param("z_integral_limit", z_integral_limit_, 0.0);
    pnh_.param("z_integral_deadband_m", z_integral_deadband_m_, 0.25);
    pnh_.param("z_integral_horizontal_error_limit_m", z_integral_horizontal_error_limit_m_, 0.0);
    pnh_.param("z_integral_unwind_rate", z_integral_unwind_rate_, 1.0);
    pnh_.param("vertical_priority_force_ratio", vertical_priority_force_ratio_, 1.0);
    pnh_.param("vertical_priority_error_z_m", vertical_priority_error_z_m_, 0.0);
    pnh_.param("vertical_priority_horizontal_error_m", vertical_priority_horizontal_error_m_, 0.0);
    pnh_.param("vertical_priority_horizontal_scale", vertical_priority_horizontal_scale_, 1.0);
    pnh_.param("kp_vx", kp_vx_, 0.5);
    pnh_.param("kd_vx", kd_vx_, 1.0);
    pnh_.param("kp_vy", kp_vy_, 0.5);
    pnh_.param("kd_vy", kd_vy_, 1.0);
    pnh_.param("kp_roll", kp_roll_, 17.0);
    pnh_.param("kd_roll", kd_roll_, 940.0);
    pnh_.param("kp_pitch", kp_pitch_, 20.0);
    pnh_.param("kd_pitch", kd_pitch_, 750.0);
    pnh_.param("kp_yaw", kp_yaw_, 5.0);
    pnh_.param("kd_yaw", kd_yaw_, 250.0);
    pnh_.param("roll_rate_damping", roll_rate_damping_, 0.0);
    pnh_.param("pitch_rate_damping", pitch_rate_damping_, 0.0);
    pnh_.param("yaw_rate_damping", yaw_rate_damping_, 0.0);
    pnh_.param("max_roll_pitch_command_rate_rad_s", max_roll_pitch_command_rate_rad_s_, 0.0);
    pnh_.param("max_yaw_command_rate_rad_s", max_yaw_command_rate_rad_s_, 0.0);
    pnh_.param("horizontal_velocity_slowdown_distance_m", horizontal_velocity_slowdown_distance_m_, 0.0);
    pnh_.param("horizontal_velocity_min_scale", horizontal_velocity_min_scale_, 1.0);
    pnh_.param("max_velocity", max_velocity_, 20.0);
    pnh_.param("max_acceleration", max_acceleration_, 20.0);
    pnh_.param("max_angle_rad", max_angle_rad_, 0.5);
    pnh_.param("tilt_compensation_min_cos", tilt_compensation_min_cos_, 0.6);
    pnh_.param("max_roll_pitch_torque", max_roll_pitch_torque_, 2.0);
    pnh_.param("max_yaw_torque", max_yaw_torque_, 1.0);

    control_rate_hz_ = std::max(20.0, control_rate_hz_);
    velocity_filter_alpha_ = Clamp(velocity_filter_alpha_, 0.0, 1.0);
    initial_pose_home_distance_tolerance_m_ = std::max(0.0, initial_pose_home_distance_tolerance_m_);
    min_active_pwm_ = Clamp(min_active_pwm_, 0.0, 1.0);
    max_velocity_ = std::max(0.5, max_velocity_);
    max_acceleration_ = std::max(0.5, max_acceleration_);
    max_angle_rad_ = Clamp(max_angle_rad_, 0.05, 1.2);
    tilt_compensation_min_cos_ = Clamp(tilt_compensation_min_cos_, 0.3, 1.0);
    z_integral_limit_ = std::max(0.0, z_integral_limit_);
    z_integral_deadband_m_ = std::max(0.0, z_integral_deadband_m_);
    z_integral_horizontal_error_limit_m_ = std::max(0.0, z_integral_horizontal_error_limit_m_);
    z_integral_unwind_rate_ = std::max(0.0, z_integral_unwind_rate_);
    vertical_priority_force_ratio_ = Clamp(vertical_priority_force_ratio_, 0.0, 1.0);
    vertical_priority_error_z_m_ = std::max(0.0, vertical_priority_error_z_m_);
    vertical_priority_horizontal_error_m_ = std::max(0.0, vertical_priority_horizontal_error_m_);
    vertical_priority_horizontal_scale_ = Clamp(vertical_priority_horizontal_scale_, 0.0, 1.0);
    roll_rate_damping_ = std::max(0.0, roll_rate_damping_);
    pitch_rate_damping_ = std::max(0.0, pitch_rate_damping_);
    yaw_rate_damping_ = std::max(0.0, yaw_rate_damping_);
    max_roll_pitch_command_rate_rad_s_ = std::max(0.0, max_roll_pitch_command_rate_rad_s_);
    max_yaw_command_rate_rad_s_ = std::max(0.0, max_yaw_command_rate_rad_s_);
    horizontal_velocity_slowdown_distance_m_ = std::max(0.0, horizontal_velocity_slowdown_distance_m_);
    horizontal_velocity_min_scale_ = Clamp(horizontal_velocity_min_scale_, 0.05, 1.0);
  }

  void InitializeController() {
    inertia_.setZero();
    inertia_(0, 0) = ixx_;
    inertia_(1, 1) = iyy_;
    inertia_(2, 2) = izz_;

    mixer_matrix_ <<
        1.0, 1.0, 1.0, 1.0,
       -1.0, 1.0, 1.0, -1.0,
       -1.0, 1.0, -1.0, 1.0,
       -1.0, -1.0, 1.0, 1.0;
    mixer_matrix_inverse_ = mixer_matrix_.inverse();

    const double max_rotor_rps = max_rotor_rpm_ / 60.0;
    max_force_per_motor_ = thrust_coefficient_ * max_rotor_rps * max_rotor_rps;
    max_total_force_ = 4.0 * max_force_per_motor_;
    hover_pwm_estimate_ = Clamp((mass_ * gravity_) / max_total_force_, 0.0, 1.0);

    if (hover_pwm_estimate_override_ > 1e-3) {
      hover_pwm_estimate_ = Clamp(hover_pwm_estimate_override_, 1e-3, 1.0);
      max_total_force_ = (mass_ * gravity_) / hover_pwm_estimate_;
      max_force_per_motor_ = max_total_force_ / 4.0;
    }
  }

  void SetupRosInterfaces() {
    pose_subscriber_ = nh_.subscribe(pose_topic_, 20, &BasicPwmControllerNode::PoseCallback, this);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 100, &BasicPwmControllerNode::ImuCallback, this);
    initial_pose_subscriber_ = nh_.subscribe(initial_pose_topic_, 2, &BasicPwmControllerNode::InitialPoseCallback, this);
    pose_setpoint_subscriber_ = nh_.subscribe(pose_setpoint_topic_, 10, &BasicPwmControllerNode::PoseSetpointCallback, this);
    enable_subscriber_ = nh_.subscribe(enable_topic_, 5, &BasicPwmControllerNode::EnableCallback, this);

    pwm_publisher_ = nh_.advertise<airsim_ros::RotorPWM>(pwm_command_topic_, 1);
    state_publisher_ = nh_.advertise<nav_msgs::Odometry>(state_topic_, 10);
    target_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(target_topic_, 10);
    pwm_debug_publisher_ = nh_.advertise<airsim_ros::RotorPWM>(pwm_debug_topic_, 10);
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    const ros::Time stamp = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    const Eigen::Vector3d position_world_ned(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    const Eigen::Quaterniond attitude_world_ned = ToEigenQuaternion(message->pose.orientation);

    if (has_pose_) {
      const double dt = (stamp - pose_stamp_).toSec();
      if (dt > 1e-4) {
        const Eigen::Vector3d raw_velocity_world_ned =
            (position_world_ned - position_world_ned_) / dt;
        velocity_world_ned_ = velocity_filter_alpha_ * raw_velocity_world_ned
            + (1.0 - velocity_filter_alpha_) * velocity_world_ned_;
      }
    }

    position_world_ned_ = position_world_ned;
    attitude_world_ned_ = attitude_world_ned;
    pose_stamp_ = stamp;
    has_pose_ = true;

    MaybeInitializeHomeLocked();
    UpdateRelativeStateLocked();
  }

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    imu_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    angular_velocity_body_ned_ = Eigen::Vector3d(
        message->angular_velocity.x,
        message->angular_velocity.y,
        message->angular_velocity.z);
    angular_velocity_body_flu_ = NedToFlu(angular_velocity_body_ned_);
    has_imu_ = true;
  }

  void InitialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!use_initial_pose_as_home_) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    initial_pose_reference_position_world_ned_ = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    initial_pose_reference_attitude_world_ned_ = ToEigenQuaternion(message->pose.orientation);
    has_initial_pose_reference_ = true;

    MaybeInitializeHomeLocked();
    UpdateRelativeStateLocked();
  }

  void PoseSetpointCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    pending_target_position_world_ned_ = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    pending_target_attitude_world_ned_ = ToEigenQuaternion(message->pose.orientation);
    has_pending_target_world_ned_ = true;
    external_setpoint_active_ = true;

    if (has_home_) {
      ApplyPendingTargetLocked();
    }
  }

  void EnableCallback(const std_msgs::Bool::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    controller_enabled_ = message->data;

    if (!controller_enabled_ && !auto_enable_) {
      ResetControllerStateLocked();
      if (!external_setpoint_active_) {
        has_target_setpoint_ = false;
      }
    }
  }

  void MaybeInitializeHomeLocked() {
    if (has_home_) {
      return;
    }

    if (use_initial_pose_as_home_ && has_initial_pose_reference_) {
      if (!has_pose_ || !home_from_first_pose_if_missing_) {
        SetHomeLocked(
            initial_pose_reference_position_world_ned_,
            initial_pose_reference_attitude_world_ned_,
            "initial_pose");
        return;
      }

      const double initial_pose_offset =
          (position_world_ned_ - initial_pose_reference_position_world_ned_).norm();
      if (initial_pose_offset <= initial_pose_home_distance_tolerance_m_) {
        SetHomeLocked(
            initial_pose_reference_position_world_ned_,
            initial_pose_reference_attitude_world_ned_,
            "initial_pose");
        ROS_INFO_STREAM("Using initial_pose as home. offset_to_pose_gt=" << initial_pose_offset << " m");
        return;
      }

      ROS_WARN_STREAM("initial_pose differs from pose_gt by " << initial_pose_offset
                      << " m; falling back to first pose_gt as home.");
    }

    if (home_from_first_pose_if_missing_ && has_pose_) {
      SetHomeLocked(position_world_ned_, attitude_world_ned_, "first pose_gt");
    }
  }

  void SetHomeLocked(
      const Eigen::Vector3d& position_world_ned,
      const Eigen::Quaterniond& attitude_world_ned,
      const std::string& source) {
    home_position_world_ned_ = position_world_ned;
    home_position_world_flu_ = NedToFlu(position_world_ned);
    home_attitude_world_flu_ = WorldNedToWorldFlu(attitude_world_ned);
    has_home_ = true;

    if (has_pending_target_world_ned_) {
      ApplyPendingTargetLocked();
    }

    ROS_INFO_STREAM("Home initialized from " << source << " at ["
                    << position_world_ned.x() << ", "
                    << position_world_ned.y() << ", "
                    << position_world_ned.z() << "]");
  }

  void UpdateRelativeStateLocked() {
    if (!has_pose_ || !has_home_) {
      return;
    }

    const Eigen::Quaterniond attitude_world_flu = WorldNedToWorldFlu(attitude_world_ned_);

    position_rel_flu_ = NedToFlu(position_world_ned_ - home_position_world_ned_);
    attitude_rel_flu_ = home_attitude_world_flu_.inverse() * attitude_world_flu;
    attitude_rel_flu_.normalize();

    const Eigen::Vector3d velocity_world_flu = NedToFlu(velocity_world_ned_);
    velocity_body_flu_ = attitude_world_flu.toRotationMatrix().transpose() * velocity_world_flu;
    angular_velocity_body_flu_ = NedToFlu(angular_velocity_body_ned_);
  }

  void ApplyPendingTargetLocked() {
    if (!has_pending_target_world_ned_ || !has_home_) {
      return;
    }

    const Eigen::Quaterniond target_attitude_world_flu = WorldNedToWorldFlu(pending_target_attitude_world_ned_);

    target_position_rel_flu_ = NedToFlu(pending_target_position_world_ned_ - home_position_world_ned_);
    const Eigen::Quaterniond target_attitude_rel_flu = home_attitude_world_flu_.inverse() * target_attitude_world_flu;
    const RpyState target_rpy = ExtractFluRpy(target_attitude_rel_flu.toRotationMatrix());
    target_yaw_rel_flu_ = target_rpy.yaw;
    has_target_setpoint_ = true;
  }

  void SetDefaultHoverTargetLocked() {
    if (!has_home_ || !has_pose_ || external_setpoint_active_) {
      return;
    }

    const RpyState current_rpy = ExtractFluRpy(attitude_rel_flu_.toRotationMatrix());
    target_position_rel_flu_ = position_rel_flu_ + Eigen::Vector3d(0.0, 0.0, hover_altitude_m_);
    target_yaw_rel_flu_ = current_rpy.yaw;
    has_target_setpoint_ = true;

    ROS_INFO_STREAM("Initialized default hover target in relative FLU at ["
                    << target_position_rel_flu_.x() << ", "
                    << target_position_rel_flu_.y() << ", "
                    << target_position_rel_flu_.z() << "] yaw="
                    << target_yaw_rel_flu_);
  }

  void ResetControllerStateLocked() {
    error_state_initialized_ = false;
    error_z_integral_ = 0.0;
    desired_command_state_initialized_ = false;
  }

  ControllerSnapshot Snapshot() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ControllerSnapshot snapshot;
    snapshot.has_pose = has_pose_;
    snapshot.has_imu = has_imu_;
    snapshot.has_home = has_home_;
    snapshot.has_target_setpoint = has_target_setpoint_;
    snapshot.external_setpoint_active = external_setpoint_active_;
    snapshot.controller_enabled = controller_enabled_;
    snapshot.pose_stamp = pose_stamp_;
    snapshot.imu_stamp = imu_stamp_;
    snapshot.position_world_ned = position_world_ned_;
    snapshot.velocity_world_ned = velocity_world_ned_;
    snapshot.velocity_world_flu = NedToFlu(velocity_world_ned_);
    snapshot.attitude_world_ned = attitude_world_ned_;
    snapshot.angular_velocity_body_ned = angular_velocity_body_ned_;
    snapshot.position_rel_flu = position_rel_flu_;
    snapshot.velocity_body_flu = velocity_body_flu_;
    snapshot.attitude_rel_flu = attitude_rel_flu_;
    snapshot.angular_velocity_body_flu = angular_velocity_body_flu_;
    snapshot.home_position_world_ned = home_position_world_ned_;
    snapshot.home_position_world_flu = home_position_world_flu_;
    snapshot.home_attitude_world_flu = home_attitude_world_flu_;
    snapshot.target_position_rel_flu = target_position_rel_flu_;
    snapshot.target_yaw_rel_flu = target_yaw_rel_flu_;
    return snapshot;
  }

  airsim_ros::RotorPWM ZeroPwmMessage(const ros::Time& stamp) const {
    airsim_ros::RotorPWM pwm_message;
    pwm_message.header.stamp = stamp;
    pwm_message.rotorPWM0 = 0.0;
    pwm_message.rotorPWM1 = 0.0;
    pwm_message.rotorPWM2 = 0.0;
    pwm_message.rotorPWM3 = 0.0;
    return pwm_message;
  }

  void PublishZeroPwm(const ros::Time& stamp) {
    const airsim_ros::RotorPWM pwm_message = ZeroPwmMessage(stamp);
    pwm_publisher_.publish(pwm_message);
    pwm_debug_publisher_.publish(pwm_message);
  }

  void PublishState(const ControllerSnapshot& snapshot) {
    nav_msgs::Odometry state_message;
    state_message.header.stamp = snapshot.pose_stamp.isZero() ? ros::Time::now() : snapshot.pose_stamp;
    state_message.header.frame_id = world_frame_id_;
    state_message.child_frame_id = body_frame_id_;
    state_message.pose.pose.position.x = snapshot.position_world_ned.x();
    state_message.pose.pose.position.y = snapshot.position_world_ned.y();
    state_message.pose.pose.position.z = snapshot.position_world_ned.z();
    state_message.pose.pose.orientation = ToRosQuaternion(snapshot.attitude_world_ned);
    state_message.twist.twist.linear.x = snapshot.velocity_world_ned.x();
    state_message.twist.twist.linear.y = snapshot.velocity_world_ned.y();
    state_message.twist.twist.linear.z = snapshot.velocity_world_ned.z();
    state_message.twist.twist.angular.x = snapshot.angular_velocity_body_ned.x();
    state_message.twist.twist.angular.y = snapshot.angular_velocity_body_ned.y();
    state_message.twist.twist.angular.z = snapshot.angular_velocity_body_ned.z();
    state_publisher_.publish(state_message);
  }

  void PublishTarget(const ControllerSnapshot& snapshot, const ros::Time& stamp) {
    if (!snapshot.has_home || !snapshot.has_target_setpoint) {
      return;
    }

    const Eigen::Quaterniond relative_target_attitude(
        Eigen::AngleAxisd(snapshot.target_yaw_rel_flu, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d target_position_world_ned =
      snapshot.home_position_world_ned + FluToNed(snapshot.target_position_rel_flu);
    const Eigen::Quaterniond target_attitude_world_flu =
      snapshot.home_attitude_world_flu * relative_target_attitude;
    Eigen::Quaterniond target_attitude_world_ned(
      WorldFluToWorldNed(target_attitude_world_flu));
    target_attitude_world_ned.normalize();

    geometry_msgs::PoseStamped target_message;
    target_message.header.stamp = stamp;
    target_message.header.frame_id = world_frame_id_;
    target_message.pose.position.x = target_position_world_ned.x();
    target_message.pose.position.y = target_position_world_ned.y();
    target_message.pose.position.z = target_position_world_ned.z();
    target_message.pose.orientation = ToRosQuaternion(target_attitude_world_ned);
    target_publisher_.publish(target_message);
  }

  airsim_ros::RotorPWM ComputePwmCommand(
      const ControllerSnapshot& snapshot,
      const ros::Time& stamp,
      const double control_dt,
      double* collective_force_out) {
    airsim_ros::RotorPWM pwm_message = ZeroPwmMessage(stamp);

    const RpyState attitude = ExtractFluRpy(snapshot.attitude_rel_flu.toRotationMatrix());

    const double error_x = snapshot.target_position_rel_flu.x() - snapshot.position_rel_flu.x();
    const double error_y = snapshot.target_position_rel_flu.y() - snapshot.position_rel_flu.y();
    const double error_z = snapshot.target_position_rel_flu.z() - snapshot.position_rel_flu.z();
    const double horizontal_error = std::hypot(error_x, error_y);

    if (!error_state_initialized_) {
      last_error_x_ = error_x;
      last_error_y_ = error_y;
      last_error_z_ = error_z;
      error_z_integral_ = 0.0;
      last_error_vx_ = 0.0;
      last_error_vy_ = 0.0;
      last_error_roll_ = 0.0;
      last_error_pitch_ = 0.0;
      last_error_yaw_ = 0.0;
      error_state_initialized_ = true;
    }

    if (error_z_integral_ * error_z < 0.0) {
      error_z_integral_ = 0.0;
    }

    const double unwind_alpha = Clamp(z_integral_unwind_rate_ * control_dt, 0.0, 1.0);
    if (horizontal_error > z_integral_horizontal_error_limit_m_) {
      error_z_integral_ *= (1.0 - unwind_alpha);
    } else if (std::abs(error_z) < z_integral_deadband_m_) {
      error_z_integral_ *= (1.0 - unwind_alpha);
    } else {
      error_z_integral_ += error_z * control_dt;
      error_z_integral_ = Clamp(error_z_integral_, -z_integral_limit_, z_integral_limit_);
    }

    const double vertical_acceleration_command =
        kp_z_ * error_z + ki_z_ * error_z_integral_ - kv_z_ * snapshot.velocity_world_flu.z();
    double collective_force = mass_ * gravity_ + mass_ * vertical_acceleration_command;
    collective_force = Clamp(collective_force, 0.0, max_total_force_);

    const double horizontal_velocity_scale = ComputeProximityScale(
      horizontal_error,
      horizontal_velocity_slowdown_distance_m_,
      horizontal_velocity_min_scale_);
    const double scaled_max_velocity = std::max(0.2, max_velocity_ * horizontal_velocity_scale);
    const double desired_vx = Clamp(
      horizontal_velocity_scale * (kp_x_ * error_x + kd_x_ * (error_x - last_error_x_)),
      -scaled_max_velocity,
      scaled_max_velocity);
    const double desired_vy = Clamp(
      horizontal_velocity_scale * (kp_y_ * error_y + kd_y_ * (error_y - last_error_y_)),
      -scaled_max_velocity,
      scaled_max_velocity);

    const double error_vx = desired_vx - snapshot.velocity_world_flu.x();
    const double error_vy = desired_vy - snapshot.velocity_world_flu.y();

    double desired_ax = Clamp(kp_vx_ * error_vx + kd_vx_ * (error_vx - last_error_vx_), -max_acceleration_, max_acceleration_);
    double desired_ay = Clamp(kp_vy_ * error_vy + kd_vy_ * (error_vy - last_error_vy_), -max_acceleration_, max_acceleration_);

    const bool prioritize_vertical_climb =
        collective_force >= vertical_priority_force_ratio_ * max_total_force_
        && error_z >= vertical_priority_error_z_m_
        && horizontal_error <= vertical_priority_horizontal_error_m_;
    if (prioritize_vertical_climb) {
      desired_ax *= vertical_priority_horizontal_scale_;
      desired_ay *= vertical_priority_horizontal_scale_;
    }

    const double safe_collective_force = std::max(collective_force, 1e-3);
    const double raw_desired_roll = Clamp(
        (mass_ / safe_collective_force)
        * (desired_ax * std::sin(attitude.yaw) - desired_ay * std::cos(attitude.yaw)),
        -max_angle_rad_,
        max_angle_rad_);
    const double raw_desired_pitch = Clamp(
      (mass_ / safe_collective_force)
      * (desired_ax * std::cos(attitude.yaw) + desired_ay * std::sin(attitude.yaw)),
        -max_angle_rad_,
        max_angle_rad_);

    if (!desired_command_state_initialized_) {
      last_desired_roll_command_ = attitude.roll;
      last_desired_pitch_command_ = attitude.pitch;
      last_desired_yaw_command_ = attitude.yaw;
      desired_command_state_initialized_ = true;
    }

    const double desired_roll = LimitLinearCommandRate(
        raw_desired_roll,
        last_desired_roll_command_,
        max_roll_pitch_command_rate_rad_s_,
        control_dt);
    const double desired_pitch = LimitLinearCommandRate(
        raw_desired_pitch,
        last_desired_pitch_command_,
        max_roll_pitch_command_rate_rad_s_,
        control_dt);
    const double desired_yaw = LimitYawCommandRate(
        snapshot.target_yaw_rel_flu,
        last_desired_yaw_command_,
        max_yaw_command_rate_rad_s_,
        control_dt);

      const double tilt_cos = std::max(
        tilt_compensation_min_cos_,
        std::cos(desired_roll) * std::cos(desired_pitch));
      collective_force = Clamp(collective_force / tilt_cos, 0.0, max_total_force_);

    const double error_roll = desired_roll - attitude.roll;
    const double error_pitch = desired_pitch - attitude.pitch;
    const double error_yaw = NormalizeAngle(desired_yaw - attitude.yaw);

    double torque_x = ixx_ * (
        kp_roll_ * error_roll
        + kd_roll_ * (error_roll - last_error_roll_)
        - roll_rate_damping_ * snapshot.angular_velocity_body_flu.x());
    double torque_y = iyy_ * (
        kp_pitch_ * error_pitch
        + kd_pitch_ * (error_pitch - last_error_pitch_)
        - pitch_rate_damping_ * snapshot.angular_velocity_body_flu.y());
    double torque_z = izz_ * (
        kp_yaw_ * error_yaw
        + kd_yaw_ * (error_yaw - last_error_yaw_)
        - yaw_rate_damping_ * snapshot.angular_velocity_body_flu.z());

    torque_x = Clamp(torque_x, -max_roll_pitch_torque_, max_roll_pitch_torque_);
    torque_y = Clamp(torque_y, -max_roll_pitch_torque_, max_roll_pitch_torque_);
    torque_z = Clamp(torque_z, -max_yaw_torque_, max_yaw_torque_);

    Eigen::Vector4d input;
    input << collective_force / thrust_coefficient_,
        torque_x / (arm_length_ * thrust_coefficient_ * kSqrtHalf),
        torque_y / (arm_length_ * thrust_coefficient_ * kSqrtHalf),
        torque_z / torque_coefficient_;

    Eigen::Vector4d motor_forces = thrust_coefficient_ * mixer_matrix_inverse_ * input;
    Eigen::Array4d pwm = motor_forces.array() / std::max(max_force_per_motor_, 1e-6);

    for (int index = 0; index < 4; ++index) {
      pwm(index) = std::max(0.0, pwm(index));
    }

    const double max_pwm = pwm.maxCoeff();
    if (max_pwm > 1.0) {
      pwm /= max_pwm;
    }

    for (int index = 0; index < 4; ++index) {
      pwm(index) = Clamp(pwm(index), min_active_pwm_, 1.0);
    }

    pwm_message.rotorPWM0 = pwm(0);
    pwm_message.rotorPWM1 = pwm(1);
    pwm_message.rotorPWM2 = pwm(2);
    pwm_message.rotorPWM3 = pwm(3);

    last_error_x_ = error_x;
    last_error_y_ = error_y;
    last_error_vx_ = error_vx;
    last_error_vy_ = error_vy;
    last_error_roll_ = error_roll;
    last_error_pitch_ = error_pitch;
    last_error_yaw_ = error_yaw;
    last_desired_roll_command_ = desired_roll;
    last_desired_pitch_command_ = desired_pitch;
    last_desired_yaw_command_ = desired_yaw;

    *collective_force_out = collective_force;
    return pwm_message;
  }

  void ControlLoop(const ros::TimerEvent&) {
    const ControllerSnapshot snapshot = Snapshot();
    const ros::Time stamp = snapshot.imu_stamp.isZero() ? ros::Time::now() : snapshot.imu_stamp;
    const double control_dt = 1.0 / control_rate_hz_;

    PublishState(snapshot);

    if (!snapshot.has_pose || !snapshot.has_imu || !snapshot.has_home) {
      if (publish_zero_when_inactive_) {
        PublishZeroPwm(stamp);
      }
      ROS_WARN_THROTTLE(2.0, "Basic PWM controller is waiting for pose, imu, and home initialization.");
      return;
    }

    const bool active = auto_enable_ || snapshot.controller_enabled;
    if (!active) {
      if (publish_zero_when_inactive_) {
        PublishZeroPwm(stamp);
      }
      ROS_INFO_THROTTLE(2.0, "Basic PWM controller is inactive. Publish std_msgs/Bool true to the enable topic or set auto_enable=true.");
      return;
    }

    if (!snapshot.has_target_setpoint && !snapshot.external_setpoint_active) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      SetDefaultHoverTargetLocked();
    }

    const ControllerSnapshot updated_snapshot = Snapshot();
    if (!updated_snapshot.has_target_setpoint) {
      if (publish_zero_when_inactive_) {
        PublishZeroPwm(stamp);
      }
      ROS_WARN_THROTTLE(2.0, "Basic PWM controller has no target setpoint.");
      return;
    }

    PublishTarget(updated_snapshot, stamp);

    double collective_force = 0.0;
  airsim_ros::RotorPWM pwm_message = ComputePwmCommand(updated_snapshot, stamp, control_dt, &collective_force);
    pwm_publisher_.publish(pwm_message);
    pwm_debug_publisher_.publish(pwm_message);

    ROS_INFO_THROTTLE(
        1.0,
    "PWM ctrl rel_target=(%.2f, %.2f, %.2f, yaw=%.2f) rel_pos=(%.2f, %.2f, %.2f) F=%.2f iz=%.2f pwm=[%.3f %.3f %.3f %.3f]",
        updated_snapshot.target_position_rel_flu.x(),
        updated_snapshot.target_position_rel_flu.y(),
        updated_snapshot.target_position_rel_flu.z(),
        updated_snapshot.target_yaw_rel_flu,
        updated_snapshot.position_rel_flu.x(),
        updated_snapshot.position_rel_flu.y(),
        updated_snapshot.position_rel_flu.z(),
        collective_force,
    error_z_integral_,
        pwm_message.rotorPWM0,
        pwm_message.rotorPWM1,
        pwm_message.rotorPWM2,
        pwm_message.rotorPWM3);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber initial_pose_subscriber_;
  ros::Subscriber pose_setpoint_subscriber_;
  ros::Subscriber enable_subscriber_;
  ros::Publisher pwm_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher target_publisher_;
  ros::Publisher pwm_debug_publisher_;
  ros::Timer control_timer_;

  std::string pose_topic_;
  std::string imu_topic_;
  std::string initial_pose_topic_;
  std::string pose_setpoint_topic_;
  std::string enable_topic_;
  std::string pwm_command_topic_;
  std::string state_topic_;
  std::string target_topic_;
  std::string pwm_debug_topic_;
  std::string world_frame_id_;
  std::string body_frame_id_;

  double control_rate_hz_ = 100.0;
  double velocity_filter_alpha_ = 0.25;
  double hover_altitude_m_ = 1.0;
  double hover_pwm_estimate_override_ = -1.0;
  bool auto_enable_ = false;
  bool publish_zero_when_inactive_ = true;
  bool use_initial_pose_as_home_ = true;
  bool home_from_first_pose_if_missing_ = true;
  double initial_pose_home_distance_tolerance_m_ = 2.0;
  double min_active_pwm_ = 0.0;

  double mass_ = 0.9;
  double gravity_ = 9.8;
  double arm_length_ = 0.18;
  double ixx_ = 0.0046890742;
  double iyy_ = 0.0069312;
  double izz_ = 0.010421166;
  double thrust_coefficient_ = 0.00036771704516278653;
  double torque_coefficient_ = 4.888486266072161e-06;
  double max_rotor_rpm_ = 11079.03;

  double kp_x_ = 1.0;
  double kd_x_ = 0.1;
  double kp_y_ = 1.0;
  double kd_y_ = 0.1;
  double kp_z_ = 2.0;
  double ki_z_ = 0.0;
  double kv_z_ = 4.0;
  double z_integral_limit_ = 0.0;
  double z_integral_deadband_m_ = 0.25;
  double z_integral_horizontal_error_limit_m_ = 0.0;
  double z_integral_unwind_rate_ = 1.0;
  double vertical_priority_force_ratio_ = 1.0;
  double vertical_priority_error_z_m_ = 0.0;
  double vertical_priority_horizontal_error_m_ = 0.0;
  double vertical_priority_horizontal_scale_ = 1.0;
  double kp_vx_ = 0.5;
  double kd_vx_ = 1.0;
  double kp_vy_ = 0.5;
  double kd_vy_ = 1.0;
  double kp_roll_ = 17.0;
  double kd_roll_ = 940.0;
  double kp_pitch_ = 20.0;
  double kd_pitch_ = 750.0;
  double kp_yaw_ = 5.0;
  double kd_yaw_ = 250.0;
  double roll_rate_damping_ = 0.0;
  double pitch_rate_damping_ = 0.0;
  double yaw_rate_damping_ = 0.0;
  double max_roll_pitch_command_rate_rad_s_ = 0.0;
  double max_yaw_command_rate_rad_s_ = 0.0;
  double horizontal_velocity_slowdown_distance_m_ = 0.0;
  double horizontal_velocity_min_scale_ = 1.0;
  double max_velocity_ = 20.0;
  double max_acceleration_ = 20.0;
  double max_angle_rad_ = 0.5;
  double tilt_compensation_min_cos_ = 0.6;
  double max_roll_pitch_torque_ = 2.0;
  double max_yaw_torque_ = 1.0;

  Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d mixer_matrix_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d mixer_matrix_inverse_ = Eigen::Matrix4d::Identity();
  double max_force_per_motor_ = 0.0;
  double max_total_force_ = 0.0;
  double hover_pwm_estimate_ = 0.0;

  mutable std::mutex data_mutex_;
  bool has_pose_ = false;
  bool has_imu_ = false;
  bool has_home_ = false;
  bool has_target_setpoint_ = false;
  bool has_initial_pose_reference_ = false;
  bool has_pending_target_world_ned_ = false;
  bool external_setpoint_active_ = false;
  bool controller_enabled_ = false;
  bool error_state_initialized_ = false;
  bool desired_command_state_initialized_ = false;
  ros::Time pose_stamp_;
  ros::Time imu_stamp_;
  Eigen::Vector3d position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_world_ned_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity_body_ned_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_rel_flu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_body_flu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude_rel_flu_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity_body_flu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d home_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d home_position_world_flu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond home_attitude_world_flu_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d initial_pose_reference_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond initial_pose_reference_attitude_world_ned_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d pending_target_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond pending_target_attitude_world_ned_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d target_position_rel_flu_ = Eigen::Vector3d::Zero();
  double target_yaw_rel_flu_ = 0.0;
  double last_error_x_ = 0.0;
  double last_error_y_ = 0.0;
  double last_error_z_ = 0.0;
  double error_z_integral_ = 0.0;
  double last_error_vx_ = 0.0;
  double last_error_vy_ = 0.0;
  double last_error_roll_ = 0.0;
  double last_error_pitch_ = 0.0;
  double last_error_yaw_ = 0.0;
  double last_desired_roll_command_ = 0.0;
  double last_desired_pitch_command_ = 0.0;
  double last_desired_yaw_command_ = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_pwm_controller_node");
  BasicPwmControllerNode controller_node;
  ros::spin();
  return 0;
}