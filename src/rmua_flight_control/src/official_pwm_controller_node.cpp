#include <airsim_ros/RotorPWM.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include "rmua_flight_control/official_pd_controller.hpp"

namespace {

double Clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(value, high));
}

Eigen::Quaterniond ToEigenQuaternion(const geometry_msgs::Quaternion& quaternion_msg) {
  Eigen::Quaterniond quaternion(
      quaternion_msg.w,
      quaternion_msg.x,
      quaternion_msg.y,
      quaternion_msg.z);
  if (quaternion.norm() < 1e-9) {
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

Eigen::Matrix4d MakeTransform(const Eigen::Vector3d& translation,
                              const Eigen::Quaterniond& rotation) {
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

airsim_ros::RotorPWM ZeroPwmMessage() {
  airsim_ros::RotorPWM message;
  message.rotorPWM0 = 0.0;
  message.rotorPWM1 = 0.0;
  message.rotorPWM2 = 0.0;
  message.rotorPWM3 = 0.0;
  return message;
}

class OfficialPwmControllerNode {
 public:
  OfficialPwmControllerNode() : nh_(), pnh_("~") {
    world_flu_from_world_ned_.setIdentity();
    world_flu_from_world_ned_(1, 1) = -1.0;
    world_flu_from_world_ned_(2, 2) = -1.0;

    LoadParameters();
    controller_.SetParameters(controller_params_);

    pwm_publisher_ = nh_.advertise<airsim_ros::RotorPWM>(pwm_command_topic_, 1);
    state_publisher_ = nh_.advertise<nav_msgs::Odometry>(state_topic_, 10);
    target_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(target_topic_, 10);

    odom_subscriber_ = nh_.subscribe(odom_topic_, 100, &OfficialPwmControllerNode::OdomCallback, this);
    initial_pose_subscriber_ = nh_.subscribe(
        initial_pose_topic_, 2, &OfficialPwmControllerNode::InitialPoseCallback, this);
    pose_setpoint_subscriber_ = nh_.subscribe(
        pose_setpoint_topic_, 10, &OfficialPwmControllerNode::PoseSetpointCallback, this);
    enable_subscriber_ = nh_.subscribe(enable_topic_, 5, &OfficialPwmControllerNode::EnableCallback, this);

    ROS_INFO_STREAM(
        "Official PWM controller ready. odom_topic=" << odom_topic_
        << ", pose_setpoint_topic=" << pose_setpoint_topic_
        << ", pwm_command_topic=" << pwm_command_topic_
        << ", warmup_samples=" << warmup_samples_);
  }

 private:
  void LoadParameters() {
    pnh_.param<std::string>("odom_topic", odom_topic_, "/rmua/control/official/eskf_odom");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/rmua/sensors/drone_1/initial_pose");
    pnh_.param<std::string>("pose_setpoint_topic", pose_setpoint_topic_, "/rmua/control/basic/pose_setpoint");
    pnh_.param<std::string>("enable_topic", enable_topic_, "/rmua/control/basic/enable");
    pnh_.param<std::string>("pwm_command_topic", pwm_command_topic_, "/airsim_node/drone_1/rotor_pwm_cmd");
    pnh_.param<std::string>("state_topic", state_topic_, "/rmua/control/official/state");
    pnh_.param<std::string>("target_topic", target_topic_, "/rmua/control/official/target_pose");
    pnh_.param<std::string>("state_frame_id", state_frame_id_, "home_flu");
    pnh_.param<std::string>("body_frame_id", body_frame_id_, "base_link_flu");

    pnh_.param("publish_zero_when_inactive", publish_zero_when_inactive_, true);
    pnh_.param("allow_first_odom_as_home", allow_first_odom_as_home_, true);
    pnh_.param("warmup_samples", warmup_samples_, 100);

    pnh_.param("mass", controller_params_.mass, controller_params_.mass);
    pnh_.param("gravity", controller_params_.gravity, controller_params_.gravity);
    pnh_.param("arm_length", controller_params_.arm_length, controller_params_.arm_length);
    pnh_.param("ixx", controller_params_.ixx, controller_params_.ixx);
    pnh_.param("iyy", controller_params_.iyy, controller_params_.iyy);
    pnh_.param("izz", controller_params_.izz, controller_params_.izz);
    pnh_.param("thrust_coefficient", controller_params_.thrust_coefficient, controller_params_.thrust_coefficient);
    pnh_.param("torque_coefficient", controller_params_.torque_coefficient, controller_params_.torque_coefficient);
    pnh_.param("max_force_per_motor", controller_params_.max_force_per_motor, controller_params_.max_force_per_motor);
    pnh_.param("kp_x", controller_params_.kp_x, controller_params_.kp_x);
    pnh_.param("kd_x", controller_params_.kd_x, controller_params_.kd_x);
    pnh_.param("kp_y", controller_params_.kp_y, controller_params_.kp_y);
    pnh_.param("kd_y", controller_params_.kd_y, controller_params_.kd_y);
    pnh_.param("kp_z", controller_params_.kp_z, controller_params_.kp_z);
    pnh_.param("kd_z", controller_params_.kd_z, controller_params_.kd_z);
    pnh_.param("kp_roll", controller_params_.kp_roll, controller_params_.kp_roll);
    pnh_.param("kd_roll", controller_params_.kd_roll, controller_params_.kd_roll);
    pnh_.param("kp_pitch", controller_params_.kp_pitch, controller_params_.kp_pitch);
    pnh_.param("kd_pitch", controller_params_.kd_pitch, controller_params_.kd_pitch);
    pnh_.param("kp_yaw", controller_params_.kp_yaw, controller_params_.kp_yaw);
    pnh_.param("kd_yaw", controller_params_.kd_yaw, controller_params_.kd_yaw);
    pnh_.param("kp_vx", controller_params_.kp_vx, controller_params_.kp_vx);
    pnh_.param("kd_vx", controller_params_.kd_vx, controller_params_.kd_vx);
    pnh_.param("kp_vy", controller_params_.kp_vy, controller_params_.kp_vy);
    pnh_.param("kd_vy", controller_params_.kd_vy, controller_params_.kd_vy);
    pnh_.param("max_acceleration", controller_params_.max_acceleration, controller_params_.max_acceleration);
    pnh_.param("max_velocity", controller_params_.max_velocity, controller_params_.max_velocity);
    pnh_.param("max_angle_rad", controller_params_.max_angle_rad, controller_params_.max_angle_rad);
    pnh_.param("max_roll_pitch_torque", controller_params_.max_roll_pitch_torque, controller_params_.max_roll_pitch_torque);
    pnh_.param("min_active_pwm", controller_params_.min_active_pwm, controller_params_.min_active_pwm);

    warmup_samples_ = std::max(0, warmup_samples_);
    controller_params_.min_active_pwm = Clamp(controller_params_.min_active_pwm, 0.0, 1.0);
  }

  void InitialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (home_from_initial_pose_) {
      return;
    }
    SetHomeLocked(
        Eigen::Vector3d(
            message->pose.position.x,
            message->pose.position.y,
            message->pose.position.z),
        ToEigenQuaternion(message->pose.orientation),
        "initial_pose",
        true);
  }

  void PoseSetpointCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_position_world_ned_ = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    target_attitude_world_ned_ = ToEigenQuaternion(message->pose.orientation);
    target_stamp_ = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    has_target_ = true;
  }

  void EnableCallback(const std_msgs::Bool::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    controller_enabled_ = message->data;
    controller_.Reset();
    odom_sample_count_ = 0;
    if (!controller_enabled_ && publish_zero_when_inactive_) {
      pwm_publisher_.publish(ZeroPwmMessage());
    }
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    const Eigen::Vector3d position_world_ned(
        message->pose.pose.position.x,
        message->pose.pose.position.y,
        message->pose.pose.position.z);
    const Eigen::Quaterniond attitude_world_ned = ToEigenQuaternion(message->pose.pose.orientation);

    if (!has_home_ && allow_first_odom_as_home_) {
      SetHomeLocked(position_world_ned, attitude_world_ned, "first_odom", false);
    }

    if (!has_home_) {
      ROS_WARN_THROTTLE(2.0, "Official PWM controller waiting for home pose.");
      return;
    }

    const Eigen::Matrix4d twb = MakeTransform(position_world_ned, attitude_world_ned);
    const Eigen::Matrix4d twflub = world_flu_from_world_ned_ * twb * world_flu_from_world_ned_;
    const Eigen::Matrix4d t0flub = home_pose_world_flu_.inverse() * twflub;

    const Eigen::Vector3d velocity_world_ned(
        message->twist.twist.linear.x,
        message->twist.twist.linear.y,
        message->twist.twist.linear.z);
    const Eigen::Vector3d velocity_body_ned =
        attitude_world_ned.toRotationMatrix().transpose() * velocity_world_ned;
    const Eigen::Vector3d velocity_body_flu =
        world_flu_from_world_ned_.block<3, 3>(0, 0) * velocity_body_ned;

    const Eigen::Vector3d angular_velocity_body_ned(
        message->twist.twist.angular.x,
        message->twist.twist.angular.y,
        message->twist.twist.angular.z);
    const Eigen::Vector3d angular_velocity_body_flu =
        world_flu_from_world_ned_.block<3, 3>(0, 0) * angular_velocity_body_ned;

    const RpyState rpy = ExtractFluRpy(t0flub.block<3, 3>(0, 0));

    nav_msgs::Odometry state_message;
    state_message.header.stamp = message->header.stamp;
    state_message.header.frame_id = state_frame_id_;
    state_message.child_frame_id = body_frame_id_;
    state_message.pose.pose.position.x = t0flub(0, 3);
    state_message.pose.pose.position.y = t0flub(1, 3);
    state_message.pose.pose.position.z = t0flub(2, 3);
    Eigen::Quaterniond relative_orientation(t0flub.block<3, 3>(0, 0));
    relative_orientation.normalize();
    state_message.pose.pose.orientation = ToRosQuaternion(relative_orientation);
    state_message.twist.twist.linear.x = velocity_body_flu.x();
    state_message.twist.twist.linear.y = velocity_body_flu.y();
    state_message.twist.twist.linear.z = velocity_body_flu.z();
    state_message.twist.twist.angular.x = angular_velocity_body_flu.x();
    state_message.twist.twist.angular.y = angular_velocity_body_flu.y();
    state_message.twist.twist.angular.z = angular_velocity_body_flu.z();
    state_publisher_.publish(state_message);

    if (has_target_) {
      geometry_msgs::PoseStamped target_message;
      target_message.header.stamp = target_stamp_.isZero() ? message->header.stamp : target_stamp_;
      target_message.header.frame_id = "world_ned";
      target_message.pose.position.x = target_position_world_ned_.x();
      target_message.pose.position.y = target_position_world_ned_.y();
      target_message.pose.position.z = target_position_world_ned_.z();
      target_message.pose.orientation = ToRosQuaternion(target_attitude_world_ned_);
      target_publisher_.publish(target_message);
    }

    if (!controller_enabled_ || !has_target_) {
      if (publish_zero_when_inactive_) {
        pwm_publisher_.publish(ZeroPwmMessage());
      }
      return;
    }

    ++odom_sample_count_;
    if (odom_sample_count_ <= warmup_samples_) {
      if (publish_zero_when_inactive_) {
        pwm_publisher_.publish(ZeroPwmMessage());
      }
      return;
    }

    const Eigen::Matrix4d target_world_ned =
        MakeTransform(target_position_world_ned_, target_attitude_world_ned_);
    const Eigen::Matrix4d target_world_flu =
        world_flu_from_world_ned_ * target_world_ned * world_flu_from_world_ned_;
    const Eigen::Matrix4d target_rel_flu = home_pose_world_flu_.inverse() * target_world_flu;
    const RpyState target_rpy = ExtractFluRpy(target_rel_flu.block<3, 3>(0, 0));

    Eigen::Matrix<double, 12, 1> desired_state = Eigen::Matrix<double, 12, 1>::Zero();
    desired_state[0] = target_rel_flu(0, 3);
    desired_state[1] = target_rel_flu(1, 3);
    desired_state[2] = target_rel_flu(2, 3);
    desired_state[8] = target_rpy.yaw;

    Eigen::Matrix<double, 12, 1> actual_state = Eigen::Matrix<double, 12, 1>::Zero();
    actual_state[0] = t0flub(0, 3);
    actual_state[1] = t0flub(1, 3);
    actual_state[2] = t0flub(2, 3);
    actual_state[3] = velocity_body_flu.x();
    actual_state[4] = velocity_body_flu.y();
    actual_state[5] = velocity_body_flu.z();
    actual_state[6] = rpy.roll;
    actual_state[7] = rpy.pitch;
    actual_state[8] = rpy.yaw;
    actual_state[9] = angular_velocity_body_flu.x();
    actual_state[10] = angular_velocity_body_flu.y();
    actual_state[11] = angular_velocity_body_flu.z();

    const Eigen::Vector4d pwm = controller_.Execute(desired_state, actual_state);
    airsim_ros::RotorPWM pwm_message;
    pwm_message.rotorPWM0 = pwm[0];
    pwm_message.rotorPWM1 = pwm[1];
    pwm_message.rotorPWM2 = pwm[2];
    pwm_message.rotorPWM3 = pwm[3];
    pwm_publisher_.publish(pwm_message);
  }

  void SetHomeLocked(const Eigen::Vector3d& position_world_ned,
                     const Eigen::Quaterniond& attitude_world_ned,
                     const std::string& source,
                     const bool from_initial_pose) {
    if (has_home_ && home_from_initial_pose_ && !from_initial_pose) {
      return;
    }

    home_pose_world_ned_ = MakeTransform(position_world_ned, attitude_world_ned);
    home_pose_world_flu_ =
        world_flu_from_world_ned_ * home_pose_world_ned_ * world_flu_from_world_ned_;
    has_home_ = true;
    home_from_initial_pose_ = from_initial_pose;

    ROS_INFO_STREAM(
        "Official PWM controller home initialized from " << source << " at ["
        << position_world_ned.x() << ", "
        << position_world_ned.y() << ", "
        << position_world_ned.z() << "]");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  rmua_flight_control::OfficialControllerParams controller_params_;
  rmua_flight_control::OfficialPdController controller_;

  ros::Publisher pwm_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher target_publisher_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber initial_pose_subscriber_;
  ros::Subscriber pose_setpoint_subscriber_;
  ros::Subscriber enable_subscriber_;

  std::mutex data_mutex_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string pose_setpoint_topic_;
  std::string enable_topic_;
  std::string pwm_command_topic_;
  std::string state_topic_;
  std::string target_topic_;
  std::string state_frame_id_;
  std::string body_frame_id_;
  bool publish_zero_when_inactive_ = true;
  bool allow_first_odom_as_home_ = true;
  int warmup_samples_ = 100;

  bool controller_enabled_ = false;
  bool has_home_ = false;
  bool home_from_initial_pose_ = false;
  bool has_target_ = false;
  int odom_sample_count_ = 0;
  ros::Time target_stamp_;
  Eigen::Matrix4d world_flu_from_world_ned_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d home_pose_world_ned_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d home_pose_world_flu_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d target_position_world_ned_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond target_attitude_world_ned_ = Eigen::Quaterniond::Identity();
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "official_pwm_controller_node");
  OfficialPwmControllerNode node;
  ros::spin();
  return 0;
}