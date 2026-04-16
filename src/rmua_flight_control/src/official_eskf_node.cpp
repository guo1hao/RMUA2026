#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>

#include "rmua_flight_control/official_eskf.hpp"

namespace {

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

class OfficialEskfNode {
 public:
  OfficialEskfNode() : nh_(), pnh_("~") {
    LoadParameters();

    filter_.reset(new rmua_flight_control::OfficialEskf(
        gravity_,
        pos_noise_,
        vel_noise_,
        ori_noise_,
        gyr_bias_noise_,
        acc_bias_noise_,
        pos_std_,
        ori_std_,
        gyr_noise_,
        acc_noise_));

    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);

    gps_subscriber_ = nh_.subscribe(gps_topic_, 10, &OfficialEskfNode::GpsCallback, this);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 200, &OfficialEskfNode::ImuCallback, this);
    initial_pose_subscriber_ = nh_.subscribe(
        initial_pose_topic_, 2, &OfficialEskfNode::InitialPoseCallback, this);

    ROS_INFO_STREAM(
        "Official ESKF node ready. gps_topic=" << gps_topic_
        << ", imu_topic=" << imu_topic_
        << ", initial_pose_topic=" << initial_pose_topic_
        << ", odom_topic=" << odom_topic_
        << ", pose_topic=" << pose_topic_);
  }

 private:
  void LoadParameters() {
    pnh_.param<std::string>("gps_topic", gps_topic_, "/rmua/sensors/drone_1/gps");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/rmua/sensors/drone_1/imu");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/rmua/sensors/drone_1/initial_pose");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/rmua/control/official/eskf_odom");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/rmua/control/official/eskf_pose");

    pnh_.param("gravity", gravity_, -9.81083);
    pnh_.param("pos_noise", pos_noise_, 0.1);
    pnh_.param("vel_noise", vel_noise_, 0.1);
    pnh_.param("ori_noise", ori_noise_, 0.1);
    pnh_.param("gyr_bias_noise", gyr_bias_noise_, 0.0003158085227);
    pnh_.param("acc_bias_noise", acc_bias_noise_, 0.001117221);
    pnh_.param("pos_std", pos_std_, 5.0);
    pnh_.param("ori_std", ori_std_, 1.0);
    pnh_.param("gyr_noise", gyr_noise_, 0.00143);
    pnh_.param("acc_noise", acc_noise_, 0.0386);
  }

  void InitialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (filter_->is_initialized()) {
      return;
    }

    Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();
    initial_pose.block<3, 3>(0, 0) = ToEigenQuaternion(message->pose.orientation).toRotationMatrix();
    initial_pose.block<3, 1>(0, 3) <<
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z;

    const ros::Time stamp = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    filter_->Init(initial_pose, Eigen::Vector3d::Zero(), stamp.toNSec());
  }

  void GpsCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!filter_->is_initialized()) {
      return;
    }

    filter_->Correct(
        Eigen::Vector3d(
            message->pose.position.x,
            message->pose.position.y,
            message->pose.position.z),
        ToEigenQuaternion(message->pose.orientation));
  }

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    if (!filter_->is_initialized()) {
      ROS_WARN_THROTTLE(2.0, "Official ESKF waiting for initial_pose.");
      return;
    }

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
    Eigen::Quaterniond orientation;
    if (!filter_->Predict(
            Eigen::Vector3d(
                message->linear_acceleration.x,
                message->linear_acceleration.y,
                message->linear_acceleration.z),
            Eigen::Vector3d(
                message->angular_velocity.x,
                message->angular_velocity.y,
                message->angular_velocity.z),
            &position,
            &velocity,
            &angular_velocity,
            &orientation,
            message->header.stamp.toNSec())) {
      return;
    }

    nav_msgs::Odometry odom_message;
    odom_message.header.stamp = message->header.stamp;
    odom_message.header.frame_id = "world_ned";
    odom_message.child_frame_id = "base_link_ned";
    odom_message.pose.pose.position.x = position.x();
    odom_message.pose.pose.position.y = position.y();
    odom_message.pose.pose.position.z = position.z();
    odom_message.pose.pose.orientation = ToRosQuaternion(orientation);
    odom_message.twist.twist.linear.x = velocity.x();
    odom_message.twist.twist.linear.y = velocity.y();
    odom_message.twist.twist.linear.z = velocity.z();
    odom_message.twist.twist.angular.x = angular_velocity.x();
    odom_message.twist.twist.angular.y = angular_velocity.y();
    odom_message.twist.twist.angular.z = angular_velocity.z();
    odom_publisher_.publish(odom_message);

    geometry_msgs::PoseStamped pose_message;
    pose_message.header = odom_message.header;
    pose_message.pose = odom_message.pose.pose;
    pose_publisher_.publish(pose_message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<rmua_flight_control::OfficialEskf> filter_;

  ros::Publisher odom_publisher_;
  ros::Publisher pose_publisher_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber initial_pose_subscriber_;

  std::string gps_topic_;
  std::string imu_topic_;
  std::string initial_pose_topic_;
  std::string odom_topic_;
  std::string pose_topic_;

  double gravity_ = -9.81083;
  double pos_noise_ = 0.1;
  double vel_noise_ = 0.1;
  double ori_noise_ = 0.1;
  double gyr_bias_noise_ = 0.0003158085227;
  double acc_bias_noise_ = 0.001117221;
  double pos_std_ = 5.0;
  double ori_std_ = 1.0;
  double gyr_noise_ = 0.00143;
  double acc_noise_ = 0.0386;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "official_eskf_node");
  OfficialEskfNode node;
  ros::spin();
  return 0;
}