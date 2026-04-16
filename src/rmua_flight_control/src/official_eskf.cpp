#include "rmua_flight_control/official_eskf.hpp"

#include <cmath>

namespace rmua_flight_control {

OfficialEskf::OfficialEskf(double gravity,
                           double pos_noise,
                           double vel_noise,
                           double ori_noise,
                           double gyr_bias_noise,
                           double acc_bias_noise,
                           double pos_std,
                           double ori_std,
                           double gyr_noise,
                           double acc_noise) {
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, gravity);
  covariance_.block<3, 3>(kStatePosIndex, kStatePosIndex) =
      Eigen::Matrix3d::Identity() * pos_noise * pos_noise;
  covariance_.block<3, 3>(kStateVelIndex, kStateVelIndex) =
      Eigen::Matrix3d::Identity() * vel_noise * vel_noise;
  covariance_.block<3, 3>(kStateOriIndex, kStateOriIndex) =
      Eigen::Matrix3d::Identity() * ori_noise * ori_noise;
  covariance_.block<3, 3>(kStateGyroBiasIndex, kStateGyroBiasIndex) =
      Eigen::Matrix3d::Identity() * gyr_bias_noise * gyr_bias_noise;
  covariance_.block<3, 3>(kStateAccelBiasIndex, kStateAccelBiasIndex) =
      Eigen::Matrix3d::Identity() * acc_bias_noise * acc_bias_noise;

  measurement_noise_covariance_(0, 0) = pos_std * pos_std;
  measurement_noise_covariance_(1, 1) = pos_std * pos_std;
  measurement_noise_covariance_(2, 2) = pos_std * pos_std;
  measurement_noise_covariance_(3, 3) = ori_std * ori_std;
  measurement_noise_covariance_(4, 4) = ori_std * ori_std;
  measurement_noise_covariance_(5, 5) = ori_std * ori_std;

  process_noise_covariance_.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * gyr_noise * gyr_noise;
  process_noise_covariance_.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * acc_noise * acc_noise;

  measurement_matrix_ =
      Eigen::Matrix<double, kMeasurementNoiseDim, kMeasurementNoiseDim>::Identity();
  observation_jacobian_.block<3, 3>(kMeasurementPosIndex, kMeasurementPosIndex) =
      Eigen::Matrix3d::Identity();
  observation_jacobian_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d OfficialEskf::SkewSymmetric(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
  skew << 0.0, -vector.z(), vector.y(),
      vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

bool OfficialEskf::Init(const Eigen::Matrix4d& init_pose,
                        const Eigen::Vector3d& init_velocity,
                        long long timestamp_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = init_pose;
  velocity_ = init_velocity;
  last_imu_timestamp_ns_ = timestamp_ns;
  last_unbiased_acc_.setZero();
  last_unbiased_gyr_.setZero();
  is_initialized_ = true;
  return true;
}

bool OfficialEskf::Predict(const Eigen::Vector3d& imu_acc,
                           const Eigen::Vector3d& imu_gyr,
                           Eigen::Vector3d* position,
                           Eigen::Vector3d* velocity,
                           Eigen::Vector3d* angular_velocity,
                           Eigen::Quaterniond* orientation,
                           long long timestamp_ns) {
  if (!is_initialized_) {
    return false;
  }

  const double delta_t = (timestamp_ns - last_imu_timestamp_ns_) / 1000000000.0;
  if (delta_t < 0.005 || delta_t > 0.015) {
    last_imu_timestamp_ns_ = timestamp_ns;
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const Eigen::Vector3d unbiased_gyr = imu_gyr - gyro_bias_;
  const Eigen::Vector3d phi = (unbiased_gyr + last_unbiased_gyr_) * 0.5 * delta_t;
  const double phi_norm = phi.norm();
  const Eigen::Matrix3d skew_phi = SkewSymmetric(phi);
  Eigen::Matrix3d delta_rotation = Eigen::Matrix3d::Identity();
  if (phi_norm > 1e-12) {
    delta_rotation += std::sin(phi_norm) / phi_norm * skew_phi +
        (1.0 - std::cos(phi_norm)) / (phi_norm * phi_norm) * skew_phi * skew_phi;
  } else {
    delta_rotation += skew_phi + 0.5 * skew_phi * skew_phi;
  }

  const Eigen::Matrix3d last_pose = pose_.block<3, 3>(0, 0);
  pose_.block<3, 3>(0, 0) = last_pose * delta_rotation;

  const Eigen::Vector3d unbiased_acc = imu_acc - accel_bias_ - gravity_vector_;
  const Eigen::Vector3d last_velocity = velocity_;
  velocity_ = last_velocity +
      (pose_.block<3, 3>(0, 0) * unbiased_acc + last_pose * last_unbiased_acc_) *
          0.5 * delta_t;
  pose_.block<3, 1>(0, 3) +=
      (last_velocity + velocity_) * 0.5 * delta_t +
      0.25 * (last_pose * last_unbiased_acc_ + pose_.block<3, 3>(0, 0) * unbiased_acc) *
          delta_t * delta_t;

  const Eigen::Vector3d current_acc_ned = pose_.block<3, 3>(0, 0) * (imu_acc - accel_bias_);
  transition_jacobian_.setZero();
  process_noise_jacobian_.setZero();

  transition_jacobian_.block<3, 3>(kStatePosIndex, kStateVelIndex) =
      Eigen::Matrix3d::Identity();
  transition_jacobian_.block<3, 3>(kStateVelIndex, kStateOriIndex) =
      SkewSymmetric(current_acc_ned);
  transition_jacobian_.block<3, 3>(kStateVelIndex, kStateAccelBiasIndex) =
      pose_.block<3, 3>(0, 0);
  transition_jacobian_.block<3, 3>(kStateOriIndex, kStateGyroBiasIndex) =
      -pose_.block<3, 3>(0, 0);

  process_noise_jacobian_.block<3, 3>(kStateVelIndex, 3) = pose_.block<3, 3>(0, 0);
  process_noise_jacobian_.block<3, 3>(kStateOriIndex, 0) = -pose_.block<3, 3>(0, 0);

  const Eigen::Matrix<double, kStateDim, kStateDim> discrete_transition =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity() + transition_jacobian_ * delta_t;

  state_ = discrete_transition * state_;
  covariance_ = discrete_transition * covariance_ * discrete_transition.transpose() +
      process_noise_jacobian_ * process_noise_covariance_ * process_noise_jacobian_.transpose();

  last_imu_timestamp_ns_ = timestamp_ns;
  last_unbiased_gyr_ = unbiased_gyr;
  last_unbiased_acc_ = unbiased_acc;

  if (position != nullptr) {
    *position = pose_.block<3, 1>(0, 3);
  }
  if (velocity != nullptr) {
    *velocity = velocity_;
  }
  if (angular_velocity != nullptr) {
    *angular_velocity = last_unbiased_gyr_;
  }
  if (orientation != nullptr) {
    Eigen::Quaterniond quaternion(pose_.block<3, 3>(0, 0));
    quaternion.normalize();
    *orientation = quaternion;
  }

  return true;
}

bool OfficialEskf::Correct(const Eigen::Vector3d& gps_position,
                           const Eigen::Quaterniond& gps_orientation) {
  if (!is_initialized_ || gps_orientation.norm() < 1e-9) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  innovation_.block<3, 1>(0, 0) = gps_position - pose_.block<3, 1>(0, 3);

  Eigen::Quaterniond normalized_gps = gps_orientation;
  normalized_gps.normalize();
  const Eigen::Matrix3d measurement_rotation = normalized_gps.toRotationMatrix();
  const Eigen::Matrix3d error_rotation = measurement_rotation.inverse() * pose_.block<3, 3>(0, 0);
  const Eigen::AngleAxisd angle_axis(error_rotation);
  innovation_.block<3, 1>(3, 0) = angle_axis.axis() * angle_axis.angle();

  kalman_gain_ = covariance_ * observation_jacobian_.transpose() *
      (observation_jacobian_ * covariance_ * observation_jacobian_.transpose() +
       measurement_matrix_ * measurement_noise_covariance_ * measurement_matrix_.transpose())
          .inverse();

  covariance_ =
      (Eigen::Matrix<double, kStateDim, kStateDim>::Identity() - kalman_gain_ * observation_jacobian_) *
      covariance_;
  state_ = state_ + kalman_gain_ * (innovation_ - observation_jacobian_ * state_);

  pose_.block<3, 1>(0, 3) += state_.block<3, 1>(kStatePosIndex, 0);
  velocity_ += state_.block<3, 1>(kStateVelIndex, 0);

  const Eigen::Vector3d orientation_error = state_.block<3, 1>(kStateOriIndex, 0);
  const double orientation_error_norm = orientation_error.norm();
  if (orientation_error_norm > 1e-12) {
    const Eigen::AngleAxisd orientation_correction(
        orientation_error_norm, orientation_error.normalized());
    pose_.block<3, 3>(0, 0) =
        pose_.block<3, 3>(0, 0) * orientation_correction.toRotationMatrix().inverse();
  }

  gyro_bias_ += state_.block<3, 1>(kStateGyroBiasIndex, 0);
  accel_bias_ += state_.block<3, 1>(kStateAccelBiasIndex, 0);
  state_.setZero();
  return true;
}

}  // namespace rmua_flight_control