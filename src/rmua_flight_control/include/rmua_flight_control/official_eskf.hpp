#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mutex>

namespace rmua_flight_control {

class OfficialEskf {
 public:
  OfficialEskf(double gravity,
               double pos_noise,
               double vel_noise,
               double ori_noise,
               double gyr_bias_noise,
               double acc_bias_noise,
               double pos_std,
               double ori_std,
               double gyr_noise,
               double acc_noise);

  bool Init(const Eigen::Matrix4d& init_pose,
            const Eigen::Vector3d& init_velocity,
            long long timestamp_ns);

  bool Predict(const Eigen::Vector3d& imu_acc,
               const Eigen::Vector3d& imu_gyr,
               Eigen::Vector3d* position,
               Eigen::Vector3d* velocity,
               Eigen::Vector3d* angular_velocity,
               Eigen::Quaterniond* orientation,
               long long timestamp_ns);

  bool Correct(const Eigen::Vector3d& gps_position,
               const Eigen::Quaterniond& gps_orientation);

  bool is_initialized() const { return is_initialized_; }

 private:
  static constexpr unsigned int kStateDim = 15;
  static constexpr unsigned int kStateNoiseDim = 6;
  static constexpr unsigned int kMeasurementDim = 6;
  static constexpr unsigned int kMeasurementNoiseDim = 6;
  static constexpr unsigned int kStatePosIndex = 0;
  static constexpr unsigned int kStateVelIndex = 3;
  static constexpr unsigned int kStateOriIndex = 6;
  static constexpr unsigned int kStateGyroBiasIndex = 9;
  static constexpr unsigned int kStateAccelBiasIndex = 12;
  static constexpr unsigned int kMeasurementPosIndex = 0;

  static Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& vector);

  bool is_initialized_ = false;
  Eigen::Matrix<double, kStateDim, 1> state_ =
      Eigen::Matrix<double, kStateDim, 1>::Zero();
  Eigen::Matrix<double, kMeasurementDim, 1> innovation_ =
      Eigen::Matrix<double, kMeasurementDim, 1>::Zero();
  Eigen::Matrix<double, kStateDim, kStateDim> transition_jacobian_ =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kStateNoiseDim> process_noise_jacobian_ =
      Eigen::Matrix<double, kStateDim, kStateNoiseDim>::Zero();
  Eigen::Matrix<double, kStateNoiseDim, kStateNoiseDim> process_noise_covariance_ =
      Eigen::Matrix<double, kStateNoiseDim, kStateNoiseDim>::Zero();
  Eigen::Matrix<double, kStateDim, kStateDim> covariance_ =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kMeasurementDim> kalman_gain_ =
      Eigen::Matrix<double, kStateDim, kMeasurementDim>::Zero();
  Eigen::Matrix<double, kMeasurementNoiseDim, kMeasurementNoiseDim> measurement_matrix_ =
      Eigen::Matrix<double, kMeasurementNoiseDim, kMeasurementNoiseDim>::Zero();
  Eigen::Matrix<double, kMeasurementDim, kStateDim> observation_jacobian_ =
      Eigen::Matrix<double, kMeasurementDim, kStateDim>::Zero();
  Eigen::Matrix<double, kMeasurementDim, kMeasurementDim> measurement_noise_covariance_ =
      Eigen::Matrix<double, kMeasurementDim, kMeasurementDim>::Zero();

  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_vector_ = Eigen::Vector3d::Zero();

  long long last_imu_timestamp_ns_ = 0;
  Eigen::Vector3d last_unbiased_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_unbiased_gyr_ = Eigen::Vector3d::Zero();
  mutable std::mutex mutex_;
};

}  // namespace rmua_flight_control