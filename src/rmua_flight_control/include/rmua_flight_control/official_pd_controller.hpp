#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace rmua_flight_control {

struct OfficialControllerParams {
  double mass = 0.9;
  double gravity = 9.8;
  double arm_length = 0.18;
  double ixx = 0.0046890742;
  double iyy = 0.0069312;
  double izz = 0.010421166;
  double thrust_coefficient = 0.00036771704516278653;
  double torque_coefficient = 4.888486266072161e-06;
  double max_force_per_motor = 4.179446268 * 3.0;
  double kp_x = 1.0;
  double kd_x = 0.1;
  double kp_y = 1.0;
  double kd_y = 0.1;
  double kp_z = 2.0;
  double kd_z = 25.0;
  double kp_roll = 17.0;
  double kd_roll = 940.0;
  double kp_pitch = 20.0;
  double kd_pitch = 750.0;
  double kp_yaw = 5.0;
  double kd_yaw = 250.0;
  double kp_vx = 0.5;
  double kd_vx = 1.0;
  double kp_vy = 0.5;
  double kd_vy = 1.0;
  double max_acceleration = 20.0;
  double max_velocity = 20.0;
  double max_angle_rad = 0.5;
  double max_roll_pitch_torque = 2.0;
  double min_active_pwm = 0.1;
};

class OfficialPdController {
 public:
  explicit OfficialPdController(
      const OfficialControllerParams& params = OfficialControllerParams())
      : params_(params) {
    const Eigen::Matrix4d mixer_matrix = (Eigen::Matrix4d() <<
        1.0, 1.0, 1.0, 1.0,
       -1.0, 1.0, 1.0, -1.0,
       -1.0, 1.0, -1.0, 1.0,
       -1.0, -1.0, 1.0, 1.0).finished();
    mixer_inverse_ = mixer_matrix.inverse();
  }

  void SetParameters(const OfficialControllerParams& params) {
    params_ = params;
    Reset();
  }

  void Reset() {
    last_e_x_ = 0.0;
    last_e_y_ = 0.0;
    last_e_z_ = 0.0;
    last_e_vx_ = 0.0;
    last_e_vy_ = 0.0;
    last_e_phi_ = 0.0;
    last_e_theta_ = 0.0;
    last_e_psi_ = 0.0;
    roll_error_initialized_ = false;
    pitch_error_initialized_ = false;
  }

  Eigen::Vector4d Execute(const Eigen::Matrix<double, 12, 1>& desired_state,
                          const Eigen::Matrix<double, 12, 1>& actual_state) {
    const double cur_e_z = desired_state[2] - actual_state[2];
    double collective_force =
        params_.mass * params_.gravity +
        params_.mass * (params_.kd_z * (cur_e_z - last_e_z_) + params_.kp_z * cur_e_z);
    last_e_z_ = cur_e_z;
    collective_force = std::min(collective_force, params_.max_force_per_motor * 4.0);
    collective_force = std::max(collective_force, 1e-3);

    const double cur_e_x = desired_state[0] - actual_state[0];
    double desired_vx =
        std::max(-params_.max_velocity,
                 std::min(params_.max_velocity,
                          params_.kd_x * (cur_e_x - last_e_x_) + params_.kp_x * cur_e_x));
    last_e_x_ = cur_e_x;
    const double cur_e_vx = desired_vx - actual_state[3];
    double desired_ax =
        std::max(-params_.max_acceleration,
                 std::min(params_.max_acceleration,
                          params_.kd_vx * (cur_e_vx - last_e_vx_) + params_.kp_vx * cur_e_vx));
    last_e_vx_ = cur_e_vx;

    const double cur_e_y = desired_state[1] - actual_state[1];
    double desired_vy =
        std::max(-params_.max_velocity,
                 std::min(params_.max_velocity,
                          params_.kd_y * (cur_e_y - last_e_y_) + params_.kp_y * cur_e_y));
    last_e_y_ = cur_e_y;
    const double cur_e_vy = desired_vy - actual_state[4];
    double desired_ay =
        std::max(-params_.max_acceleration,
                 std::min(params_.max_acceleration,
                          params_.kd_vy * (cur_e_vy - last_e_vy_) + params_.kp_vy * cur_e_vy));
    last_e_vy_ = cur_e_vy;

    const double desired_yaw = desired_state[8];
    double desired_roll = params_.mass / collective_force *
        (desired_ax * std::sin(desired_yaw) - desired_ay * std::cos(desired_yaw));
    double desired_pitch = params_.mass / collective_force *
      (desired_ax * std::cos(desired_yaw) + desired_ay * std::sin(desired_yaw));
    desired_roll = std::max(-params_.max_angle_rad,
                            std::min(params_.max_angle_rad, desired_roll));
    desired_pitch = std::max(-params_.max_angle_rad,
                             std::min(params_.max_angle_rad, desired_pitch));

    const double cur_e_roll = desired_roll - actual_state[6];
    if (!roll_error_initialized_) {
      last_e_phi_ = cur_e_roll;
      roll_error_initialized_ = true;
    }
    double torque_x = params_.ixx *
        (params_.kp_roll * cur_e_roll + params_.kd_roll * (cur_e_roll - last_e_phi_));
    torque_x = std::max(-params_.max_roll_pitch_torque,
                        std::min(params_.max_roll_pitch_torque, torque_x));
    last_e_phi_ = cur_e_roll;

    const double cur_e_pitch = desired_pitch - actual_state[7];
    if (!pitch_error_initialized_) {
      last_e_theta_ = cur_e_pitch;
      pitch_error_initialized_ = true;
    }
    double torque_y = params_.iyy *
        (params_.kp_pitch * cur_e_pitch + params_.kd_pitch * (cur_e_pitch - last_e_theta_));
    torque_y = std::max(-params_.max_roll_pitch_torque,
                        std::min(params_.max_roll_pitch_torque, torque_y));
    last_e_theta_ = cur_e_pitch;

    const double cur_e_yaw = desired_yaw - actual_state[8];
    const double torque_z = params_.izz *
        (params_.kp_yaw * cur_e_yaw + params_.kd_yaw * (cur_e_yaw - last_e_psi_));
    last_e_psi_ = cur_e_yaw;

    static constexpr double kSqrtHalf = 0.7071067811865476;
    const Eigen::Vector4d control_vector(
        collective_force / params_.thrust_coefficient,
        torque_x / (params_.arm_length * params_.thrust_coefficient * kSqrtHalf),
        torque_y / (params_.arm_length * params_.thrust_coefficient * kSqrtHalf),
        torque_z / params_.torque_coefficient);
    const Eigen::Vector4d thrust_vector =
        params_.thrust_coefficient * mixer_inverse_ * control_vector;

    Eigen::Vector4d pwm = Eigen::Vector4d::Zero();
    double max_output = 0.0;
    for (int index = 0; index < 4; ++index) {
      pwm[index] = std::max(params_.min_active_pwm,
                            thrust_vector[index] / params_.max_force_per_motor);
      max_output = std::max(max_output, pwm[index]);
    }

    if (max_output > 1.0) {
      for (int index = 0; index < 4; ++index) {
        pwm[index] = std::max(params_.min_active_pwm, pwm[index] / max_output);
      }
    }

    return pwm;
  }

 private:
  OfficialControllerParams params_;
  Eigen::Matrix4d mixer_inverse_ = Eigen::Matrix4d::Identity();

  double last_e_x_ = 0.0;
  double last_e_y_ = 0.0;
  double last_e_z_ = 0.0;
  double last_e_vx_ = 0.0;
  double last_e_vy_ = 0.0;
  double last_e_phi_ = 0.0;
  double last_e_theta_ = 0.0;
  double last_e_psi_ = 0.0;
  bool roll_error_initialized_ = false;
  bool pitch_error_initialized_ = false;
};

}  // namespace rmua_flight_control