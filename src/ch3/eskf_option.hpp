//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_ESKF_OPTIONS_HPP
#define SLAM_IN_AUTO_DRIVING_ESKFOPTIONS_HPP

#include "common/eigen_types.h"
#include "common/math_utils.h"

namespace sad {

struct ESKFOptions {
    /// IMU measurements and bias parameters
    double imu_dt_ = 0.01;  // IMU measurement interval
    // NOTE: IMU noise terms are already discrete-time, no need to multiply
    // by dt, they can be specified in the initializer as IMU noise.
    double gyro_var_ =
        0.00001;  // 1e-5, Gyroscope measurement standard deviation
    double acce_var_ =
        0.01;  // 1e-2, Accelerometer measurement standard deviation
    double bias_gyro_var_ =
        1e-6;  // 1e-6;, Gyroscope bias random walk standard deviation
    double bias_acce_var_ =
        1e-4;  // 1e-4;, Accelerometer bias random walk standard deviation

    /// Odometry parameters
    double odom_var_ = 0.5;
    double odom_span_ = 0.1;        // Odometry measurement interval
    double wheel_radius_ = 0.155;   // Wheel radius
    double circle_pulse_ = 1024.0;  // Encoder pulse count per revolution

    /// RTK observation parameters
    double gnss_pos_noise_ = 0.1;                   // 0.1,. GNSS position noise
    double gnss_height_noise_ = 0.1;                // 0.1, GNSS height noise
    double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // 1.0, GNSS rotation noise

    /// Other configurations
    bool update_bias_gyro_ = true;  // Whether to update gyroscope bias
    bool update_bias_acce_ = true;  // Whether to update accelerometer bias
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_ESKFOPTIONS_HPP
