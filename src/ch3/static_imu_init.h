//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
#define SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/odom.h"

#include <deque>

namespace sad {

/**
    * IMU initializer for horizontal stationary state.

    * Usage: Call AddIMU and AddOdom to add data, use InitSuccess to check if
    initialization was successful.

    * Once successful, use the respective Get functions to retrieve internal
    parameters.

    * The initializer attempts to initialize the system every time AddIMU is
   called. In the presence of odometry data, the initialization requires the
   odometry wheel speed readings to be close to zero. In the absence of
   odometry, it assumes the vehicle is initially stationary.

    * The initializer collects IMU readings over a period of time and estimates
    initial biases and noise parameters according to section 3.5.4 of the book,
    providing them to the ESKF or other filters.
 */

class StaticIMUInit {
   public:
    struct Options {
        Options() {}
        const double init_time_seconds_ = 10.0;
        const int init_imu_queue_max_size_ =
            2000;  // Maximum length of the IMU queue during initialization
        const int static_odom_pulse_ =
            5;  // Noise in the odometry output during stationary state
        const double max_static_gyro_var =
            0.5;  // Variance of gyroscope measurements in static state
        const double max_static_acce_var =
            0.05;  // Variance of accelerometer measurements in static state
        const double gravity_norm_ = 9.81;  // Magnitude of gravity
        const bool use_speed_for_static_checking_ =
            true;  // Whether to use odom to determine vehicle stationary state
                   // (some datasets may not have odom option)
    };

    StaticIMUInit(Options options = Options()) : options_(options) {}

    bool AddIMU(const IMU& imu);
    bool AddOdom(const Odom& odom);

    /// Determine if initialization is successful
    bool InitSuccess() const { return init_success_; }

    /// 获取各Cov, bias, gravity
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

   private:
    /// Attempt system initialization
    bool TryInit();

    Options options_;
    bool init_success_ = false;

    // the boths measurement noise covariances
    // are evaluated during initialization
    Vec3d cov_gyro_ = Vec3d::Zero();
    Vec3d cov_acce_ = Vec3d::Zero();

    Vec3d init_bg_ = Vec3d::Zero();  // Initial gyroscope bias
    Vec3d init_ba_ = Vec3d::Zero();  // Initial accelerometer bias

    Vec3d gravity_ = Vec3d::Zero();

    bool is_static_ = false;  // Flag indicating if the vehicle is stationary

    std::deque<IMU> init_imu_deque_;  // Data for initialization
    double current_time_ = 0.0;       // Current time
    double init_start_time_ = 0.0;    // Initial time for stationary state
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
