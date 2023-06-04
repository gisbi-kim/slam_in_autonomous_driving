//
// Created by xiang on 2021/11/5.
//

#ifndef SLAM_IN_AUTO_DRIVING_IMU_INTEGRATION_H
#define SLAM_IN_AUTO_DRIVING_IMU_INTEGRATION_H

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/nav_state.h"

namespace sad {

/**
 * This program demonstrates the use of pure IMU integration for navigation.
 */
class IMUIntegration {
   public:
    IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg,
                   const Vec3d& init_ba)
        : gravity_(gravity), bg_(init_bg), ba_(init_ba) {}

    void AddIMU(const IMU& imu) {
        double dt = imu.timestamp_ - timestamp_;

        // Assuming IMU time intervals are within the range of 0 to 0.1
        if (dt > 0 && dt < 0.1) {
            p_ = p_ + (v_ * dt) +
                 (0.5 * (R_ * (imu.acce_ - ba_) + gravity_) * dt * dt);
            v_ = v_ + (R_ * (imu.acce_ - ba_) * dt) + (gravity_ * dt);
            R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
        }

        // Update time
        timestamp_ = imu.timestamp_;
    }

    NavStated GetNavState() const {
        return NavStated(timestamp_, R_, p_, v_, bg_, ba_);
    }

    SO3 GetR() const { return R_; }
    Vec3d GetV() const { return v_; }
    Vec3d GetP() const { return p_; }

   private:
    SO3 R_;
    Vec3d v_ = Vec3d::Zero();
    Vec3d p_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d gravity_ = Vec3d(0, 0, -9.8);

    double timestamp_ = 0.0;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_IMU_INTEGRATION_H
