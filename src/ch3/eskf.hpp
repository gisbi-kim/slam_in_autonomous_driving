//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_ESKF_HPP
#define SLAM_IN_AUTO_DRIVING_ESKF_HPP

#include "ch3/eskf_option.hpp"
#include "common/eigen_types.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/nav_state.h"
#include "common/odom.h"

#include <glog/logging.h>
#include <iomanip>

namespace sad {

/**
 * Error-state Kalman Filter introduced in Chapter 3 of the book.
 * The filter can be specified to observe GNSS readings, which
 *  should be pre-transformed to the vehicle coordinate system.
 * This book uses an 18-dimensional ESKF, and the scalar type
 *  can be specified by S, with the default being double.
 * Variable order: p, v, R, bg, ba, grav, corresponding to the book.
 * @tparam S Precision of the state variables, can be float or double
 */
template <typename S = double>
class ESKF {
   public:
    /// 类型定义
    using SO3 = Sophus::SO3<S>;
    using VecT = Eigen::Matrix<S, 3, 1>;
    using Vec18T = Eigen::Matrix<S, 18, 1>;
    using Mat3T = Eigen::Matrix<S, 3, 3>;
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;
    using Mat18T = Eigen::Matrix<S, 18, 18>;
    using NavStateT =
        NavState<S>;  // Type for the overall nominal state variables

    using Options = ESKFOptions;

    /**
     * Initial bias set to zero.
     */
    ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

    /**
     * Set initial conditions.
     * @param options Configuration for noise terms
     * @param init_bg Initial gyroscope bias
     * @param init_ba Initial accelerometer bias
     * @param gravity Gravity
     */
    void SetInitialConditions(Options options, const VecT& init_bg,
                              const VecT& init_ba,
                              const VecT& gravity = VecT(0, 0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;
    }

    /// Propagate using IMU measurements
    bool Predict(const IMU& imu);

    /// Observe wheel speed measurements
    bool ObserveWheelSpeed(const Odom& odom);

    /// Observe GPS measurements
    bool ObserveGps(const GNSS& gnss);

    /**
     * Observe using SE3 measurements
     * @param pose Observed pose
     * @param trans_noise Translation noise
     * @param ang_noise Angle noise
     * @return
     */
    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1,
                    double ang_noise = 1.0 * math::kDEG2RAD);

    /// Get full state
    NavStateT GetNominalState() const {
        return NavStateT(current_time_, R_, p_, v_, bg_, ba_);
    }

    /// Get SE3 state
    SE3 GetNominalSE3() const { return SE3(R_, p_); }

    /// Set state X
    void SetX(const NavStated& x, const Vec3d& grav) {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = grav;
    }

    /// Set covariance
    void SetCov(const Mat18T& cov) { cov_ = cov; }

    /// Get gravity
    Vec3d GetGravity() const { return g_; }

   private:
    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev;  // * ev;
        double et2 = et;  // * et;
        double eg2 = eg;  // * eg;
        double ea2 = ea;  // * ea;

        // Set process noise
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2,
            ea2, ea2, ea2, 0, 0, 0;

        // Set measurement (odometry) noise
        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_.diagonal() << o2, o2, o2;

        // Set measurement (GNSS) noise
        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
    }

    /// Update nominal state variables and reset the error state.
    void UpdateAndReset() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));
        if (options_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }
        if (options_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }
        g_ += dx_.template block<3, 1>(15, 0);

        ProjectCov();

        dx_.setZero();
    }

    /// Projection of P matrix, referring to equation (3.63)
    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) =
            Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

   private:
    double current_time_ = 0.0;

    /// Nominal state
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    SO3 R_;
    VecT bg_ = VecT::Zero();
    VecT ba_ = VecT::Zero();
    VecT g_{0, 0, -9.8};

    /// Error state
    Vec18T dx_ = Vec18T::Zero();

    /// Covariance matrix
    Mat18T cov_ = Mat18T::Identity();

    /// Noise matrix
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    bool first_gnss_ = true;

    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);

    double dt = imu.timestamp_ - current_time_;

    // The time interval is incorrect, possibly the first IMU data with no
    // historical information.
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }

    // Propagation of the nominal state
    {
        p_ = p_ + (v_ * dt) + (0.5 * (R_ * (imu.acce_ - ba_) + g_) * dt * dt);
        v_ = v_ + (R_ * (imu.acce_ - ba_) * dt) + (g_ * dt);
        R_ = R_ * SO3::exp((imu.gyro_ - bg_) * dt);
        // , and the remaining state dimensions remain unchanged
    }

    // Propagation of the error state
    // - Compute the motion process Jacobian matrix F, as described in
    // (3.47)
    // - F is actually a sparse matrix, and it can be multiplied in a
    // non-matrix
    //   form. However, for the sake of teaching convenience, we use the
    //   matrix form here.
    Mat18T F = Mat18T::Identity();
    {
        // Position (p) with respect to velocity (v)
        F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;

        // Velocity (v) with respect to rotation (theta)
        F.template block<3, 3>(3, 6) =
            -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;

        // Velocity (v) with respect to accelerometer bias (ba)
        F.template block<3, 3>(3, 12) = -R_.matrix() * dt;

        // Velocity (v) with respect to gravity (g)
        F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;

        // Rotation (theta) with respect to rotation (theta)
        F.template block<3, 3>(6, 6) =
            SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();

        // Rotation (theta) with respect to gyroscope bias (bg)
        F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;
    }

    // mean and cov prediction
    {
        // This line is not necessary to calculate as dx_ should be zero
        // after resetting. Therefore, this step can be skipped. However, F
        // needs to participate in the covariance calculation, so it is
        // retained.
        dx_ = F * dx_;

        cov_ = F * cov_.eval() * F.transpose() + Q_;
    }

    current_time_ = imu.timestamp_;

    return true;
}

template <typename S>
bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);
    /// Odom correction and Jacobian
    /// Using a three-dimensional wheel speed observation,
    ///  H is a 3x18 matrix with mostly zeros.
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Mat3T::Identity();

    // 卡尔曼增益
    Eigen::Matrix<S, 18, 3> K =
        cov_ * H.transpose() *
        (H * cov_ * H.transpose() + odom_noise_).inverse();

    // velocity obs
    double velo_l = options_.wheel_radius_ * odom.left_pulse_ /
                    options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double velo_r = options_.wheel_radius_ * odom.right_pulse_ /
                    options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double average_vel = 0.5 * (velo_l + velo_r);

    VecT vel_odom(average_vel, 0.0, 0.0);
    VecT vel_world = R_ * vel_odom;

    dx_ = K * (vel_world - v_);

    // update cov
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();

    return true;
}

template <typename S>
bool ESKF<S>::ObserveGps(const GNSS& gnss) {
    /// Correction of GNSS observations
    assert(gnss.unix_time_ >= current_time_);

    if (first_gnss_) {
        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return true;
    }

    assert(gnss.heading_valid_);
    ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_,
               options_.gnss_ang_noise_);
    current_time_ = gnss.unix_time_;

    LOG(INFO) << "GNSS-based measurement updated.";

    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise,
                         double ang_noise) {
    /// Both rotation and translation are involved.
    /// In the observation state variables,
    ///  p and R are of size 6x18, while the rest are zero.
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // P部分
    H.template block<3, 3>(3, 6) = Mat3T::Identity();  // R部分（3.66)

    // Kalman gain and update process
    Vec6d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise,
        ang_noise;

    Mat6d V = noise_vec.asDiagonal();
    Eigen::Matrix<S, 18, 6> K =
        cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    // Update x and cov
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);  // 平移部分
    innov.template tail<3>() =
        (R_.inverse() * pose.so3()).log();  // 旋转部分(3.67)

    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();

    return true;
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_ESKF_HPP
