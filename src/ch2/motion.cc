//
// Created by xiang on 22-12-29.
// Refactored by Giseop Kim on 23-06-05
//

#include <chrono>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// This section demonstrates a vehicle performing circular motion.
/// The angular velocity and linear velocity of the vehicle can be set in the flags.

DEFINE_double(angular_velocity, 10.0, "Angular velocity (in degrees)");
DEFINE_double(linear_velocity, 5.0, "Linear velocity of the vehicle in m/s");
DEFINE_bool(use_quaternion, false, "Whether to use quaternion calculation");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// Visualization
    sad::ui::PangolinWindow ui;
    if (!ui.Init()) {
        return -1;
    }

    const double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // Angular velocity in radians
    const Vec3d omega(0, 0, angular_velocity_rad);                                     // Angular velocity vector
    const Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // Body-frame velocity
    const double dt = 0.5;                                                             // Time for each update

    SE3 pose;  // Pose represented by TWB (B: body, W: world)
    while (!ui.ShouldQuit()) {
        // Update ego position global
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt;

        // Update ego orientation global
        if (FLAGS_use_quaternion) {
            auto dq = Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            auto q = pose.unit_quaternion() * dq;
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }

        LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        const std::chrono::microseconds sleepTime(static_cast<long long>(dt * 1e6));
        std::this_thread::sleep_for(sleepTime);
    }

    ui.Quit();
    return 0;
}