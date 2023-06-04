//
// Created by xiang on 22-12-29.
// Refactored by Giseop Kim on 23-06-05
//

#include <chrono>
#include <thread>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// This section demonstrates a vehicle performing circular motion.
/// The angular velocity and linear velocity of the vehicle can be set in the
/// flags.

DEFINE_double(angular_velocity, 10.0, "Angular velocity (in degrees)");
DEFINE_double(linear_velocity, 5.0, "Linear velocity of the vehicle in m/s");
DEFINE_double(dt_control_period_sec, 0.05,
              "Linear velocity of the vehicle in sec");

struct ControlInput {
    Vec3d v_body{0.0, 0.0, 0.0};
    Vec3d w_body{0.0, 0.0, 0.0};  // omega
    double dt = 0.001;
};

class SE3Propagator {
   public:
    SE3Propagator() = default;
    SE3Propagator(const SE3 pose_in) { pose = pose_in; }

    std::pair<SE3, Vec3d> propagte(const ControlInput u) {
        auto [pose_propagated, v_world] = _propagte_pos(pose, u.v_body, u.dt);
        pose_propagated = _propagte_rot(pose_propagated, u.w_body, u.dt);

        SetPose(pose_propagated);

        return {pose_propagated, v_world};
    }

    void SetPose(const SE3 pose_in) { pose = pose_in; }

    // Update ego position global (world)
    std::pair<SE3, Vec3d> _propagte_pos(const SE3 pose_in, const Vec3d v_body,
                                        const double dt) const {
        SE3 pose{pose_in};
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt;
        return {pose, v_world};
    }

    // Update ego orientation global (world)
    SE3 _propagte_rot(const SE3 pose_in, const Vec3d omega,
                      const double dt) const {
        auto dq = [](const Vec3d wt) {
            return Quatd(1.0, 0.5 * wt[0], 0.5 * wt[1], 0.5 * wt[2]);
        };

        SE3 pose{pose_in};
        if (use_quaternion) {
            // updated on the quat space
            const auto& q_world_current = pose.unit_quaternion();
            auto q_world_propagated = q_world_current * dq(omega * dt);
            q_world_propagated.normalize();

            // just re-representing: world quat to world SO3
            pose.so3() = SO3(q_world_propagated);
        } else {
            // updated on the SO3 group space (dRot is calculated at its tangent
            // space)
            auto dR = SO3::exp(omega * dt);
            pose.so3() = pose.so3() * dR;
        }
        return pose;
    }

   private:
    SE3 pose;

    const bool use_quaternion = false;
};

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

    SE3Propagator pose_propagator;
    while (!ui.ShouldQuit()) {
        // This turn's control (in this case, constant velocity model)
        ControlInput u{
            .v_body = Vec3d(FLAGS_linear_velocity, 0.0, 0.0),  // v
            .w_body =
                Vec3d(0, 0, FLAGS_angular_velocity * sad::math::kDEG2RAD),  // w
            .dt = FLAGS_dt_control_period_sec};

        // Update the pose using the single control input
        auto [pose_world, v_world] = pose_propagator.propagte(u);

        // Redraw and print
        ui.UpdateNavState(sad::NavStated(0, pose_world, v_world));
        LOG(INFO) << "pose: " << pose_world.translation().transpose();

        // Sleep during a single control tick
        const std::chrono::microseconds sleepTime(
            static_cast<long long>(FLAGS_dt_control_period_sec * 1e6));
        std::this_thread::sleep_for(sleepTime);
    }

    ui.Quit();
    return 0;
}