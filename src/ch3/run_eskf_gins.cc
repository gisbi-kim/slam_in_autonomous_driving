//
// Created by xiang on 2021/11/11.
//

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "../data/ch3/10.txt", "Data file path");

// The following parameters are only for the data provided in this book
DEFINE_double(antenna_angle, 12.06,
              "RTK antenna installation angle (in degrees)");
DEFINE_double(antenna_pox_x, -0.17, "RTK antenna installation offset in X");
DEFINE_double(antenna_pox_y, -0.20, "RTK antenna installation offset in Y");
DEFINE_bool(with_ui, true, "Whether to display the graphical interface");
DEFINE_bool(with_odom, true, "Whether to include odometry information");

/**
 * This program demonstrates the use of RTK+IMU for integrated navigation.
 */

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    sad::StaticIMUInit imu_init;
    sad::ESKFD eskf;

    sad::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) {
        fout << v[0] << " " << v[1] << " " << v[2] << " ";
    };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    auto save_result = [&save_vec3, &save_quat](
                           std::ofstream& fout,
                           const sad::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ << " "
             << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };

    std::ofstream fout("../data/ch3/gins.txt");
    bool imu_inited = false, gnss_inited = false;

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          if (!imu_inited) {
              // Read initial biases and set up ESKF.
              sad::ESKFOptions options;

              // Noise estimated by the initializer.
              //   options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              //   options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(),
                                        imu_init.GetInitBa(),
                                        imu_init.GetGravity());

              imu_inited = true;
              return;
          }

          /// Wait for valid RTK data.
          if (!gnss_inited) {
              return;
          }

          /// Once GNSS is also received, start the prediction.
          eskf.Predict(imu);

          /// The predict function will update the ESKF,
          ///   so data can be sent at\  this point.
          auto state = eskf.GetNominalState();
          if (ui) {
              ui->UpdateNavState(state);
          }

          /// Record data for plotting purposes.
          save_result(fout, state);

          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
            if (!imu_inited) {
                return;
            }

            sad::GNSS gnss_convert = gnss;
            if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos,
                                     FLAGS_antenna_angle) ||
                !gnss_convert.heading_valid_) {
                return;
            }

            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            gnss_convert.utm_pose_.translation() -= origin;

            // RTK heading must be valid in order to integrate with ESKF.
            eskf.ObserveGps(gnss_convert);

            auto state = eskf.GetNominalState();
            if (ui) {
                ui->UpdateNavState(state);
            }
            save_result(fout, state);

            gnss_inited = true;
        })
        .SetOdomProcessFunc([&](const sad::Odom& odom) {
            /// Odom processing function, Odom is only used for initialization
            /// in this chapter
            imu_init.AddOdom(odom);
            if (FLAGS_with_odom && imu_inited && gnss_inited) {
                eskf.ObserveWheelSpeed(odom);
            }
        })
        .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e5);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}