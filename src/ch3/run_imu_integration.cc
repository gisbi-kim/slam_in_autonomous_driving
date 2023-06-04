//
// Created by xiang on 2021/11/5.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iomanip>

#include "ch3/imu_integration.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "../data/ch3/10.txt", "Data file path");
DEFINE_bool(with_ui, true, "Whether to display the graphical interface");

/**
 * This program demonstrates how to perform direct integration on an IMU.
 * The program requires input text files located in the data/ch3/ directory,
 * and it outputs the state to data/ch3/state.txt. The vehicle's motion can
 * also be observed in the UI.
 */

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(FLAGS_imu_txt_path);

    // In this experiment, we assume that the biases are known.
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);
    Vec3d gravity(0, 0, -9.8);

    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果
    auto save_result = [](std::ofstream& fout, double timestamp,
                          const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z()
                 << " ";
        };

        fout << std::setprecision(18) << timestamp << " "
             << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    std::ofstream fout("../data/ch3/state.txt");
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout,
                          &ui](const sad::IMU& imu) {
          imu_integ.AddIMU(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(),
                      imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      }).Go();

    // If visualization is enabled, wait for the interface to exit
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}