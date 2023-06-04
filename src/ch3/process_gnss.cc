//
// Created by xiang on 2022/1/4.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iomanip>
#include <memory>

#include "common/gnss.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

DEFINE_string(txt_path, "../data/ch3/10.txt", "Data file path");

// The following parameters are only for the data provided in this book
DEFINE_double(antenna_angle, 12.06,
              "RTK antenna installation angle (in degrees)");
DEFINE_double(antenna_pox_x, -0.17, "RTK antenna installation offset in X");
DEFINE_double(antenna_pox_y, -0.20, "RTK antenna installation offset in Y");
DEFINE_bool(with_ui, true, "Whether to display the graphical interface");

/**
 * This program demonstrates how to process GNSS data.
 * We convert the raw GNSS readings into a 6-degree-of-freedom Pose that can be
 * further processed.
 * The process involves three steps: UTM conversion, RTK
 * antenna extrinsics, and coordinate system conversion.
 * We save the results in a file and then visualize them using a Python script.
 */

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(fLS::FLAGS_txt_path);

    std::ofstream fout("../data/ch3/gnss_output.txt");
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_result = [](std::ofstream& fout, double timestamp,
                          const SE3& pose) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z()
                 << " ";
        };

        fout << std::setprecision(18) << timestamp << " "
             << std::setprecision(9);
        save_vec3(fout, pose.translation());
        save_quat(fout, pose.unit_quaternion());
        fout << std::endl;
    };

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();
    io.SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
          sad::GNSS gnss_out = gnss;
          if (sad::ConvertGps2UTM(gnss_out, antenna_pos, FLAGS_antenna_angle)) {
              if (!first_gnss_set) {
                  origin = gnss_out.utm_pose_.translation();
                  first_gnss_set = true;
              }

              gnss_out.utm_pose_.translation() -= origin;
              save_result(fout, gnss_out.unix_time_, gnss_out.utm_pose_);
              ui->UpdateNavState(
                  sad::NavStated(gnss_out.unix_time_, gnss_out.utm_pose_.so3(),
                                 gnss_out.utm_pose_.translation()));
              usleep(1e3);
          }
      }).Go();

    if (ui) {
        while (!ui->ShouldQuit()) {
            usleep(1e5);
        }
        ui->Quit();
    }

    return 0;
}