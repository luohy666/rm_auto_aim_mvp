#include "auto_aim/detector.hpp"
#include "auto_aim/pnp_solver.hpp"
#include "auto_aim/coord_transform.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace fs = std::filesystem;

int main() {
    // ---------- 1. 路径 ----------
    fs::path root = fs::current_path();
    while (!fs::exists(root / "CMakeLists.txt") && root.has_parent_path())
        root = root.parent_path();

    fs::path inPath   = root / "data/videos/armor.mp4";
    fs::path outDir   = root / "data/results";
    fs::create_directories(outDir);
    fs::path outVideo = outDir / "result.mp4";
    fs::path outCsv   = outDir / "pose.csv";

    // ---------- 2. 视频 ----------
    cv::VideoCapture cap(inPath.string());
    if (!cap.isOpened()) { std::cerr << "无法打开: " << inPath << "\n"; return -1; }

    int    w   = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int    h   = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;

    cv::VideoWriter writer(outVideo.string(),
    cv::VideoWriter::fourcc('m','p','4','v'), fps, {w, h});

    // ---------- 3. 检测器 + PnP ----------
    auto_aim::Detector detector;
    auto_aim::ArmorPnPSolver     pnp;  

    // ---------- 3.1 Camera -> Gimbal / Gun 坐标变换 ----------
    auto_aim::Transform camera_to_gimbal(
    auto_aim::Vec3(0.02, 0.0, 0.0),
    auto_aim::Quaternion::fromEuler(0.0, 0.0, 0.0));


    // ---------- 4. CSV 表头 ----------
    std::ofstream csv(outCsv);
    csv << "frame,armor_id,"
           "rvec_x,rvec_y,rvec_z,"
           "tvec_x,tvec_y,tvec_z,"
           "distance_m,yaw,pitch,roll\n";
    csv << std::fixed << std::setprecision(4);

    // ---------- 5. 主循环 ----------
    cv::Mat frame;
    int frameIdx = 0;
    while (cap.read(frame)) {
        auto armors = detector.detect(frame);

        for (size_t i = 0; i < armors.size(); ++i) {
            // ★ 直接用 detector 已经排好序的 4 个角点（左上→左下→右下→右上）
            std::vector<cv::Point2f> image_points(armors[i].pts, armors[i].pts + 4);

            auto_aim::PnPResult r = pnp.solve(image_points, auto_aim::ArmorType::SMALL);
            if (!r.success) continue;

            // ---------- Camera 坐标系 -> Gimbal / Gun 坐标系 ----------
           // PnP 得到的 tvec 是目标在 Camera 坐标系下的位置
           auto_aim::Vec3 target_camera(
             r.tvec.at<double>(0),
             r.tvec.at<double>(1),
             r.tvec.at<double>(2)
            );

            // 转换到 Gimbal / Gun 坐标系
         auto_aim::Vec3 target_gimbal =
         auto_aim::transformPoint(target_camera, camera_to_gimbal);

           // 在 Gimbal / Gun 坐标系下重新计算枪口 yaw / pitch
        double gun_yaw = 0.0;
        double gun_pitch = 0.0;
        double gun_dist = 0.0;

         auto_aim::calcYawPitchFromPoint(
         target_gimbal,
         gun_yaw,
         gun_pitch,
         gun_dist
        );


            // (a) 写 CSV —— 后续分析用
            csv << frameIdx << "," << i << ","
                << r.rvec.at<double>(0) << "," << r.rvec.at<double>(1) << "," << r.rvec.at<double>(2) << ","
                << r.tvec.at<double>(0) << "," << r.tvec.at<double>(1) << "," << r.tvec.at<double>(2) << ","
                << r.distance << "," << r.yaw << "," << r.pitch << "," << r.roll << "\n";

            // (b) 实时画在画面上 —— 跑的时候肉眼看
            cv::Point2f c = armors[i].center;
            cv::putText(frame, cv::format("dist=%.2fm", r.distance),
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0, 255, 255), 2);

            cv::putText(frame, cv::format("yaw=%.1f deg", r.yaw),
            cv::Point(20, 55),
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0, 255, 255), 2);
            
            cv::putText(frame, cv::format("pit=%.1f deg", r.pitch),
            cv::Point(20, 80),
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0, 255, 255), 2);

        }

        auto_aim::Detector::draw(frame, armors);
        writer.write(frame);
        cv::imshow("armor", frame);
        if (cv::waitKey(1) == 27) break;
        ++frameIdx;
    }

    csv.close();
    std::cout << "完成 " << frameIdx << " 帧\n视频: " << outVideo
              << "\nCSV : " << outCsv << "\n";
    return 0;
}