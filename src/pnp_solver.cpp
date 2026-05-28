#include "auto_aim/pnp_solver.hpp"
#include <cmath>

ArmorPnPSolver::ArmorPnPSolver() {
    // ===== 相机内参 K =====
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        1567.7907457705167, 0.0,                662.3933648922284,
        0.0,                1564.9113082257936, 536.8662848443158,
        0.0,                0.0,                1.0);

    // ===== 畸变系数 D =====
    dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
        -0.0682737005569565,
         0.1983544402464456,
         0.0016855914452479342,
         0.0024125119646311016,
         0.0);

    initObjectPoints();
}

void ArmorPnPSolver::initObjectPoints() {
    // 小装甲板 3D 坐标（以装甲板中心为原点）
    // 顺序: 左上 -> 左下 -> 右下 -> 右上
    // 注意:  与图像点顺序保持严格对应！
    float half_w = SMALL_ARMOR_WIDTH  / 2.0f;
    float half_h = SMALL_ARMOR_HEIGHT / 2.0f;

    small_armor_points_ = {
        cv::Point3f(-half_w, -half_h, 0),   // 左上
        cv::Point3f(-half_w,  half_h, 0),   // 左下
        cv::Point3f( half_w,  half_h, 0),   // 右下
        cv::Point3f( half_w, -half_h, 0)    // 右上
    };

    // 大装甲板（预留，宽度 0.225m）
    float large_half_w = 0.225f / 2.0f;
    large_armor_points_ = {
        cv::Point3f(-large_half_w, -half_h, 0),
        cv::Point3f(-large_half_w,  half_h, 0),
        cv::Point3f( large_half_w,  half_h, 0),
        cv::Point3f( large_half_w, -half_h, 0)
    };
}

PnPResult ArmorPnPSolver::solve(const std::vector<cv::Point2f>& image_points,
                                ArmorType type) {
    PnPResult result;
    result.success = false;

    if (image_points.size() != 4) {
        std::cerr << "[PnP] image_points 数量必须为 4，当前为 "
                  << image_points.size() << std::endl;
        return result;
    }

    // 选择对应的 3D 模型点
    const auto& object_points = (type == ArmorType::SMALL)
                                ? small_armor_points_
                                : large_armor_points_;

    // ===== 调用 solvePnP =====
    // SOLVEPNP_IPPE 专为共面 4 点设计，速度快、精度高
    bool ok = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix_,
        dist_coeffs_,
        result.rvec,
        result.tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (!ok) {
        std::cerr << "[PnP] solvePnP 求解失败" << std::endl;
        return result;
    }

    // ===== 提取距离 =====
    double tx = result.tvec.at<double>(0);
    double ty = result.tvec.at<double>(1);
    double tz = result.tvec.at<double>(2);
    result.distance = std::sqrt(tx * tx + ty * ty + tz * tz);

    // ===== 计算 yaw / pitch（相机坐标系下，对准目标的角度） =====
    // yaw: 水平偏角 (绕 y 轴)，目标在相机正前方时为 0
    // pitch: 俯仰角 (绕 x 轴)，向上为正
    result.yaw   = std::atan2(tx, tz) * 180.0 / CV_PI;
    result.pitch = std::atan2(-ty, std::sqrt(tx * tx + tz * tz)) * 180.0 / CV_PI;

    // ===== 从旋转向量提取装甲板自身姿态 (roll) =====
    cv::Mat R;
    cv::Rodrigues(result.rvec, R);

    // 欧拉角分解 (ZYX 顺序)
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                          R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    double roll;
    if (!singular) {
        roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    } else {
        roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    }
    result.roll = roll * 180.0 / CV_PI;

    result.success = true;
    return result;
}