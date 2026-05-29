#pragma once
#include <opencv2/opencv.hpp>

namespace auto_aim {

// ---------- 灯条 ----------
struct Light {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length;
    float width;
    float angle;
};

// ---------- 装甲板类型 ----------
enum class ArmorType { SMALL, LARGE };

// ---------- 装甲板 ----------
struct Armor {
    Light left, right;
    cv::Point2f pts[4];      // 左上, 左下, 右下, 右上 (供PnP用)
    cv::Point2f center;
    double score = 0.0;
};

// ---------- PnP 解算结果 ----------
struct PoseResult {
    cv::Vec3d tvec;          // 相机系下的平移 (x, y, z) m
    cv::Vec3d rvec;          // 旋转向量
    double yaw;              // 偏航角 (deg)
    double pitch;            // 俯仰角 (deg)
    double distance;         // 距离 (m)
};

// ---------- 相机参数 ----------
struct CameraParam {
    cv::Mat K;               // 3x3 内参
    cv::Mat dist;            // 1x5 畸变系数
};



} // namespace auto_aim