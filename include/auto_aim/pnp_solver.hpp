#ifndef ARMOR_PNP_HPP
#define ARMOR_PNP_HPP

#include <opencv2/opencv.hpp>
#include <vector>

// 装甲板物理尺寸（单位：米）
constexpr float SMALL_ARMOR_WIDTH  = 0.135f;  // 水平方向两灯条中心距离
constexpr float SMALL_ARMOR_HEIGHT = 0.055f;  // 灯条高度

// 装甲板类型（后续可扩展大装甲板）
enum class ArmorType {
    SMALL,
    LARGE
};

// PnP 解算结果
struct PnPResult {
    cv::Mat rvec;          // 旋转向量 (3x1)
    cv::Mat tvec;          // 平移向量 (3x1)，单位：米
    double distance;       // 装甲板到相机的距离 (m)
    double yaw;            // 偏航角 (度)，绕 y 轴
    double pitch;          // 俯仰角 (度)，绕 x 轴
    double roll;           // 翻滚角 (度)，绕 z 轴
    bool   success;        // 解算是否成功
};

class ArmorPnPSolver {
public:
    ArmorPnPSolver();

    /**
     * @brief 求解装甲板位姿
     * @param image_points 灯条 4 个角点的像素坐标
     *                     顺序: 左上 -> 左下 -> 右下 -> 右上
     * @param type 装甲板类型
     * @return PnPResult 解算结果
     */
    PnPResult solve(const std::vector<cv::Point2f>& image_points,
                    ArmorType type = ArmorType::SMALL);

private:
    cv::Mat camera_matrix_;       // 相机内参矩阵 K (3x3)
    cv::Mat dist_coeffs_;         // 畸变系数 D (1x5)

    std::vector<cv::Point3f> small_armor_points_;  // 小装甲板 3D 点
    std::vector<cv::Point3f> large_armor_points_;  // 大装甲板 3D 点（预留）

    void initObjectPoints();
};

#endif // ARMOR_PNP_HPP