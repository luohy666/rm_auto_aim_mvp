#pragma once
#include "auto_aim/types.hpp"
#include <vector>

namespace auto_aim {

class Detector {
public:
    std::vector<Armor> detect(const cv::Mat& frame);
    static void draw(cv::Mat& frame, const std::vector<Armor>& armors);

private:
    cv::Mat getRedMask(const cv::Mat& frame);
    bool isLight(const std::vector<cv::Point>& contour, Light& light);
    bool matchArmor(const Light& a, const Light& b, Armor& armor, const std::vector<Light>& lights);   // ← 多了一个参数
};

} // namespace auto_aim