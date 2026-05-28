#include "auto_aim/detector.hpp"
#include <algorithm>
#include <cmath>

namespace auto_aim {

// ---------- 工具 ----------
static float normAngle(float a) {
    while (a >= 90.f)  a -= 180.f;
    while (a < -90.f)  a += 180.f;
    return a;
}

// ---------- 红色掩码 ----------
cv::Mat Detector::getRedMask(const cv::Mat& frame) {
    std::vector<cv::Mat> ch;
    cv::split(frame, ch);
    cv::Mat diff = ch[2] - ch[0];          // R - B
    cv::Mat mask;
    cv::threshold(diff, mask, 50, 255, cv::THRESH_BINARY);
    cv::dilate(mask, mask,
        cv::getStructuringElement(cv::MORPH_RECT, {3, 5}));
    return mask;
}

// ---------- 灯条筛选 ----------
bool Detector::isLight(const std::vector<cv::Point>& contour, Light& light) {
    if (contour.size() < 5) return false;

    cv::RotatedRect r = cv::minAreaRect(contour);
    float len = std::max(r.size.width, r.size.height);
    float wid = std::min(r.size.width, r.size.height);
    if (wid < 1.f || len < 5.f) return false;

    float ratio = len / wid;
    if (ratio < 1.5f || ratio > 15.f) return false;

    float angle = (r.size.width > r.size.height) ? r.angle + 90.f : r.angle;
    angle = normAngle(angle);
    if (std::abs(angle) > 45.f) return false;

    light.rect   = r;
    light.center = r.center;
    light.length = len;
    light.width  = wid;
    light.angle  = angle;
    return true;
}

// ---------- 灯条配对 ----------
bool Detector::matchArmor(const Light& a, const Light& b, Armor& armor,
                          const std::vector<Light>& lights) {
    const Light& L = (a.center.x < b.center.x) ? a : b;
    const Light& R = (a.center.x < b.center.x) ? b : a;

    float avgLen = (L.length + R.length) * 0.5f;
    float dx = R.center.x - L.center.x;
    float dy = R.center.y - L.center.y;        // 带正负号
    float absDy = std::abs(dy);

    if (dx <= 0.f) return false;

    // 1. 长度比
    float lenRatio = std::max(L.length, R.length) / std::min(L.length, R.length);
    if (lenRatio > 1.6f) return false;

    // 2. 【加强】角度差：从 12° 收到 8°
    float angleDiff = std::abs(L.angle - R.angle);
    if (angleDiff > 8.f) return false;

    // 3. 【新增】两灯条朝向一致（同号或都接近 0）
    //    避免一个 +10° 一个 -10° 这种"八字形"
    if (L.angle * R.angle < -25.f) return false;   // 异号且乘积绝对值大 → 拒绝

    // 4. 中心连线倾角
    float lineAngle = std::atan2(dy, dx) * 180.f / CV_PI;   // 带正负号
    if (std::abs(lineAngle) > 18.f) return false;

    // 5. 【新增·关键】灯条倾角 与 中心连线倾角 应该"垂直"
    //    几何关系：灯条向右倾 +α°，则连线应向左倾 -α°，即两者符号相反、大小相近
    //    判据：灯条平均角度 + 连线角度 ≈ 0（差 < 15°）
    float avgLightAngle = (L.angle + R.angle) * 0.5f;
    if (std::abs(avgLightAngle + lineAngle) > 15.f) return false;

    // 6. 间距比例
    float xRatio = dx / avgLen;
    if (xRatio < 1.0f || xRatio > 4.5f) return false;

    // 7. 垂直错位
    if (absDy > avgLen * 0.7f) return false;

    // 8. 中间不能有其他灯条
    float midY = (L.center.y + R.center.y) * 0.5f;
    for (const Light& m : lights) {
        if (&m == &L || &m == &R) continue;
        if (m.center.x > L.center.x + 2 && m.center.x < R.center.x - 2 &&
            std::abs(m.center.y - midY) < avgLen * 1.5f) {
            return false;
        }
    }

    // 取四个角点
    cv::Point2f pL[4], pR[4];
    L.rect.points(pL);
    R.rect.points(pR);
    std::sort(pL, pL + 4, [](auto& p, auto& q){ return p.y < q.y; });
    std::sort(pR, pR + 4, [](auto& p, auto& q){ return p.y < q.y; });
    cv::Point2f lt = (pL[0] + pL[1]) * 0.5f;
    cv::Point2f lb = (pL[2] + pL[3]) * 0.5f;
    cv::Point2f rt = (pR[0] + pR[1]) * 0.5f;
    cv::Point2f rb = (pR[2] + pR[3]) * 0.5f;

    armor.left   = L;
    armor.right  = R;
    armor.pts[0] = lt;   // 左上
    armor.pts[1] = lb;   // 左下
    armor.pts[2] = rb;   // 右下
    armor.pts[3] = rt;   // 右上
    armor.center = (L.center + R.center) * 0.5f;

    // 评分（越小越好）
    armor.score = angleDiff * 1.5
                + std::abs(L.length - R.length) * 2.0
                + std::abs(xRatio - 2.0f) * 5.0
                + absDy * 0.5
                + std::abs(avgLightAngle + lineAngle) * 1.0;
    return true;
}

// ---------- 主流程 ----------
std::vector<Armor> Detector::detect(const cv::Mat& frame) {
    cv::Mat mask = getRedMask(frame);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Light> lights;
    for (auto& c : contours) {
        Light l;
        if (isLight(c, l)) lights.push_back(l);
    }

    struct Cand { Armor armor; int li, ri; };
    std::vector<Cand> candidates;
    for (size_t i = 0; i < lights.size(); ++i) {
        for (size_t j = i + 1; j < lights.size(); ++j) {
            Armor ar;
            if (matchArmor(lights[i], lights[j], ar, lights)) {
                candidates.push_back({ar, (int)i, (int)j});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Cand& a, const Cand& b){ return a.armor.score < b.armor.score; });

    std::vector<Armor> result;
    std::vector<bool> used(lights.size(), false);
    for (const auto& c : candidates) {
        if (used[c.li] || used[c.ri]) continue;
        used[c.li] = used[c.ri] = true;
        result.push_back(c.armor);
    }
    return result;
}

// ---------- 可视化 ----------
void Detector::draw(cv::Mat& frame, const std::vector<Armor>& armors) {
    // 角点颜色（4 个不同颜色，便于区分顺序）
    const cv::Scalar cornerColors[4] = {
        cv::Scalar(0, 255, 255),    // 0 左上 - 黄
        cv::Scalar(0, 255, 0),      // 1 左下 - 绿
        cv::Scalar(255, 255, 0),    // 2 右下 - 青
        cv::Scalar(255, 0, 255),    // 3 右上 - 品红
    };

    for (const auto& a : armors) {
        // 1. 画红色矩形框
        for (int i = 0; i < 4; ++i) {
            cv::line(frame, a.pts[i], a.pts[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);
        }

        // 2. 画 4 个角点（实心圆 + 编号）
        for (int i = 0; i < 4; ++i) {
            cv::circle(frame, a.pts[i], 5, cornerColors[i], -1);
            cv::circle(frame, a.pts[i], 6, cv::Scalar(255, 255, 255), 1);  // 白色描边
            cv::putText(frame, std::to_string(i),
                        a.pts[i] + cv::Point2f(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cornerColors[i], 1);
        }

        // 3. 画中心点 + "armor" 文字
        cv::circle(frame, a.center, 3, cv::Scalar(0, 0, 255), -1);
    }
}

} // namespace auto_aim