#include "auto_aim/coord_transform.hpp"

namespace auto_aim {

Vec3::Vec3(double x, double y, double z)
    : x(x), y(y), z(z) {}

Vec3 Vec3::operator+(const Vec3& v) const {
    return Vec3(x + v.x, y + v.y, z + v.z);
}

Quaternion::Quaternion(double w, double x, double y, double z)
    : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

void Quaternion::normalize() {
    double n = std::sqrt(w * w + x * x + y * y + z * z);

    if (n < 1e-12) {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
        return;
    }

    w /= n;
    x /= n;
    y /= n;
    z /= n;
}

Quaternion Quaternion::inverse() const {
    return Quaternion(w, -x, -y, -z);
}

Vec3 Quaternion::rotate(const Vec3& v) const {
    Quaternion q_norm = *this;
    q_norm.normalize();

    Quaternion p(0.0, v.x, v.y, v.z);
    Quaternion r = q_norm * p * q_norm.inverse();

    return Vec3(r.x, r.y, r.z);
}

Quaternion Quaternion::fromEuler(double yaw, double pitch, double roll) {
    double cy = std::cos(yaw / 2.0);
    double sy = std::sin(yaw / 2.0);

    double cp = std::cos(pitch / 2.0);
    double sp = std::sin(pitch / 2.0);

    double cr = std::cos(roll / 2.0);
    double sr = std::sin(roll / 2.0);

    Quaternion q(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );

    q.normalize();
    return q;
}

void Quaternion::toEuler(double& yaw, double& pitch, double& roll) {
    normalize();

    roll = std::atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y)
    );

    double s = 2.0 * (w * y - z * x);

    if (std::fabs(s) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, s);
    } else {
        pitch = std::asin(s);
    }

    yaw = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    );
}

Pose::Pose(double x, double y, double z,
           double yaw, double pitch, double roll)
    : p(x, y, z), yaw(yaw), pitch(pitch), roll(roll) {}

Quaternion Pose::q() const {
    return Quaternion::fromEuler(yaw, pitch, roll);
}

Transform::Transform(Vec3 t, Quaternion q)
    : t(t), q(q) {}

Pose transformPose(const Pose& pose, const Transform& tf) {
    Vec3 newP = tf.q.rotate(pose.p) + tf.t;

    Quaternion newQ = tf.q * pose.q();
    newQ.normalize();

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    newQ.toEuler(yaw, pitch, roll);

    return Pose(newP.x, newP.y, newP.z, yaw, pitch, roll);
}

Vec3 transformPoint(const Vec3& point, const Transform& tf) {
    return tf.q.rotate(point) + tf.t;
}

void calcYawPitchFromPoint(
    const Vec3& point,
    double& yaw_deg,
    double& pitch_deg,
    double& distance
) {
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;

    distance = std::sqrt(x * x + y * y + z * z);

    constexpr double RAD_TO_DEG = 180.0 / M_PI;

    yaw_deg = std::atan2(x, z) * RAD_TO_DEG;
    pitch_deg = std::atan2(-y, std::sqrt(x * x + z * z)) * RAD_TO_DEG;
}

}  // namespace auto_aim