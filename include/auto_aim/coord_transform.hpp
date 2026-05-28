#pragma once

#include <cmath>

namespace auto_aim {

struct Vec3 {
    double x, y, z;

    Vec3(double x = 0.0, double y = 0.0, double z = 0.0);

    Vec3 operator+(const Vec3& v) const;
};

struct Quaternion {
    double w, x, y, z;

    Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0);

    Quaternion operator*(const Quaternion& q) const;

    void normalize();

    Quaternion inverse() const;

    Vec3 rotate(const Vec3& v) const;

    static Quaternion fromEuler(double yaw, double pitch, double roll);

    void toEuler(double& yaw, double& pitch, double& roll);
};

struct Pose {
    Vec3 p;
    double yaw, pitch, roll;

    Pose(double x = 0.0, double y = 0.0, double z = 0.0,
         double yaw = 0.0, double pitch = 0.0, double roll = 0.0);

    Quaternion q() const;
};

struct Transform {
    Vec3 t;
    Quaternion q;

    Transform(Vec3 t = Vec3(), Quaternion q = Quaternion());
};

Pose transformPose(const Pose& pose, const Transform& tf);

Vec3 transformPoint(const Vec3& point, const Transform& tf);

void calcYawPitchFromPoint(
    const Vec3& point,
    double& yaw_deg,
    double& pitch_deg,
    double& distance
);

}  // namespace auto_aim