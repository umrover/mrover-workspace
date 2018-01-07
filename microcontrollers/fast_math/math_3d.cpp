#include <math.h>
#include "math_3d.hpp"

// "Fast Inverse Square Root" from Wikipedia
static float invsqrt(float x) {
    float xhalf = 0.5f * x;
    union {
        float x;
        int i;
    } u;
    u.x = x;
    u.i = 0x5f3759df - (u.i >> 1);
    // The next line can be repeated any number of times to increase accuracy
    u.x = u.x * (1.5f - xhalf * u.x * u.x);
    return u.x;
}

void Math::normalize_vec(Math::Vector3f &v) {
    float recip_norm = invsqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    v.x *= recip_norm;
    v.y *= recip_norm;
    v.z *= recip_norm;
}

void Math::normalize_quat(Math::Quaternion &q) {
    float recip_norm = invsqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
    q.q0 *= recip_norm;
    q.q1 *= recip_norm;
    q.q2 *= recip_norm;
    q.q3 *= recip_norm;
}

Math::Rot3f Math::quat_to_rpy(Math::Quaternion q) {
    Math::Rot3f rot;
    rot.roll = atan2f(q.q0*q.q1 + q.q2*q.q3, 0.5f - q.q1*q.q1 - q.q2*q.q2);
    rot.pitch = asinf(-2.0f * (q.q1*q.q3 - q.q0*q.q2));
    rot.yaw = atan2f(q.q1*q.q2 + q.q0*q.q3, 0.5f - q.q2*q.q2 - q.q3*q.q3);
    return rot;
}

Math::Vector3f Math::rotate_vec(Math::Quaternion q, Math::Vector3f v) {
    Math::Vector3f res;
    res.x = (1.0f - 2.0f*(q.q2*q.q2 + q.q3*q.q3))
          + 2.0f*(q.q0*q.q3 + q.q1*q.q2)
          + 2.0f*(q.q1*q.q3 - q.q0*q.q2);
    res.y = 2.0f*(q.q1*q.q2 - q.q0*q.q3)
          + (1.0f - 2.0f*(q.q1*q.q1 + q.q3*q.q3))
          + 2.0f*(q.q0*q.q2 + q.q2*q.q3);
    res.z = 2.0f*(q.q0*q.q2 + q.q1*q.q3)
          + 2.0f*(q.q2*q.q3 - q.q0*q.q1)
          + (1.0f - 2.0f*(q.q1*q.q1 + q.q2*q.q2));

    return res;
}

float Math::dot(Math::Vector3f u, Math::Vector3f v) {
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

Math::Vector3f Math::cross(Math::Vector3f u, Math::Vector3f v) {
    Math::Vector3f res;
    res.x = u.y*v.z - u.z*v.y;
    res.y = -(u.x*v.z - u.z*v.x);
    res.z = u.x*v.y - u.y*v.x;
    return res;
}
