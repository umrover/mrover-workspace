#include <math.h>
#include "madgwick.hpp"

static inline void rotate_scale_vec(Math::Quaternion q, Math::Vector3f _2d, Math::Vector3f &r) {
    r.x = _2d.x * (0.5f - q.q2*q.q2 - q.q3*q.q3)
        + _2d.y * (q.q0*q.q3 + q.q1*q.q2)
        + _2d.z * (q.q1*q.q3 - q.q0*q.q2);
    r.y = _2d.x * (q.q1*q.q2 - q.q0*q.q3)
        + _2d.y * (0.5f - q.q1*q.q1 - q.q3*q.q3)
        + _2d.z * (q.q0*q.q1 + q.q2*q.q3);
    r.z = _2d.x * (q.q0*q.q2 + q.q1*q.q3)
        + _2d.y * (q.q2*q.q3 - q.q0*q.q1)
        + _2d.z * (0.5f - q.q1*q.q1 - q.q2*q.q2);
}

static inline void compensate_gyro_drift(
        Math::Quaternion q, Math::Quaternion s,
        float dt, float zeta,
        Math::Vector3f &w_b,
        Math::Vector3f &g) {
    float w_err_x = 2.0f*q.q0*s.q1 - 2.0f*q.q1*s.q0 - 2.0f*q.q2*s.q3 + 2.0f*q.q3*s.q2;
    float w_err_y = 2.0f*q.q0*s.q2 + 2.0f*q.q1*s.q3 - 2.0f*q.q2*s.q0 - 2.0f*q.q3*s.q1;
    float w_err_z = 2.0f*q.q0*s.q3 - 2.0f*q.q1*s.q2 + 2.0f*q.q2*s.q1 - 2.0f*q.q3*s.q0;

    w_b.x += w_err_x * dt * zeta;
    w_b.y += w_err_y * dt * zeta;
    w_b.z += w_err_z * dt * zeta;

    g.x -= w_b.x;
    g.y -= w_b.y;
    g.z -= w_b.z;
}

static inline void orientation_change_from_gyro(
        Math::Quaternion q, Math::Vector3f g, Math::Quaternion &q_dot) {
    q_dot.q0 = 0.5f * (-q.q1*g.x - q.q2*g.y - q.q3*g.z);
    q_dot.q1 = 0.5f * (q.q0*g.x + q.q2*g.z - q.q3*g.y);
    q_dot.q2 = 0.5f * (q.q0*g.y - q.q1*g.z + q.q3*g.x);
    q_dot.q3 = 0.5f * (q.q0*g.z + q.q1*g.y - q.q2*g.x);
}

static inline void gradient_descent(
        Math::Quaternion q,
        Math::Vector3f _2d,
        Math::Vector3f m,
        Math::Quaternion &s) {
    Math::Vector3f f;
    rotate_scale_vec(q, _2d, f);

    f.x -= m.x;
    f.y -= m.y;
    f.z -= m.z;

    s.q0 += (_2d.y*q.q3 - _2d.z*q.q2)*f.x
          + (-_2d.x*q.q3 + _2d.z*q.q1)*f.y
          + (_2d.x*q.q2 - _2d.y*q.q1)*f.z;
    s.q1 += (_2d.y*q.q2 + _2d.z*q.q3)*f.x
          + (_2d.x*q.q2 - 2.0f*_2d.y*q.q1 + _2d.z*q.q0)*f.y
          + (_2d.x*q.q3 - _2d.y*q.q0 - 2.0f*_2d.z*q.q1)*f.z;
    s.q2 += (-2.0f*_2d.x*q.q2 + _2d.y*q.q1 - _2d.z*q.q0)*f.x
          + (_2d.x*q.q1 + _2d.z*q.q3)*f.y
          + (_2d.x*q.q0 + _2d.y*q.q3 - 2.0f*_2d.z*q.q2)*f.z;
}

static inline void compensate_magnetic_distortion(
        Math::Quaternion q,
        Math::Vector3f m,
        float &_2bxy, float &_2bz) {
    Math::Quaternion q_temp = {q.q0, -q.q1, -q.q2, -q.q3};
    Math::Vector3f h;
    rotate_scale_vec(q_temp, m, h);

    _2bxy = 4.0f*sqrtf(h.x*h.x + h.y*h.y);
    _2bz = 4.0f*h.z;
}

Madgwick::Madgwick(float gain, float zeta) :
    gain_(gain),
    zeta_(zeta)
{
    memset(&this->quat_, 0, sizeof(this->quat_));
    this->quat_.q0 = 1.0;
}

Math::Quaternion Madgwick::update(Math::Vector3f g, Math::Vector3f a, Math::Vector3f m, float dt) {
    Math::Quaternion s;
    Math::Quaternion q_dot;
    float _2bz, _2bxy;

    Math::Vector3f gravity_2d_vec = {0.0f, 0.0f, 2.0f};

    // TODO add magnetometer NaN handling

    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {
        Math::normalize_vec(a);
        Math::normalize_vec(m);
        compensate_magnetic_distortion(quat_, m, _2bxy, _2bz);

        memset(&s, 0, sizeof(s));
        gradient_descent(quat_, gravity_2d_vec, a, s);
        Math::Vector3f enu_2d_vec = {0.0f, _2bxy, _2bz};
        gradient_descent(quat_, enu_2d_vec, m, s);

        Math::normalize_quat(s);

        compensate_gyro_drift(quat_, s, dt, zeta_, omega_, g);
        orientation_change_from_gyro(quat_, g, q_dot);

        q_dot.q0 -= gain_*s.q0;
        q_dot.q1 -= gain_*s.q1;
        q_dot.q2 -= gain_*s.q2;
        q_dot.q3 -= gain_*s.q3;
    } else {
        orientation_change_from_gyro(quat_, g, q_dot);
    }

    quat_.q0 += q_dot.q0 * dt;
    quat_.q1 += q_dot.q1 * dt;
    quat_.q2 += q_dot.q2 * dt;
    quat_.q3 += q_dot.q3 * dt;

    Math::normalize_quat(quat_);

    return quat_;
}
