#pragma once

// TODO write to make use of ARM vectorization intrinsics

namespace Math {
    struct Quaternion {
        float q0;
        float q1;
        float q2;
        float q3;
    };

    struct Vector3f {
        float x;
        float y;
        float z;
    };

    struct Rot3f {
        float roll;
        float pitch;
        float yaw;
    };

    void normalize_vec(Vector3f &v);
    void normalize_quat(Quaternion &q);

    Rot3f quat_to_rpy(Quaternion q);

    Vector3f rotate_vec(Quaternion q, Vector3f v);
    float dot(Vector3f u, Vector3f v);
    Vector3f cross(Vector3f u, Vector3f v);
}
