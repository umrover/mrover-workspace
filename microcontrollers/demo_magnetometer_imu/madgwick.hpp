#pragma once

#include "imu.hpp"
#include "math_3d.hpp"

class Madgwick {
    public:
        Madgwick(float gain, float zeta);

        Math::Quaternion update(
                Math::Vector3f g,
                Math::Vector3f a,
                Math::Vector3f m,
                float dt);

    private:
        float gain_;
        float zeta_;

        Math::Quaternion quat_;
        Math::Vector3f omega_;
};
