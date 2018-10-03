#include <mbed.h>
#include "math_3d.hpp"

int main() {
    Math::Vector3f v = {0, 1, 0};

    while (1) {
        printf("v = {%.2f, %.2f, %.2f}\n", v.x, v.y, v.z);
        wait(2.0);
    }
}
