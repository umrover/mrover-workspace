#include <cstdio>
#include <thread>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "layers.hpp"

int main(int argc, char **argv) {
    using namespace std::chrono_literals;
    Auton::System sys;
    // TODO create more Auton::Layer objects
    while (true) {
        std::this_thread::sleep_for(2s);
    }
    return 0;
}
