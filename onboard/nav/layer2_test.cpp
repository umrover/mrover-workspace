//layer2 Test

#include <iostream>
#include <thread>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Course.hpp"
#include "calculations.hpp"
#include "layer2.hpp"
#include <vector>

std::vector<double> bearing_degs = {0.0000001, 0.0001, 120.23214, 130.3801, 148.2321};

int main() {
    using namespace std::chrono_literals;
    std::cout << "Testing layer2..." << std::endl;
   
    /*Auton::System sys;
    lcm::LCM lcm_;
    Layer2 layer2(sys, lcm_);*/

    return 0;   //std::cout << layer2.state << std::endl;
}
