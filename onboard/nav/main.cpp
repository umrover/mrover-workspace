#include <iostream>
#include <thread>
#include <chrono>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "layers.hpp"

struct LcmHandlers {
    void odom(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::Odometry *odom) {
        std::cout << "currently at " << odom->latitude_deg << " deg "
                  << odom->latitude_min << " min latitude by "
                  << odom->longitude_deg << " deg " << odom->longitude_min
                  << " min longitude bearing " << odom->bearing_deg
                  << " degrees True" << std::endl;
    }

    void goal(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::Odometry *odom) {
        std::cout << "going to " << odom->latitude_deg << " deg "
                  << odom->latitude_min << " min latitude by "
                  << odom->longitude_deg << " deg " << odom->longitude_min
                  << " min longitude." << std::endl;
    }
};

int main(int argc, char **argv) {
    using namespace std::chrono_literals;
    Auton::System sys;
    lcm::LCM lcm_;
    if (!lcm_.good()) {
        std::cerr << "error: cannot create LCM" << std::endl;
        return 1;
    }
    LcmHandlers handlers;
    lcm_.subscribe("/odom", &LcmHandlers::odom, &handlers);
    lcm_.subscribe("/goal", &LcmHandlers::goal, &handlers);

    // TODO create Auton::Layer objects and attach them to the System
    while (lcm_.handle() == 0);
    return 0;
}
