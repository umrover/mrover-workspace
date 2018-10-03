#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/AutonState.hpp"
#include "layer1.hpp"
#include "layer2.hpp"
#include "calculations.hpp"

std::ostream & operator<< (std::ostream & out, const rover_msgs::Odometry & odom) {
    out << "odom set: " << odom.latitude_deg << "-" << odom.latitude_min << ", ";
    out << odom.longitude_deg << "-" << odom.longitude_min;
    return out;
}

std::ostream & operator<< (std::ostream & out, const rover_msgs::Waypoint & waypoint) {
    out << waypoint.odom;
    if(waypoint.search) out << " - search";
    else out << " - do not search";
    return out;
}

std::ostream & operator<< (std::ostream & out, const rover_msgs::Course & course) {
    for (int i = 0; i < course.num_waypoints; ++i) {
        out << course.waypoints[i] << "\n";
    }
    return out;
}

struct LcmHandlers {
    std::shared_ptr<Layer2> layer2;
    void odom(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::Odometry *odom) {
        layer2->cur_odom_.set(*odom);
       // std::cout << *odom << "\n";
    }

    void course(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::Course *course) {
        layer2->set_course(course);
        //std::cout << "course set: " << *course;
    }

    void auton(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::AutonState *state) {
        layer2->auton_state_.set(*state);
    }

    void ball(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::TennisBall *ball) {
        layer2->tennis_ball_.set(*ball);
    }

    void obstacle(
            const lcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const rover_msgs::Obstacle *obstacle) {
        layer2->obstacle_.set(*obstacle);
    }
};

int main(int argc, char **argv) {
    using namespace std::chrono_literals;
    lcm::LCM lcm_;
    if (!lcm_.good()) {
        std::cerr << "error: cannot create LCM" << std::endl;
        return 1;
    }

    auto layer2 = std::make_shared<Layer2>(std::ref(lcm_));
    LcmHandlers handlers{ layer2 };
    lcm_.subscribe("/odom", &LcmHandlers::odom, &handlers);
    lcm_.subscribe("/course", &LcmHandlers::course, &handlers);
    lcm_.subscribe("/auton", &LcmHandlers::auton, &handlers);
    lcm_.subscribe("/tennis_ball", &LcmHandlers::ball, &handlers);
    lcm_.subscribe("/obstacle", &LcmHandlers::obstacle, &handlers);

    std::thread l2_thread(&Layer2::run, layer2);

    while (lcm_.handle() == 0);
    return 0;
}
