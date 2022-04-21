#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
// #include <filesystem> does not compile with ubuntu18

#include <lcm/lcm-cpp.hpp>
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include "stateMachine.hpp"
#include "environment.hpp"
#include "courseProgress.hpp"


using namespace rover_msgs;

rapidjson::Document readConfig() {
    std::ifstream configFile;
    char* mrover_config = getenv("MROVER_CONFIG");
    std::string path = std::string(mrover_config) + "/config_nav/config.json";
//    std::filesystem::path path;
//    if (mrover_config) {
//        path = std::filesystem::path{mrover_config} / "config_nav" / "config.json";
//    } else {
//        path = std::filesystem::current_path() / "config" / "nav" / "config.json";
//    }
    configFile.open(path);
//    if (!configFile) throw std::runtime_error("Could not open config file at: " + path.string());
    if (!configFile) throw std::runtime_error("Could not open config file at: " + path);
    rapidjson::Document document;
    rapidjson::IStreamWrapper isw(configFile);
    document.ParseStream(isw);
    return document;
}

// Runs the autonomous navigation of the rover.
int main() {
    lcm::LCM lcm;
    if (!lcm.good()) throw std::runtime_error("Cannot create LCM");


    auto courseProgress = std::make_shared<CourseProgress>();
    auto config = readConfig();
    auto env = std::make_shared<Environment>(config);
    auto rover = std::make_shared<Rover>(config, lcm);
    auto stateMachine = std::make_shared<StateMachine>(config, rover, env, courseProgress, lcm);

    auto autonCallback = [rover](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const AutonState* autonState) mutable {
        rover->setAutonState(*autonState);
    };
    lcm.subscribe("/auton", &decltype(autonCallback)::operator(), &autonCallback)->setQueueCapacity(3);

    auto courseCallback = [courseProgress](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const Course* course) mutable {
        courseProgress->setCourse(*course);
    };
    lcm.subscribe("/course", &decltype(courseCallback)::operator(), &courseCallback)->setQueueCapacity(3);

    auto obstacleCallback = [env](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const Obstacle* obstacle) mutable {
        env->setObstacle(*obstacle);
    };
    lcm.subscribe("/obstacle", &decltype(obstacleCallback)::operator(), &obstacleCallback)->setQueueCapacity(3);

    auto odometryCallback = [rover](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const Odometry* odometry) mutable {
        rover->setOdometry(*odometry);
    };
    lcm.subscribe("/odometry", &decltype(odometryCallback)::operator(), &odometryCallback)->setQueueCapacity(3);

    auto targetCallback = [env](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const TargetList* targetList) mutable {
        env->setTargets(*targetList);
    };
    lcm.subscribe("/target_list", &decltype(targetCallback)::operator(), &targetCallback)->setQueueCapacity(3);
    while (lcm.handle() == 0) {
        //while (lcm.handleTimeout(0) > 0) {}
        stateMachine->run();
    }
    return 0;
} // main()
