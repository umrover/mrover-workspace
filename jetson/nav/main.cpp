#include <memory>
#include <fstream>
#include <iostream>
#include <filesystem>

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
    std::filesystem::path path;
    if (mrover_config) {
        path = std::filesystem::path{mrover_config} / "config_nav" / "config.json";
    } else {
        path = std::filesystem::current_path() / "config" / "nav" / "config.json";
    }
    configFile.open(path);
    if (!configFile) throw std::runtime_error("Could not open config file at: " + path.string());
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
    lcm.subscribe("/auton", &decltype(autonCallback)::operator(), &autonCallback);

    auto courseCallback = [courseProgress](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const Course* course) mutable {
        courseProgress->setCourse(*course);
    };
    lcm.subscribe("/course", &decltype(courseCallback)::operator(), &courseCallback);

    auto obstacleCallback = [env](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const Obstacle* obstacle) mutable {
        env->setObstacle(*obstacle);
    };
    lcm.subscribe("/obstacle", &decltype(obstacleCallback)::operator(), &obstacleCallback);

    auto odometryCallback = [rover](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const Odometry* odometry) mutable {
        rover->setOdometry(*odometry);
    };
    lcm.subscribe("/odometry", &decltype(odometryCallback)::operator(), &odometryCallback);

    auto targetCallback = [env](const lcm::ReceiveBuffer* recBuf, const std::string& channel, const TargetList* targetList) mutable {
        env->setTargets(*targetList);
    };
    lcm.subscribe("/target_list", &decltype(targetCallback)::operator(), &targetCallback);

    while (lcm.handle() == 0) {
        stateMachine->run();
    }
    return 0;
} // main()
