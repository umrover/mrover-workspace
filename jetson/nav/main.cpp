#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include "stateMachine.hpp"
#include "environment.hpp"
#include "courseState.hpp"

using namespace rover_msgs;
using namespace std;

rapidjson::Document readConfig(string const& name) {
    ifstream configFile;
    char* path_cstr = getenv("MROVER_CONFIG");
    if (!path_cstr) throw runtime_error("MROVER_CONFIG environment variable not set");
    string path = path_cstr;
    path += "/" + name;
    configFile.open(path);
    if (!configFile) throw runtime_error("Could not open config file at: " + path);
    rapidjson::Document document;
    rapidjson::IStreamWrapper isw(configFile);
    document.ParseStream(isw);
    return document;
}

// Runs the autonomous navigation of the rover.
int main() {
    lcm::LCM lcm;
    if (!lcm.good()) throw runtime_error("Cannot create LCM");

    auto env = make_shared<Environment>();
    auto courseState = make_shared<CourseProgress>();
    auto config = readConfig("nav/config.json");
    auto rover = make_shared<Rover>(config, lcm);
    auto stateMachine = make_shared<StateMachine>(config, rover, env, courseState, lcm);

    auto autonCallback = [rover](const lcm::ReceiveBuffer* recBuf, const string& channel, const AutonState* autonState) mutable {
        rover->setAutonState(*autonState);
    };
    lcm.subscribe("/auton", &decltype(autonCallback)::operator(), &autonCallback);

    auto courseCallback = [courseState](const lcm::ReceiveBuffer* recBuf, const string& channel, const Course* course) mutable {
        courseState->setCourse(*course);
    };
    lcm.subscribe("/course", &decltype(courseCallback)::operator(), &courseCallback);

    auto obstacleCallback = [env](const lcm::ReceiveBuffer* recBuf, const string& channel, const Obstacle* obstacle) mutable {
        env->setObstacle(*obstacle);
    };
    lcm.subscribe("/obstacle", &decltype(obstacleCallback)::operator(), &obstacleCallback);

    auto odometryCallback = [rover](const lcm::ReceiveBuffer* recBuf, const string& channel, const Odometry* odometry) mutable {
        rover->setOdometry(*odometry);
    };
    lcm.subscribe("/odometry", &decltype(odometryCallback)::operator(), &odometryCallback);

    auto targetCallback = [env](const lcm::ReceiveBuffer* recBuf, const string& channel, const TargetList* targetList) mutable {
        env->setTargets(*targetList);
    };
    lcm.subscribe("/target_list", &decltype(targetCallback)::operator(), &targetCallback);

    auto now = std::chrono::system_clock::now();
    while (lcm.handle() == 0) {
        stateMachine->run();
//        if (std::chrono::system_clock::now() - now > std::chrono::seconds(10)) {
//            break;
//        }
    }
    return 0;
} // main()
