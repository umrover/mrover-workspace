#include <memory>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "stateMachine.hpp"
#include "environment.hpp"
#include "course_state.hpp"

using namespace rover_msgs;
using namespace std;

// Runs the autonomous navigation of the rover.
int main() {
    lcm::LCM lcmObject;
    if (!lcmObject.good()) {
        throw runtime_error("Cannot create LCM");
    }

    auto env = make_shared<Environment>();
    auto courseState = make_shared<CourseState>();
    auto stateMachine = make_shared<StateMachine>(env, lcmObject);

    auto autonCallback = [stateMachine](const lcm::ReceiveBuffer* recBuf, const string& channel, const AutonState* autonState) mutable {
        stateMachine->updateRoverStatus(*autonState);
    };
    lcmObject.subscribe("/auton", &decltype(autonCallback)::operator(), &autonCallback);

    auto courseCallback = [courseState](const lcm::ReceiveBuffer* recBuf, const string& channel, const Course* course) mutable {
        courseState->update(*course);
    };
    lcmObject.subscribe("/course", &decltype(courseCallback)::operator(), &courseCallback);

    auto obstacleCallback = [env](const lcm::ReceiveBuffer* recBuf, const string& channel, const Obstacle* obstacle) mutable {
        env->setObstacle(*obstacle);
    };
    lcmObject.subscribe("/obstacle", &decltype(obstacleCallback)::operator(), &obstacleCallback);

    auto odometryCallback = [stateMachine](const lcm::ReceiveBuffer* recBuf, const string& channel, const Odometry* odometry) mutable {

    };
    lcmObject.subscribe("/odometry", &decltype(odometryCallback)::operator(), &odometryCallback);

    auto targetCallback = [env](const lcm::ReceiveBuffer* recBuf, const string& channel, const TargetList* targetList) mutable {
        env->setTargets(*targetList);
    };
    lcmObject.subscribe("/target_list", &decltype(targetCallback)::operator(), &targetCallback);

    while (lcmObject.handle() == 0) {
        stateMachine->run();
    }
    return 0;
} // main()
