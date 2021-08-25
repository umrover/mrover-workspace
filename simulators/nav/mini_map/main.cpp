#include <lcm/lcm-cpp.hpp>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/Joystick.hpp"

using namespace rover_msgs;

#include <iostream>

AutonState autonState;
Odometry odometry;
Joystick joystick;

class LcmHandlers {
    void autonStateHandler(
    const lcm::ReceiveBuffer* receiveBuffer,
    const string& channel,
    const AutonState autonStateIn
    )
    {
    autonState = autonStateIn;
    }

    void odometryHandler(
    const lcm::ReceiveBuffer* receiveBuffer,
    const string& channel,
    const Obstacle odometryIn
    )
    {
    odometry = odometryIn;
    }

    void joystickHandler(
    const lcm::ReceiveBuffer* receiveBuffer,
    const string& channel,
    const Obstacle obstacleIn
    )
    {
    joystick = joystickIn;
    }


}

int main() {
    lcm::LCM lcm_;
    if( !lcm_.good() )
    {
        cerr << "Error: cannot create LCM\n";
        return 1;
    }
    lcm_.subscribe( "/auton", &LcmHandlers::autonStateHandler, &lcmHandlers );
    lcm_.subscribe( "/odometry", &LcmHandlers::odometryHandler, &lcmHandlers );
    lcm_.subscribe( "/joystick", &LcmHandlers::joystickHandler, &lcmHandlers );
}
