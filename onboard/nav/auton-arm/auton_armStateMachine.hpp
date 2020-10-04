#ifndef AUTONARM_STATE_MACHINE_HPP
#define AUTONARM_STATE_MACHINE_HPP

#include <lcm/lcm-cpp.hpp>
#include "rapidjson/document.h"
#include "rover.hpp"

using namespace std;
using namespace rover_msgs;

class AutonArmStateMachine
{
public:
    AutonArmStateMachine( lcm::LCM& lcmObject );

    ~AutonArmStateMachine();

    void updateRoverStatus( AutonState autonState );

    void updateRoverStatus( TargetList targetList );


private:
    bool isRoverReady() const;

    NavState executeOff();

    NavState executeDone();
}

#endif