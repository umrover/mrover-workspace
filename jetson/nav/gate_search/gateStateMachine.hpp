#pragma once

#include <deque>
#include <memory>

#include "../rover.hpp"
#include "rover_msgs/Odometry.hpp"


using namespace rover_msgs;

class StateMachine;

class GateStateMachine {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig);

    virtual ~GateStateMachine();

    NavState run();

    virtual void initializeSearch() = 0;

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/

    // Pointer to rover State Machine to access member functions
    std::weak_ptr<StateMachine> mStateMachine;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    NavState executeGateSpin();

    NavState executeGateSpinWait();

    NavState executeGateTurn();

    NavState executeGateDrive();

    NavState executeGateTurnToCentPoint();

    NavState executeGateDriveToCentPoint();

    NavState executeGateFace();

    NavState executeGateTurnToFarPost();

    NavState executeGateDriveToFarPost();

    NavState executeGateTurnToGateCenter();;

    NavState executeGateDriveThrough();

    void updatePost2Info();

    void calcCenterPoint();

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    const rapidjson::Document& mRoverConfig;
};

std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig);
