#ifndef GATE_STATE_MACHINE_HPP
#define GATE_STATE_MACHINE_HPP

#include <deque>

#include "../rover.hpp"
#include "rover_msgs/Odometry.hpp"
// #include "../gate_search/gateStateMachine.hpp"

class StateMachine;

class GateStateMachine
{
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    GateStateMachine( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig );

    virtual ~GateStateMachine();

    NavState run();

    virtual void initializeSearch() = 0;

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/
    /* saved last known location of first tag of a gate */
    Waypoint lastKnownPost1;

    /* saved last known location of second tag of a gate */
    Waypoint lastKnownPost2;

    // Queue of search points
    deque<Odometry> mGateSearchPoints;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    NavState executeGateSpin();

    NavState executeGateSpinWait();

    void updatePost2Info();

    void calcCenterPoint();

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Pointer to rover State Machine to access member functions
    StateMachine* mRoverStateMachine;

    // Reference to config variables
    const rapidjson::Document& mRoverConfig;

    // Point in frnot of center of gate
    Odometry centerPoint;

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/
    // Pointer to rover object
    Rover* mPhoebe;
};

GateStateMachine* GateFactory( StateMachine* stateMachine, Rover* phoebe, const rapidjson::Document& roverConfig );

#endif //GATE_STATE_MACHINE_HPP
