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
    Waypoint lastKnownRightPost;

    /* saved last known location of second tag of a gate */
    Waypoint lastKnownLeftPost;

    // Queue of search points
    deque<Odometry> mGateSearchPoints;

private:
    /*************************************************************************/
    /* Private Member Functions */

    //TODO: make sure order of declarations is consistent.
    /*************************************************************************/
    NavState executeGateSpin();

    NavState executeGateSearchGimbal();

    NavState executeGateSpinWait();

    NavState executeGateTurn();

    NavState executeGateDrive();

    NavState executeGateTurnToCentPoint();

    NavState executeGateDriveToCentPoint();

    NavState executeGateFace();

    NavState executeGateDriveThrough();

    NavState executeGateTurnToFarPost();

    NavState executeGateDriveToFarPost();

    NavState executeGateTurnToGateCenter();

   

    void updatePost2Info();

    void calcCenterPoint();

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Pointer to rover State Machine to access member functions
    StateMachine* mRoverStateMachine;

    // Reference to config variables
    const rapidjson::Document& mRoverConfig;

    // Points in frnot of center of gate
    Odometry centerPoint1;
    Odometry centerPoint2;

    // Check if we drove through the correct direction
    bool isCorrectGateDir;

    // Minor adjustment to gate targeting, due to issue
    // of driving through a post when driving through the
    // wrong direction
    double gateAdjustmentDist;

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/
    // Pointer to rover object
    Rover* mPhoebe;
};

GateStateMachine* GateFactory( StateMachine* stateMachine, Rover* phoebe, const rapidjson::Document& roverConfig );

#endif //GATE_STATE_MACHINE_HPP
