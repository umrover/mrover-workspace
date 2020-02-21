#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <lcm/lcm-cpp.hpp>
#include "rapidjson/document.h"
#include "rover.hpp"
#include "search/searchStateMachine.hpp"
#include "gate_search/gateStateMachine.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"

using namespace std;
using namespace rover_msgs;

// This class implements the logic for the state machine for the
// autonomous navigation of the rover.
class StateMachine
{
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    StateMachine( lcm::LCM& lcmObject );

    ~StateMachine();

    void run( );

    void updateRoverStatus( AutonState autonState );

    void updateRoverStatus( Bearing bearing );

    void updateRoverStatus( Course course );

    void updateRoverStatus( Obstacle obstacle );

    void updateRoverStatus( Odometry odometry );

    void updateRoverStatus( TargetList targetList );

    void updateRoverStatus( RadioSignalStrength radioSignalStrength );

    void updateCompletedPoints( );

    void updateObstacleAngle( double bearing );

    void updateObstacleDistance( double distance );

    void updateObstacleElements( double bearing, double distance );

    void updateRepeaterComplete( );

    void setSearcher(SearchType type);

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/
    // Gate State Machine instance
    GateStateMachine* mGateStateMachine;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    bool isRoverReady() const;

    void publishNavState() const;

    NavState executeOff();

    NavState executeDone();

    NavState executeTurn();

    NavState executeDrive();

    NavState executeRepeaterDropWait();

    NavState executeSearch();

    void initializeSearch();

    bool addFourPointsToSearch();

    string stringifyNavState() const;

    bool isObstacleDetected() const;

    double getOptimalAvoidanceAngle() const;

    double getOptimalAvoidanceDistance() const;

    bool isWaypointReachable( double distance );

    bool isAddRepeaterDropPoint() const;

    void addRepeaterDropPoint();

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Rover object to do basic rover operations in the state machine.
    Rover* mPhoebe;

    // RoverStatus object for updating the rover's status.
    Rover::RoverStatus mNewRoverStatus;

    // Lcm object for sending and recieving messages.
    lcm::LCM& mLcmObject;

    // Configuration file for the rover.
    rapidjson::Document mRoverConfig;

    // Number of waypoints in course.
    unsigned mTotalWaypoints;

    // Number of waypoints completed.
    unsigned mCompletedWaypoints;

    // Bool of whether radio repeater has been dropped.
    bool mRepeaterDropComplete = false;

    // Indicates if the state changed on a given iteration of run.
    bool mStateChanged;

    // Search pointer to control search states
    SearchStateMachine* mSearchStateMachine;

    // Avoidance pointer to control obstacle avoidance states
    ObstacleAvoidanceStateMachine* mObstacleAvoidanceStateMachine;

}; // StateMachine

#endif // STATE_MACHINE_HPP
