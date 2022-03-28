#pragma once

#include <memory>
#include <lcm/lcm-cpp.hpp>
#include "rover.hpp"
#include "search/searchStateMachine.hpp"
#include "gate_search/gateStateMachine.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "environment.hpp"
#include "courseState.hpp"

using namespace std;
using namespace rover_msgs;

// This class implements the logic for the state machine for the
// autonomous navigation of the rover.
class StateMachine : public enable_shared_from_this<StateMachine> {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    StateMachine(rapidjson::Document& config,
                 shared_ptr<Rover> rover, shared_ptr<Environment> env, shared_ptr<CourseProgress> courseProgress,
                 lcm::LCM& lcmObject);

    void run();

    void updateObstacleElements(double leftBearing, double rightBearing, double distance);

    void updateObstacleDistance(double distance);

    void setSearcher(SearchType type, const shared_ptr<Rover>& rover, const rapidjson::Document& roverConfig);

    shared_ptr<Environment> getEnv();

    shared_ptr<CourseProgress> getCourseState();

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/
    // Gate State Machine instance
    shared_ptr<GateStateMachine> mGateStateMachine;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishNavState() const;

    NavState executeOff();

    NavState executeDone();

    NavState executeTurn();

    NavState executeDrive();

    string stringifyNavState() const;

    double getOptimalAvoidanceDistance() const;

    bool isWaypointReachable(double distance);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Rover object to do basic rover operations in the state machine.
    shared_ptr<Rover> mRover;

    shared_ptr<Environment> mEnv;

    shared_ptr<CourseProgress> mCourseProgress;

    // Lcm object for sending and receiving messages.
    lcm::LCM& mLcmObject;

    // Configuration file for the rover.
    rapidjson::Document& mConfig;

    // Number of waypoints in course.
    int32_t mTotalWaypoints = 0;

    // Search pointer to control search states
    shared_ptr<SearchStateMachine> mSearchStateMachine;

    // Avoidance pointer to control obstacle avoidance states
    shared_ptr<ObstacleAvoidanceStateMachine> mObstacleAvoidanceStateMachine;
}; // StateMachine
