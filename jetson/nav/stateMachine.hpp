#pragma once

#include <memory>
#include <lcm/lcm-cpp.hpp>
#include "rapidjson/document.h"
#include "rover.hpp"
#include "search/searchStateMachine.hpp"
#include "gate_search/gateStateMachine.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "environment.hpp"
#include "course_state.hpp"

using namespace rover_msgs;

// This class implements the logic for the state machine for the
// autonomous navigation of the rover.
class StateMachine : std::enable_shared_from_this<StateMachine> {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    StateMachine(std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseState, lcm::LCM& lcmObject);

    ~StateMachine();

    void run();

    void updateRoverStatus(AutonState autonState);

    void updateRoverStatus(const Course& course);

    void updateRoverStatus(Odometry odometry);

    void updateCompletedPoints();

    void updateObstacleAngle(double bearing, double rightBearing);

    void updateObstacleDistance(double distance);

    void updateObstacleElements(double bearing, double rightBearing, double distance);

    void setSearcher(SearchType type, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig);

    std::shared_ptr<Environment> getEnv();

    std::shared_ptr<CourseProgress> getCourseState();

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/
    // Gate State Machine instance
    std::shared_ptr<GateStateMachine> mGateStateMachine;

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

    string stringifyNavState() const;

    double getOptimalAvoidanceDistance() const;

    bool isWaypointReachable(double distance);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Rover object to do basic rover operations in the state machine.
    std::shared_ptr<Rover> mRover;

    std::shared_ptr<Environment> mEnv;

    std::shared_ptr<CourseProgress> mCourseProgress;

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

    // Indicates if the state changed on a given iteration of run.
    bool mStateChanged;

    // Search pointer to control search states
    std::shared_ptr<SearchStateMachine> mSearchStateMachine;

    // Avoidance pointer to control obstacle avoidance states
    std::shared_ptr<ObstacleAvoidanceStateMachine> mObstacleAvoidanceStateMachine;
}; // StateMachine
