#pragma once

#include <chrono>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "courseProgress.hpp"
#include "environment.hpp"
#include "gate_search/gateStateMachine.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "rover.hpp"
#include "search/searchStateMachine.hpp"


using namespace rover_msgs;
using namespace std::chrono_literals;
using time_point = std::chrono::high_resolution_clock::time_point;

constexpr auto LOOP_DURATION = 0.01s;

class StateMachine : public std::enable_shared_from_this<StateMachine> {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    StateMachine(rapidjson::Document& config,
                 std::shared_ptr<Rover> rover, std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseProgress,
                 lcm::LCM& lcmObject);

    void run();

    void setSearcher(SearchType type);

    void setGateSearcher();

    std::shared_ptr<Environment> getEnv();

    std::shared_ptr<CourseProgress> getCourseState();

    std::shared_ptr<Rover> getRover();

    lcm::LCM& getLCM();

    double getDtSeconds();

    /*************************************************************************/
    /* Public Member Variables */
    /*************************************************************************/
    // Gate State Machine instance
    std::shared_ptr<GateStateMachine> mGateStateMachine;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishNavState() const;

    NavState executeOff();

    NavState executeDone();

    NavState executeDrive();

    std::string stringifyNavState() const;

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Configuration file for the rover.
    rapidjson::Document& mConfig;

    // Rover object to do basic rover operations in the state machine.
    std::shared_ptr<Rover> mRover;

    std::shared_ptr<Environment> mEnv;

    std::shared_ptr<CourseProgress> mCourseProgress;

    // Lcm object for sending and receiving messages.
    lcm::LCM& mLcmObject;

    // Search pointer to control search states
    std::shared_ptr<SearchStateMachine> mSearchStateMachine;

    // Avoidance pointer to control obstacle avoidance states
    std::shared_ptr<ObstacleAvoidanceStateMachine> mObstacleAvoidanceStateMachine;

    time_point mTimePoint, mPrevTimePoint;
};// StateMachine
