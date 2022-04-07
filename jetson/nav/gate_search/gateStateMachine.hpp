#pragma once

#include <deque>
#include <memory>

#include <eigen3/Eigen/Core>

#include "../rover.hpp"
#include "../filter.hpp"
#include "rover_msgs/Odometry.hpp"

using namespace rover_msgs;

class StateMachine;

class GateStateMachine {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig);

    ~GateStateMachine();

    NavState run();

    void updateGateTraversalPath();

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

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    const rapidjson::Document& mRoverConfig;

    std::deque<Odometry> mPath;
};

std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig);
