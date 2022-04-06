#pragma once

#include <deque>
#include <memory>

#include "../rover.hpp"
#include "filter.hpp"
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

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    const rapidjson::Document& mRoverConfig;

    Filter<double> mTargetFilter;
};

std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig);
