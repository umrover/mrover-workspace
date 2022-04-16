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

    void publishGatePath();

    Odometry getPointToFollow(Odometry curRoverLocation);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    const rapidjson::Document& mConfig;

    ProjectedPoints mProjectedPoints{};

    std::deque<Odometry> mPath;

    size_t mPathIndex = 0;

    void makeDualSegmentPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env);

    void makeSpiderPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment> const& env);

    bool isParallelToGate();
};

std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig);
