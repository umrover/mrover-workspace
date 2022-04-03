#pragma once

#include "environment.hpp"
#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine {
public:
    SimpleAvoidance(weak_ptr<StateMachine> roverStateMachine, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig);

    ~SimpleAvoidance() override;

    NavState executeTurnAroundObs(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) override;

    NavState executeDriveAroundObs(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) override;

    Odometry createAvoidancePoint(shared_ptr<Rover> rover, double distance) override;
};
