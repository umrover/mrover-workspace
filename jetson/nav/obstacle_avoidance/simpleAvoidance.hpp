#ifndef SIMPLE_AVOIDANCE_HPP
#define SIMPLE_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    SimpleAvoidance( std::weak_ptr<StateMachine> roverStateMachine, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig );

    ~SimpleAvoidance();

    NavState executeTurnAroundObs( std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig );


    NavState executeDriveAroundObs( std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig );


    Odometry createAvoidancePoint( std::shared_ptr<Rover> rover, const double distance );
};

#endif //SIMPLE_AVOIDANCE_HPP
