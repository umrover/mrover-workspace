#ifndef SIMPLE_AVOIDANCE_HPP
#define SIMPLE_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine
{
public:
    SimpleAvoidance( StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig );

    ~SimpleAvoidance();

    NavState executeTurnAroundObs( Rover* rover, const rapidjson::Document& roverConfig );


    NavState executeDriveAroundObs( Rover* rover, const rapidjson::Document& roverConfig );


    Odometry createAvoidancePoint( Rover* rover, const double distance );
};

#endif //SIMPLE_AVOIDANCE_HPP
