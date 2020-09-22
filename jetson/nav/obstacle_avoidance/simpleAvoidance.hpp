#ifndef SIMPLE_AVOIDANCE_HPP
#define SIMPLE_AVOIDANCE_HPP

#include "obstacleAvoidanceStateMachine.hpp"

// This class implements the logic for the simple obstacle avoidance algorithm.
// If an obstacle is seen, create an avoidance point using trigonometry with the angle turned and 
// distance from obstacle.
class SimpleAvoidance : public ObstacleAvoidanceStateMachine 
{
public:
    SimpleAvoidance( StateMachine* roverStateMachine );

    ~SimpleAvoidance();

    NavState executeTurnAroundObs( Rover* phoebe, const rapidjson::Document& roverConfig );

    NavState executeDriveAroundObs( Rover* phoebe );

    Odometry createAvoidancePoint( Rover* phoebe, const double distance );
};

#endif //SIMPLE_AVOIDANCE_HPP
