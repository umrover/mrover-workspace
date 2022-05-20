#pragma once

#include <memory>
#include "rover.hpp"
#include "environment.hpp"


using namespace rover_msgs;

class StateMachine;

// This class is the representation of different 
// obstacle avoidance algorithms
enum class ObstacleAvoidanceAlgorithm {
    SimpleAvoidance
};

// This class is the base class for the logic of the obstacle avoidance state machine 
class ObstacleAvoidanceStateMachine {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    ObstacleAvoidanceStateMachine(std::weak_ptr<StateMachine> sm, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig);

    virtual ~ObstacleAvoidanceStateMachine() = default;

    void updateObstacleAngle(double bearing, double rightBearing);

    void updateObstacleDistance(double distance);

    void updateObstacleElements(double bearing, double rightBearing, double distance);

    NavState run();

    bool isTargetDetected();

    virtual Odometry createAvoidancePoint(std::shared_ptr<Rover> rover, double distance) = 0;

    virtual NavState executeTurnAroundObs(std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) = 0;

    virtual NavState
    executeDriveAroundObs(std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) = 0;

protected:
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/

    // Pointer to rover State Machine to access member functions
    std::weak_ptr<StateMachine> mStateMachine;

    // Odometry point used when avoiding obstacles.
    Odometry mObstacleAvoidancePoint;

    // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleAngle;

    // Initial angle to go around obstacle upon detection.
    double mOriginalObstacleDistance;

    // bool for consecutive obstacle detections
    bool mJustDetectedObstacle;

    // Last obstacle angle for consecutive angles
    double mLastObstacleAngle;

    // Pointer to rover object
    std::shared_ptr<Rover> mRover;

private:
    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // Reference to config variables
    const rapidjson::Document& mConfig;

};

// Creates an ObstacleAvoidanceStateMachine object based on the inputted obstacle 
// avoidance algorithm. This allows for an an ease of transition between obstacle 
// avoidance algorithms
std::shared_ptr<ObstacleAvoidanceStateMachine> ObstacleAvoiderFactory(
        std::weak_ptr<StateMachine> roverStateMachine, ObstacleAvoidanceAlgorithm algorithm, std::shared_ptr<Rover> rover,
        const rapidjson::Document& roverConfig
);
