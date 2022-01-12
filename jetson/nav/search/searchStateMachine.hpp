#ifndef SEARCH_STATE_MACHINE_HPP
#define SEARCH_STATE_MACHINE_HPP

#include "rover.hpp"
#include "utilities.hpp"

class StateMachine;

// This class is the representation of different
// search algorithms
enum class SearchType
{
    SPIRALOUT,
    LAWNMOWER,
    SPIRALIN
};

class SearchStateMachine {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    SearchStateMachine( StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig );

    virtual ~SearchStateMachine() {}

    NavState run();

    bool targetReachable( Rover* rover, double distance, double bearing );

    virtual void initializeSearch( Rover* rover, const rapidjson::Document& roverConfig, double pathWidth ) = 0; // TODO

protected:
    /*************************************************************************/
    /* Protected Member Functions */
    /*************************************************************************/

    void insertIntermediatePoints();

    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/

    // Pointer to rover State Machine to access member functions
    StateMachine* roverStateMachine;

    // Vector of search point multipliers used as a base for the search points.
    vector< pair<short, short> > mSearchPointMultipliers;

    // Queue of search points.
    deque<Odometry> mSearchPoints;

    // Pointer to rover object
    Rover* mRover;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    NavState executeSearchSpin();

    NavState executeRoverWait();

    NavState executeSearchTurn();

    NavState executeSearchDrive();

    NavState executeTurnToTarget();

    NavState executeDriveToTarget();

    void updateTargetAngle( double bearing );

    void updateTurnToTargetRoverAngle( double bearing );

    void updateTargetDetectionElements( double target_bearing, double rover_bearing );

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // Last known angle to turn to target.
    double mTargetAngle;

    // Last known angle of rover from turn to target.
    double mTurnToTargetRoverAngle;

    // Reference to config variables
    const rapidjson::Document& mRoverConfig;

};

// Creates an ObstacleAvoidanceStateMachine object based on the inputted obstacle
// avoidance algorithm. This allows for an an ease of transition between obstacle
// avoidance algorithms
SearchStateMachine* SearchFactory( StateMachine* stateMachine, SearchType type, Rover* rover, const rapidjson::Document& roverConfig );

#endif //SEARCH_STATE_MACHINE_HPP
