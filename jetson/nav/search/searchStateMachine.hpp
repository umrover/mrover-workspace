#pragma once

#include <memory>

#include "rover.hpp"
#include "utilities.hpp"

class StateMachine;

// This class is the representation of different
// search algorithms
enum class SearchType {
    FROM_PATH_FILE = 0, 
    FROM_PATH_FILE_GATE = 1
};

class SearchStateMachine {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    SearchStateMachine(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig);

    virtual ~SearchStateMachine() = default;

    NavState run();

//    bool targetReachable(std::shared_ptr<Rover> rover, double distance, double bearing);

    virtual void initializeSearch(const rapidjson::Document& roverConfig, double pathWidth) = 0; // TODO

protected:
    /*************************************************************************/
    /* Protected Member Functions */
    /*************************************************************************/

    void insertIntermediatePoints();

    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/

    // Pointer to rover State Machine to access member functions
    std::weak_ptr<StateMachine> mStateMachine;

    // Vector of search point multipliers used as a base for the search points.
    std::vector<std::pair<short, short>> mSearchPointMultipliers;

    // Queue of search points.
    std::deque<Odometry> mSearchPoints;

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    NavState executeSearchTurn();

    NavState executeSearchDrive();

    NavState executeTurnToTarget();

    NavState executeDriveToTarget();

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // Reference to config variables
    const rapidjson::Document& mRoverConfig;

};

// Creates an ObstacleAvoidanceStateMachine object based on the inputted obstacle
// avoidance algorithm. This allows for an an ease of transition between obstacle
// avoidance algorithms
std::shared_ptr<SearchStateMachine>
SearchFactory(const std::weak_ptr<StateMachine>& sm, SearchType type, const std::shared_ptr<Rover>& rover, const rapidjson::Document& roverConfig);
