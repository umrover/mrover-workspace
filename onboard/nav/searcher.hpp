#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class StateMachine;

class Searcher {
public:
    /*************************************************************************/
    /* Public Member Functions */
    /*************************************************************************/
    Searcher(StateMachine* stateMachine_)
    : stateMachine(stateMachine_) {}

    virtual ~Searcher() {}

    NavState run( Rover * phoebe, const rapidjson::Document& roverConfig );

    Odometry frontSearchPoint();

    void popSearchPoint();

    virtual void initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig, double pathWidth ) = 0; // TODO

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    NavState executeSearchFaceNorth( Rover* phoebe );

    NavState executeSearchSpin( Rover* phoebe, const rapidjson::Document& roverConfig );

    NavState executeSearchSpinWait( Rover* phoebe, const rapidjson::Document& roverConfig );

    NavState executeSearchTurn( Rover* phoebe, const rapidjson::Document& roverConfig );

    NavState executeSearchDrive( Rover* phoebe );

    NavState executeTurnToBall( Rover* phoebe );

    NavState executeDriveToBall( Rover* phoebe );

protected:  // TODO
    /*************************************************************************/
    /* Protected Member Variables */
    /*************************************************************************/
    StateMachine* stateMachine;

    // Vector of search point multipliers used as a base for the search points.
    vector< pair<short, short> > mSearchPointMultipliers;

    // Queue of search points.
    deque<Odometry> mSearchPoints;

};

#endif //SEARCHER_HPP


/*************************************************************************/
/* TODO */
/*************************************************************************/
// TODO: Attempt to remove protected member variables ( private? )
// TODO: move initializeSearch back to private function

