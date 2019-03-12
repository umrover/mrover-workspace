/* Searches.cpp */

#include "searches.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>
#include <stack>

void reverseQueue(queue<Odometry>& Queue) //TODO
{
    stack<Odometry> Stack;
    while (!Queue.empty()) {
        Stack.push(Queue.front());
        Queue.pop();
    }
    while (!Stack.empty()) {
        Queue.push(Stack.top());
        Stack.pop();
    }
}

/*************************************************************************/
/* Searcher Factory */
/*************************************************************************/
Searcher* SearchFactory( StateMachine* stateMachine, SearchType type )  //TODO
{
    Searcher* search = nullptr;
    switch (type)
    {
        case SearchType::SPIRALOUT:
            search = new SpiralOut( stateMachine );
            break;

        case SearchType::LAWNMOWER:
            search = new LawnMower( stateMachine );
            break;

        case SearchType::SPIRALIN:
            search = new SpiralIn( stateMachine );
            break;

        case SearchType::UNKNOWN:
            std::cerr << "Unkown Search Type. Defaulting to Spiral\n";
            search = new SpiralOut( stateMachine );
            break;
    }
    return search;
}

/*************************************************************************/
/* Spiral Search */
/*************************************************************************/
SpiralOut::~SpiralOut() {}

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void SpiralOut::initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig, const double pathWidth )
{
    mSearchPoints.clear();

    mSearchPointMultipliers.clear();
    mSearchPointMultipliers.push_back( pair<short, short> (  0,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1, -1 ) );

    while( mSearchPointMultipliers[ 0 ].second * pathWidth < mRoverConfig[ "search" ][ "bailThresh" ].GetDouble() ) {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = mPhoebe->roverStatus().path().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * pathWidth * mPhoebe->longMeterInMinutes() );

            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            mSearchPoints.push_back( nextSearchPoint );

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;

        }
    }
} // initializeSearch()

/*************************************************************************/
/* Spiral Search */
/*************************************************************************/
SpiralIn::~SpiralIn() {}

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void SpiralIn::initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig, const double pathWidth )
{
    mSearchPoints.clear();

    mSearchPointMultipliers.clear();
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1, -1 ) );

    while( mSearchPointMultipliers[ 0 ].second * pathWidth < mRoverConfig[ "search" ][ "bailThresh" ].GetDouble() ) {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = mPhoebe->roverStatus().path().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * pathWidth * mPhoebe->longMeterInMinutes() );

            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            mSearchPoints.push_back( nextSearchPoint );

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
        }
    }
    //TODO Reverse Deque. Not using this search though...
} // initializeSearch()

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
LawnMower::~LawnMower() {}

void LawnMower::initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig, const double pathWidth )
{
    const double searchBailThresh = mRoverConfig[ "search" ][ "bailThresh" ].GetDouble();

    mSearchPoints.clear();

    mSearchPointMultipliers.clear();
    // mSearchPointMultipliers.push_back( pair<short, short> (  0, 0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  0, 1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, 0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -2, 0 ) );


    while( fabs(mSearchPointMultipliers[ 0 ].first * pathWidth) < searchBailThresh )
    {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = mPhoebe->roverStatus().odometry();

            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * (2 * searchBailThresh) * mPhoebe->longMeterInMinutes() );

            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            mSearchPointMultiplier.first -= 2;
            mSearchPoints.push_back( nextSearchPoint );
        }
    }
} // initializeSearch()



/*************/
/* TODO */
/*************/
// TODO: Incorporate this into the StateMachine Function?
//       Currently seems like a swiss-army knife function. Too abstracted. No reason for it to be here.
// TODO: More efficient spiral in than reversing the whole queue?


