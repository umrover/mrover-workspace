#include "spiralInSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>

SpiralIn::~SpiralIn() {}

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void SpiralIn::initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig, const double visionDistance )
{
    mSearchPoints.clear();

    mSearchPointMultipliers.clear();
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1, -1 ) );

    while( mSearchPointMultipliers[ 0 ].second * visionDistance < roverConfig[ "search" ][ "bailThresh" ].GetDouble() ) {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = phoebe->roverStatus().path().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * visionDistance  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * visionDistance * phoebe->longMeterInMinutes() );
            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            mSearchPoints.push_back( nextSearchPoint );

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
        }
    }
    insertIntermediatePoints( phoebe, roverConfig );
    //TODO Reverse Deque. Not using this search though...
} // initializeSearch()