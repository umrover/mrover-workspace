/* Searches.cpp */

#include "searches.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>

/*************************************************************************/
/* Spiral Search */
/*************************************************************************/
Spiral::~Spiral() {}

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void Spiral::initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig )
{
	const double pathWidth = mRoverConfig[ "pathWidth" ].GetDouble();

	clear( mSearchPoints );

	mSearchPointMultipliers.clear();
	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 1, -1 ) );
	
	while( mSearchPointMultipliers[ 0 ].second * pathWidth < mRoverConfig[ "searchBailThresh" ].GetDouble() ) {
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

			mSearchPoints.push( nextSearchPoint );

			mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
			mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
		}
	}
	cout << mSearchPoints.size() << endl;
} // initializeSearch()

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
LawnMower::~LawnMower() {}

void LawnMower::initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig )
{
	const double pathWidth = mRoverConfig[ "pathWidth" ].GetDouble();
	const double searchBailThresh = mRoverConfig[ "searchBailThresh" ].GetDouble();

	clear( mSearchPoints );
	mSearchPointMultipliers.clear();
	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 0 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 1, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 1, 0 ) );

	while( mSearchPointMultipliers[ 0 ].first * pathWidth < searchBailThresh ) 
	{
		for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
		{
			Odometry nextSearchPoint = mPhoebe->roverStatus().path().front().odom;

			double totalLatitudeMinutes = nextSearchPoint.latitude_min +
				( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
			double totalLongitudeMinutes = nextSearchPoint.longitude_min +
				( mSearchPointMultiplier.second * (2 * searchBailThresh) * mPhoebe->longMeterInMinutes() );

			nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
			nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
			nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
			nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

			mSearchPointMultiplier.first += 2;
			mSearchPoints.push( nextSearchPoint );
		}
	}
} // initializeSearch()


/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
Searcher* SearchFactory( StateMachine* stateMachine, SearchType type ) 
{
	Searcher* search = nullptr;
	switch (type)
	{
		case SearchType::SPIRAL:
			search = new Spiral( stateMachine );
			break;

		case SearchType::LAWNMOWER:
			search = new LawnMower( stateMachine );
			break;
		
		case SearchType::UNKNOWN:
			std::cerr << "Unkown Search Type. Defaulting to Spiral\n";
			search = new Spiral( stateMachine );
			break;
	}
	return search;
}
