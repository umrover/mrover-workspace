#ifndef NAV_UTILITES
#define NAV_UTILITES

#include <deque>
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"

using namespace std;
using namespace rover_msgs;

const int EARTH_RADIUS = 6371000; // meters
const int EARTH_CIRCUM = 40075000; // meters
const double PI = 3.141592654; // radians
const double LAT_METER_IN_MINUTES = 0.0005389625; // minutes/meters

double degreeToRadian( const double degree, const double minute = 0 );

double radianToDegree( const double radian );

Odometry addMinToDegrees( const Odometry & current, const double lat_minutes = 0, const double lon_minutes = 0 );

double estimateNoneuclid( const Odometry& start, const Odometry& dest );

Odometry createOdom ( const Odometry & current, const double bearing, const double distance, Rover * phoebe );

double calcBearing( const Odometry& start, const Odometry& dest );

double mod( const double degree, const int modulus );

void throughZero( double& destinationBearing, const double currentBearing );

void clear( deque<Waypoint>& aDeque );

bool isTargetReachable( Rover* phoebe, const rapidjson::Document& roverConfig );

bool isLocationReachable( Rover* phoebe, const rapidjson::Document& roverConfig, const double locDist, const double distThresh );

#endif // NAV_UTILITES
