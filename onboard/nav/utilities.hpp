#ifndef NAV_UTILITES
#define NAV_UTILITES

#include "rover_msgs/Odometry.hpp"

#include <queue>

using namespace std;
using namespace rover_msgs;

const int EARTH_RADIUS = 6371000; // meters
const int EARTH_CIRCUM = 40075000; // meters
const double PI = 3.141592654; // radians
const double LAT_METER_IN_MINUTES = 0.0005389625; // minutes/meters

double degreeToRadian( const double degree, const double minute = 0 );

double radianToDegree( const double radian );

double estimateNoneuclid(
	const Odometry& start,
	const Odometry& dest );

double calcBearing(
	const Odometry& start,
	const Odometry& dest );

double mod( const double degree, const int modulus );

void throughZero( double& destinationBearing, const double currentBearing );

void clear( queue<Odometry>& aQueue );

#endif // NAV_UTILITES
