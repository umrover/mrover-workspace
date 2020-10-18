#include "utilities.hpp"
#include <iostream> // remove
#include <cmath>

// Coverts the input degree (and optional minute) to radians.
double degreeToRadian( const double degree, const double minute )
{
    return ( PI / 180 ) * ( degree + minute / 60 );
} // degreeToRadian

// Converts the input radians to degrees.
double radianToDegree( const double radian )
{
    return radian * 180 / PI;
}

// create a new odom with coordinates offset from current odom by a certain lat and lon change
Odometry addMinToDegrees( const Odometry & current, const double lat_minutes, const double lon_minutes )
{
    Odometry newOdom = current;
    double total_lat_min = current.latitude_min + lat_minutes;
    int sign_lat = total_lat_min < 0 ? -1 : 1;
    newOdom.latitude_min = mod( fabs( total_lat_min ), 60 ) * sign_lat;
    newOdom.latitude_deg += ( total_lat_min ) / 60;
    double total_lon_min = current.longitude_min + lon_minutes;
    int sign_lon = total_lon_min < 0 ? -1 : 1;
    newOdom.longitude_min = mod( fabs( total_lon_min ), 60 ) * sign_lon;
    newOdom.longitude_deg += ( total_lon_min )/60;

    return newOdom;
}

// Caclulates the non-euclidean distance between the current odometry and the
// destination odometry.
double estimateNoneuclid( const Odometry& current, const Odometry& dest )
{
    double currentLat = degreeToRadian( current.latitude_deg, current.latitude_min );
    double currentLon = degreeToRadian( current.longitude_deg, current.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double diffLat = ( destLat - currentLat );
    double diffLon = ( destLon - currentLon ) * cos( ( currentLat + destLat ) / 2 );
    return sqrt( diffLat * diffLat + diffLon * diffLon ) * EARTH_RADIUS;
}

// create a new Odometry point at a bearing and distance from a given odometry point
// Note this uses the absolute bearing not a bearing relative to the rover.
Odometry createOdom( const Odometry & current, double bearing, const double distance, Rover * phoebe )
{
    bearing = degreeToRadian( bearing );
    double latChange = distance * cos( bearing ) * LAT_METER_IN_MINUTES;
    double lonChange = distance * sin( bearing  ) * phoebe->longMeterInMinutes();
    Odometry newOdom = addMinToDegrees( current, latChange, lonChange );
    return newOdom;
}

// Caclulates the bearing between the current odometry and the
// destination odometry.
double calcBearing( const Odometry& start, const Odometry& dest )
{
    double currentLat = degreeToRadian( start.latitude_deg, start.latitude_min );
    double currentLon = degreeToRadian( start.longitude_deg, start.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double verticleComponentDist = EARTH_RADIUS * sin( destLat - currentLat );
    double noneuclidDist = estimateNoneuclid( start, dest );

    double bearing = acos( verticleComponentDist / noneuclidDist );
    if( currentLon > destLon )
    {
        bearing = 2 * PI - bearing;
    }

    if( verticleComponentDist < 0.001 && verticleComponentDist > -0.001 )
    {
        if( currentLon < destLon )
        {
            bearing = PI / 2;
        }
        else
        {
            bearing = 3 * PI / 2;
        }
    }
    return radianToDegree( bearing );
} // calcBearing()

// // Calculates the modulo of degree with the given modulus.
double mod( const double degree, const int modulus )
{
    double mod = fmod( degree, modulus );
    if( mod < 0 )
    {
        return ( mod + modulus );
    }
    return mod;
}

// Corrects the destination bearing to account for the ability to turn
// through zero degrees.
void throughZero( double& destinationBearing, const double currentBearing )
{
    if( fabs( currentBearing - destinationBearing ) > 180 )
    {
        if( currentBearing < 180 )
        {
            destinationBearing -= 360;
        }
        else
        {
            destinationBearing += 360;
        }
    }
} // throughZero()

// Clears the queue.
void clear( deque<Waypoint>& aDeque )
{
    deque<Waypoint> emptyDeque;
    swap( aDeque, emptyDeque );
} // clear()


// Checks to see if target is reachable before hitting obstacle
// If the x component of the distance to obstacle is greater than
// half the width of the rover the obstacle if reachable
bool isTargetReachable( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    double distToTarget = phoebe->roverStatus().target().distance;
    double distThresh = roverConfig["navThresholds"]["targetDistance"].GetDouble();
    return isLocationReachable( phoebe, roverConfig, distToTarget, distThresh );
} // istargetReachable()

// Returns true if the rover can reach the input location without hitting the obstacle.
// ASSUMPTION: There is an obstacle detected.
// ASSUMPTION: The rover is driving straight.
bool isLocationReachable( Rover* phoebe, const rapidjson::Document& roverConfig, const double locDist, const double distThresh )
{
    double distToObs = phoebe->roverStatus().obstacle().distance;
    double bearToObs = phoebe->roverStatus().obstacle().bearing;
    double bearToObsComplement = 90 - bearToObs;
    double xComponentOfDistToObs = distToObs * cos(bearToObsComplement);

    bool isReachable = false;

    // if location - distThresh is closer than the obstacle, it's reachable
    isReachable |= distToObs > locDist - distThresh;

    // if obstacle is farther away in "x direction" than rover's width, it's reachable
    isReachable |= xComponentOfDistToObs > roverConfig["roverMeasurements"]["width"].GetDouble() / 2;

    return isReachable;
} // isLocationReachable()
