#include "utilities.hpp"

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

// Caclulates the bearing between the current odometry and the
// destination odometry.
double calcBearing( const Odometry& current, const Odometry& dest )
{
	double currentLat = degreeToRadian( current.latitude_deg, current.latitude_min );
	double currentLon = degreeToRadian( current.longitude_deg, current.longitude_min );
	double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
	double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

	double verticleComponentDist = EARTH_RADIUS * sin( destLat - currentLat );
	double noneuclidDist = estimateNoneuclid( current, dest );

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

// Calculates the modulo of degree with the given modulus.
double mod( const double degree, const int modulus )
{
	if( degree >= 0 && degree < modulus )
	{
		return degree;
	}
	if( degree >= modulus )
	{
		return mod( degree - modulus, modulus );
	}
	// if degree < 0
	return mod( degree + modulus, modulus );
} // mod()

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
void clear( queue<Odometry>& aQueue )
{
	queue<Odometry> emptyQueue;
	swap( aQueue, emptyQueue );
} // clear()
