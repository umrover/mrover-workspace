#include "utilities.hpp"
#include <iostream> // remove
#include <cmath>

// Coverts the input degree (and optional minute) to radians.
double degreeToRadian(const double degree, const double minute) {
    return (PI / 180) * (degree + minute / 60);
} // degreeToRadian

// Converts the input radians to degrees.
double radianToDegree(const double radian) {
    return radian * 180 / PI;
} // radianToDegree

// create a new odom with coordinates offset from current odom by a certain lat and lon change
Odometry addMinToDegrees(const Odometry& current, const double lat_minutes, const double lon_minutes) {
    Odometry newOdom = current;
    double total_lat_min = current.latitude_min + lat_minutes;
    int sign_lat = total_lat_min < 0 ? -1 : 1;
    newOdom.latitude_min = mod(fabs(total_lat_min), 60) * sign_lat;
    newOdom.latitude_deg += static_cast<int32_t>(total_lat_min) / 60;
    double total_lon_min = current.longitude_min + lon_minutes;
    int sign_lon = total_lon_min < 0 ? -1 : 1;
    newOdom.longitude_min = mod(fabs(total_lon_min), 60) * sign_lon;
    newOdom.longitude_deg += static_cast<int32_t>(total_lon_min) / 60;

    return newOdom;
} // addMinToDegrees

// Estimate approximate distance using euclidean methods
double estimateDistance(const Odometry& current, const Odometry& dest) {
    double currentLat = degreeToRadian(current.latitude_deg, current.latitude_min);
    double currentLon = degreeToRadian(current.longitude_deg, current.longitude_min);
    double destLat = degreeToRadian(dest.latitude_deg, dest.latitude_min);
    double destLon = degreeToRadian(dest.longitude_deg, dest.longitude_min);

    double diffLat = (destLat - currentLat);
    double diffLon = (destLon - currentLon) * cos((currentLat + destLat) / 2);
    return sqrt(diffLat * diffLat + diffLon * diffLon) * EARTH_RADIUS;
} // estimateDistance

/***
 * @param current           Current position
 * @param absoluteBearing   Absolute bearing (relative to North) in degrees
 * @param distance          Distance in meters
 * @return                  New odometry point in given bearing direction and distance away
 */
Odometry createOdom(const Odometry& current, double absoluteBearing, double distance, const std::shared_ptr<Rover>& rover) {
    absoluteBearing = degreeToRadian(absoluteBearing);
    double latChange = distance * cos(absoluteBearing) * LAT_METER_IN_MINUTES;
    double lonChange = distance * sin(absoluteBearing) * rover->longMeterInMinutes();
    Odometry newOdom = addMinToDegrees(current, latChange, lonChange);
    return newOdom;
} // createOdom

/***
 * @param current   Current position
 * @param offset    Relative offset from the given position, (+1, 0) is North
 * @return          New odometry offset by given vector
 */
Odometry createOdom(const Odometry& current, Vector2d offset, const std::shared_ptr<Rover>& rover) {
    double bearing = radianToDegree(atan2(offset.y(), offset.x()));
    double distance = offset.norm();
    return createOdom(current, bearing, distance, rover);
} // createOdom

// Approximate the LHS bearing (clockwise rotation in positive) between two global odometries.
// The linearization that occurs is implicitly defined relative to the destination.
double estimateBearing(const Odometry& start, const Odometry& dest) {
    double currentLat = degreeToRadian(start.latitude_deg, start.latitude_min);
    double currentLon = degreeToRadian(start.longitude_deg, start.longitude_min);
    double destLat = degreeToRadian(dest.latitude_deg, dest.latitude_min);
    double destLon = degreeToRadian(dest.longitude_deg, dest.longitude_min);

    double vertComponentDist = EARTH_RADIUS * sin(destLat - currentLat);
    double noneuclidDist = estimateDistance(start, dest);

    double bearing = acos(vertComponentDist / noneuclidDist);
    if (currentLon > destLon) {
        bearing = 2 * PI - bearing;
    }

    if (vertComponentDist < 0.001 && vertComponentDist > -0.001) {
        if (currentLon < destLon) {
            bearing = PI / 2;
        } else {
            bearing = 3 * PI / 2;
        }
    }
    return radianToDegree(bearing);
} // estimateBearing

// Calculates the modulo of value with the given modulus.
// This handles the case where value is negatively properly.
double mod(double value, double modulus) {
    double mod = fmod(value, modulus);
    return mod < 0 ? mod + modulus : mod;
} // mod()

Odometry createOdom(double latitude, double longitude) {
    double latitudeDeg;
    double longitudeDeg;
    double latitudeMin = std::modf(latitude, &latitudeDeg);
    double longitudeMin = std::modf(longitude, &longitudeDeg);
    latitudeMin *= 60.0;
    longitudeMin *= 60.0;
    return Odometry{
            static_cast<int32_t>(latitudeDeg), latitudeMin,
            static_cast<int32_t>(longitudeDeg), longitudeMin
    };
} // createOdom

/***
 * @param current   From position
 * @param target    To position
 * @return          Vector offset in space where (+1, 0) is North
 */
Vector2d getOffsetInCartesian(Odometry current, Odometry target) {
    double bearing = degreeToRadian(estimateBearing(current, target));
    double distance = estimateDistance(current, target);
    return {distance * cos(bearing), distance * sin(bearing)};
} // getOffsetInCartesian
