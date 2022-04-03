#pragma once

#include <deque>
#include <memory>
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"
#include "environment.hpp"

using namespace std;
using namespace rover_msgs;

const int EARTH_RADIUS = 6371000; // meters
const int EARTH_CIRCUM = 40075000; // meters
const double PI = 3.141592654; // radians
const double LAT_METER_IN_MINUTES = 0.0005389625; // minutes/meters

double degreeToRadian(double degree, double minute = 0);

double radianToDegree(double radian);

Odometry addMinToDegrees(const Odometry& current, double lat_minutes = 0, double lon_minutes = 0);

double estimateNoneuclid(const Odometry& start, const Odometry& dest);

Odometry createOdom(const Odometry& current, double bearing, double distance, const shared_ptr<Rover>& rover);

double calcBearing(const Odometry& start, const Odometry& dest);

double mod(double degree, int modulus);

void throughZero(double& destinationBearing, double currentBearing);

void clear(deque<Waypoint>& aDeque);

bool isTargetReachable(const shared_ptr<Rover>&, const shared_ptr<Environment>& env, const rapidjson::Document& roverConfig);

bool isLocationReachable(
        const shared_ptr<Rover>&, const shared_ptr<Environment>&, const rapidjson::Document& roverConfig, double locDist, double distThresh
);

bool isObstacleDetected(const shared_ptr<Rover>& rover, const shared_ptr<Environment>& env);

bool isObstacleInThreshold(const shared_ptr<Rover>& rover, const shared_ptr<Environment>& env, const rapidjson::Document& roverConfig);
