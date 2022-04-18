#pragma once

#include <deque>
#include <memory>
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"
#include "environment.hpp"


using namespace rover_msgs;

const int EARTH_RADIUS = 6371000; // meters
const int EARTH_CIRCUM = 40075000; // meters
const double PI = 3.141592654; // radians
const double LAT_METER_IN_MINUTES = 0.0005389625; // minutes/meters

double degreeToRadian(double degree, double minute = 0);

double radianToDegree(double radian);

Odometry addMinToDegrees(const Odometry& current, double lat_minutes = 0, double lon_minutes = 0);

double estimateDistance(const Odometry& current, const Odometry& dest);

Odometry createOdom(const Odometry& current, double absoluteBearing, double distance, const std::shared_ptr<Rover>& rover);

Odometry createOdom(const Odometry& current, Vector2d offset, const std::shared_ptr<Rover>& rover);

double estimateBearing(const Odometry& start, const Odometry& dest);

double mod(double value, double modulus);

bool isObstacleInThreshold(const std::shared_ptr<Rover>& rover, const std::shared_ptr<Environment>& env, const rapidjson::Document& roverConfig);

double distanceFromPointToLine();

double doesVectorIntersectCircle();

Odometry createOdom(double latitude, double longitude);

Vector2d getOffsetInCartesian(Odometry current, Odometry target);


