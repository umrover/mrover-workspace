#include <cmath>

#include "utilities.hpp"
#include "environment.hpp"
#include <iostream>
#include <string>

Environment::Environment(const rapidjson::Document& config) :
        mConfig(config),
        mLeftBearingFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()),
        mRightBearingFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()),
        mLeftDistanceFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()),
        mRightDistanceFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()) {}

void Environment::setObstacle(Obstacle const& obstacle) {
    mObstacle = obstacle;
}

Obstacle Environment::getObstacle() {
    return mObstacle;
}

Target Environment::getLeftTarget() {
    return mTargetLeft;
}

Target Environment::getRightTarget() {
    return mTargetRight;
}

void Environment::setBaseGateID(int b){
    baseGateID = b;
}

int Environment::getBaseGateID(){
    return baseGateID;
}

void Environment::setTargets(TargetList const& targets) {
    mTargetLeft = targets.targetList[0];
    mTargetRight = targets.targetList[1];
    if (mTargetLeft.id == mConfig["navThresholds"]["noTargetDist"].GetInt()) {
        mLeftBearingFilter.reset();
        mLeftDistanceFilter.reset();
    } else {
        mLeftBearingFilter.push(mTargetLeft.bearing);
        mLeftDistanceFilter.push(mTargetLeft.distance);
        mTargetLeft.bearing = mLeftBearingFilter.get();
        mTargetLeft.distance = mLeftDistanceFilter.get();
    }

    if (targets.targetList[1].id == mConfig["navThresholds"]["noTargetDist"].GetInt()) {
        mRightBearingFilter.reset();
        mRightDistanceFilter.reset();
    } else {
        std::cout << "adding right readings" << std::endl;
        mRightBearingFilter.push(mTargetRight.bearing);
        mRightDistanceFilter.push(mTargetRight.distance);
        mTargetRight.bearing = mRightBearingFilter.get();
        mTargetRight.distance = mRightDistanceFilter.get();
    }
}

void Environment::updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course) {
    if (rover->autonState().is_auton) {
        bool rightReady = isRightTargetFilterReady();
        bool leftReady = isLeftTargetFilterReady();
        double currentBearing = rover->odometry().bearing_deg;
        if (rightReady){
//            std::cout << "right ready: " << mTargetRight.id << std::endl;
            if (mTargetRight.id == baseGateID){
//                std::cout << "updated post 1" << std::endl;
                mPostOne = createOdom(rover->odometry(), currentBearing + mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
                hasPostOne = true;
            }
            if (mTargetRight.id == baseGateID + 1){
//                std::cout << "updated post 2" << std::endl;
                mPostTwo = createOdom(rover->odometry(), currentBearing + mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
                hasPostTwo = true;
            }
        }
        if (leftReady){
//            std::cout << "left ready: " << mTargetLeft.id << std::endl;
//            std::cout << "course waypoint id: " << course->getCurrentWaypoint().id << std::endl;
            if (mTargetLeft.id == baseGateID){
//                std::cout << "updated post 1" << std::endl;
                mPostOne = createOdom(rover->odometry(), currentBearing + mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
                hasPostOne = true;
            }
            if (mTargetLeft.id == baseGateID + 1){
//                std::cout << "updated post 2" << std::endl;
                mPostTwo = createOdom(rover->odometry(), currentBearing + mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
                hasPostTwo = true;
            }
        }
    } else {
        hasPostOne = hasPostTwo = false;
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes(60 / (EARTH_CIRCUM * cosine / 360));
    }
}

bool Environment::hasGateLocation() const {
    return hasPostOne && hasPostTwo;
}

bool Environment::hasPostOneLocation() const {
    return hasPostOne;
}

bool Environment::hasPostTwoLocation() const {
    return hasPostTwo;
}

Odometry Environment::getPostOneLocation() {
    return mPostOne;
}

Odometry Environment::getPostTwoLocation() {
    return mPostTwo;
}

Vector2d Environment::getPostOneRelative(Odometry cur) {
    double bearing = degreeToRadian(calcBearing(cur, mPostOne));
    double distance = estimateNoneuclid(cur, mPostOne);
    return {distance * cos(bearing), distance * sin(bearing)};
}

Vector2d Environment::getPostTwoRelative(Odometry cur) {
    double bearing = degreeToRadian(calcBearing(cur, mPostTwo));
    double distance = estimateNoneuclid(cur, mPostTwo);
    return {distance * cos(bearing), distance * sin(bearing)};
}

bool Environment::areTargetFiltersReady() const {
    return mLeftDistanceFilter.ready() && mRightDistanceFilter.ready() && mLeftBearingFilter.ready() && mRightBearingFilter.ready();
}

bool Environment::isLeftTargetFilterReady() const {
    return mLeftDistanceFilter.ready() && mLeftBearingFilter.ready();
}

bool Environment::isRightTargetFilterReady() const {
    return mRightDistanceFilter.ready() && mRightBearingFilter.ready();
}

std::optional<Target> Environment::tryGetTargetWithId(int32_t id) {
    if (mTargetLeft.id == id && mTargetLeft.distance > 0.0) {
        return {mTargetLeft};
    } else if (mTargetRight.id == id && mTargetRight.distance > 0.0) {
        return {mTargetRight};
    }
    return std::nullopt;
}
