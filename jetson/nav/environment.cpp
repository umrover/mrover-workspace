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

void Environment::setBaseGateID(int baseGateId) {
    mBaseGateId = baseGateId;
}

int Environment::getBaseGateID() const {
    return mBaseGateId;
}

void Environment::setTargets(TargetList const& targets) {
    Target const& leftTarget = targets.targetList[0];
    Target const& rightTarget = targets.targetList[1];
    if (leftTarget.id < 0 || leftTarget.distance < 0.0) {
        mLeftBearingFilter.decrementCount();
        mLeftDistanceFilter.decrementCount();
        if (mLeftBearingFilter.filterCount() == 0) {
            mTargetLeft.id = -1;
        }
    } else {
        std::cout << "adding left readings" << std::endl;
        mLeftBearingFilter.push(leftTarget.bearing);
        mLeftDistanceFilter.push(leftTarget.distance);
        mTargetLeft.bearing = mLeftBearingFilter.get();
        mTargetLeft.distance = mLeftDistanceFilter.get();
        mTargetLeft.id = leftTarget.id;
    }

    if (rightTarget.id < 0 || rightTarget.distance < 0.0) {
        mRightBearingFilter.decrementCount();
        mRightDistanceFilter.decrementCount();
        if (mRightBearingFilter.filterCount() == 0) {
            mTargetRight.id = -1;
        }
    } else {
        std::cout << "adding right readings" << std::endl;
        mRightBearingFilter.push(rightTarget.bearing);
        mRightDistanceFilter.push(rightTarget.distance);
        mTargetRight.bearing = mRightBearingFilter.get();
        mTargetRight.distance = mRightDistanceFilter.get();
        mTargetRight.id = rightTarget.id;
    }
}

void Environment::updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course) {
    if (rover->autonState().is_auton) {
        bool rightReady = isRightTargetFilterReady();
        bool leftReady = isLeftTargetFilterReady();
        double currentBearing = rover->odometry().bearing_deg;
        if (rightReady) {
//            std::cout << "right ready: " << mTargetRight.id << std::endl;
            if (mTargetRight.id == mBaseGateId) {
//                std::cout << "updated post 1" << std::endl;
                mPostOne = createOdom(rover->odometry(), currentBearing + mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
                mHasPostOne = true;
            }
            if (mTargetRight.id == mBaseGateId + 1) {
//                std::cout << "updated post 2" << std::endl;
                mPostTwo = createOdom(rover->odometry(), currentBearing + mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
                mHasPostTwo = true;
            }
        }
        if (leftReady) {
//            std::cout << "left ready: " << mTargetLeft.id << std::endl;
//            std::cout << "course waypoint id: " << course->getCurrentWaypoint().id << std::endl;
            if (mTargetLeft.id == mBaseGateId) {
//                std::cout << "updated post 1" << std::endl;
                mPostOne = createOdom(rover->odometry(), currentBearing + mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
                mHasPostOne = true;
            }
            if (mTargetLeft.id == mBaseGateId + 1) {
//                std::cout << "updated post 2" << std::endl;
                mPostTwo = createOdom(rover->odometry(), currentBearing + mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
                mHasPostTwo = true;
            }
        }
    } else {
        mHasPostOne = mHasPostTwo = false;
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes(60 / (EARTH_CIRCUM * cosine / 360));
    }
}

bool Environment::hasGateLocation() const {
    return mHasPostOne && mHasPostTwo;
}

bool Environment::hasPostOneLocation() const {
    return mHasPostOne;
}

bool Environment::hasPostTwoLocation() const {
    return mHasPostTwo;
}

Odometry Environment::getPostOneLocation() {
    return mPostOne;
}

Odometry Environment::getPostTwoLocation() {
    return mPostTwo;
}

// Offset of the post in our linearized cartesian space.
Vector2d Environment::getPostOneOffsetInCartesian(Odometry cur) {
    double bearing = degreeToRadian(estimateBearing(cur, mPostOne));
    double distance = estimateDistance(cur, mPostOne);
    return {distance * cos(bearing), distance * sin(bearing)};
}

Vector2d Environment::getPostTwoOffsetInCartesian(Odometry cur) {
    double bearing = degreeToRadian(estimateBearing(cur, mPostTwo));
    double distance = estimateDistance(cur, mPostTwo);
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
