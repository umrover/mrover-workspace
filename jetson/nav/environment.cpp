#include <iostream>
#include "cmath"
#include "environment.hpp"
#include "utilities.hpp"

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
        mRightBearingFilter.push(mTargetRight.bearing);
        mRightDistanceFilter.push(mTargetRight.distance);
        mTargetRight.bearing = mRightBearingFilter.get();
        mTargetRight.distance = mRightDistanceFilter.get();
    }
    else{
        rightBearingFilter.push(mTargetRight.bearing);
        rightDistanceFilter.push(mTargetRight.distance);
        mTargetRight.bearing = rightBearingFilter.get(0.75);
        mTargetRight.distance = rightDistanceFilter.get(0.75);
    }
    
}

void Environment::updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course) {
    if (rover->autonState().is_auton) {
        bool isReady = areTargetFiltersReady();
        if (isReady) {
            mLeftPost = createOdom(rover->odometry(), mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
            mHasLeftPost = true;
            mRightPost = createOdom(rover->odometry(), mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
            mHasRightPost = true;
        } else {
            mHasLeftPost = mHasRightPost = false;
        }
    } else {
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes( 60 / (EARTH_CIRCUM * cosine / 360) );
    }

    
}

bool Environment::hasGateLocation() const {
    return mHasLeftPost && mHasRightPost;
}

Odometry Environment::getLeftPostLocation(){
    return leftPost;
}

Odometry Environment::getRightPostLocation() {
    return mRightPost;
}

Vector2d Environment::getLeftPostRelative() {
    double d = mLeftDistanceFilter.get(), b = mLeftBearingFilter.get();
    return {d * cos(degreeToRadian(b)), d * sin(degreeToRadian(b))};
}

Vector2d Environment::getRightPostRelative() {
    double d = mRightDistanceFilter.get(), b = mRightBearingFilter.get();
    return {d * cos(degreeToRadian(b)), d * sin(degreeToRadian(b))};
}

bool Environment::areTargetFiltersReady() const {
    return mLeftDistanceFilter.full() && mRightDistanceFilter.full() && mLeftBearingFilter.full() && mRightBearingFilter.full();
}

std::optional<Target> Environment::tryGetTargetWithId(int32_t id) {
    if (mTargetLeft.id == id && mTargetLeft.distance > 0.0) {
        return {mTargetLeft};
    } else if (mTargetRight.id == id && mTargetRight.distance > 0.0) {
        return {mTargetRight};
    }
    return std::nullopt;
}
