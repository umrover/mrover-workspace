#include <iostream>
#include "cmath"
#include "environment.hpp"
#include "utilities.hpp"

Environment::Environment(const rapidjson::Document& config) : mRoverConfig(config), leftBearingFilter(config["gate"]["filterSize"].GetInt()),
                                                             rightBearingFilter(config["gate"]["filterSize"].GetInt()), leftDistanceFilter(config["gate"]["filterSize"].GetInt()),
                                                             rightDistanceFilter(config["gate"]["filterSize"].GetInt()) {}

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
    return mTargetLeft;
}


void Environment::setTargets(TargetList const& targets) {
    mTargetLeft = targets.targetList[0];
    mTargetRight = targets.targetList[1];
    if (targets.targetList[0].id == -1){
        leftBearingFilter.reset();
        leftDistanceFilter.reset();
    }
    else{
        leftBearingFilter.push(mTargetLeft.bearing);
        leftDistanceFilter.push(mTargetLeft.distance);
        mTargetLeft.bearing = leftBearingFilter.get(0.75);
        mTargetLeft.distance = leftDistanceFilter.get(0.75);
    }

    if (targets.targetList[1].id == -1){
        rightBearingFilter.reset();
        rightDistanceFilter.reset();
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
        // Cache Left Target if we had detected one
        if (mTargetLeft.distance != mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
            // Associate with single post
            if (mTargetLeft.id == course->getRemainingWaypoints().front().id) {
                mCountLeftHits++;
            } else {
                mCountLeftHits = 0;
            }
            // Update leftTarget if we have 3 or more consecutive hits
            if (mCountLeftHits >= 3) {
                mCacheTargetLeft = mTargetLeft;
                mCountLeftMisses = 0;
            }
            // Cache Right Target if we had detected one (only can see right if we see the left one, otherwise
            // results in some undefined behavior)
            if (mTargetRight.distance != mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
                mCacheTargetRight = mTargetRight;
                mCountRightMisses = 0;
            } else {
                mCountRightMisses++;
            }
        } else {
            mCountLeftMisses++;
            mCountRightMisses++; // need to increment since we don't see both
            mCountLeftHits = 0;
            mCountRightHits = 0;
        }

        // Check if we need to reset left cache
        if (mCountLeftMisses > mRoverConfig["navThresholds"]["cacheMissMax"].GetDouble()) {
            mCountLeftMisses = 0;
            mCountLeftHits = 0;
            // Set to empty target
            mCacheTargetLeft = {-1, 0, 0};
        }
        // Check if we need to reset right cache
        if (mCountRightMisses > mRoverConfig["navThresholds"]["cacheMissMax"].GetDouble()) {
            mCountRightMisses = 0;
            mCountRightHits = 0;
            // Set to empty target
            mCacheTargetRight = {-1, 0, 0};
        }

        bool gate = course->getRemainingWaypoints().front().gate;
        if (gate){
            if (leftCacheTarget().id != -1){
                leftPost = createOdom(rover->odometry(), leftCacheTarget().bearing, leftCacheTarget().distance, rover);
                hasLeftPost = true;
            }
            if (rightCacheTarget().id != -1){
                rightPost = createOdom(rover->odometry(), rightCacheTarget().bearing, rightCacheTarget().distance, rover);
                hasRightPost = true;
            }
        }
    } else {
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes( 60 / (EARTH_CIRCUM * cosine / 360) );
    }

    
}

Target const& Environment::leftCacheTarget() const {
    return mCacheTargetLeft;
} // leftCacheTarget()

Target const& Environment::rightCacheTarget() const {
    return mCacheTargetRight;
} // rightCacheTarget()

int Environment::getLeftMisses() const {
    return mCountLeftMisses;
}

int Environment::getRightMisses() const {
    return mCountRightMisses;
}

int Environment::getLeftHits() const {
    return mCountLeftHits;
}

int Environment::getRightHits() const {
    return mCountRightHits;
}

void Environment::resetMisses() {
    mCountLeftMisses = 0;
    mCountRightMisses = 0;
}

bool Environment::hasGateLocation() {
    return hasLeftPost && hasRightPost;
}

Odometry Environment::getLeftPostLocation(){
    return leftPost;
}

Odometry Environment::getRightPostLocation(){
    return rightPost;
}