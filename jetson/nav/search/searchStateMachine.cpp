#include "searchStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "searchFromPathFile.hpp"

#include <utility>
#include <iostream>

#include <eigen3/Eigen/Geometry>

using Eigen::Rotation2Dd;

SearchStateMachine::SearchStateMachine(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig)
        : mStateMachine(move(sm)), mConfig(roverConfig) {}

NavState SearchStateMachine::run() {
    switch (mStateMachine.lock()->getRover()->currentState()) {
        case NavState::Search: {
            return executeSearch();
        }
        case NavState::DriveToTarget: {
            return executeDriveToTarget();
        }
        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    }// switch
}// run()

NavState SearchStateMachine::executeSearch() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();
    Recovery* recovery = sm->getRecovery();

    Target const& leftTarget = env->getLeftTarget();
    Target const& rightTarget = env->getRightTarget();
    // TODO: this breaks when we dynamically update course
    Waypoint const& lastWaypoint = sm->getCourseState()->getLastCompletedWaypoint();
    bool isGate = lastWaypoint.gate;
    if (isGate) {
        if (env->hasGateLocation()) {
            return NavState::BeginGateSearch;
        } else {
            // Only drive to one of the posts if we don't have both locations
            if (!mDrivenToFirstPost) {
                if (leftTarget.id == lastWaypoint.id || leftTarget.id == lastWaypoint.id + 1) {
                    return NavState::DriveToTarget;
                }
            }
        }
    } else {
        // Either target works
        bool isWantedTarget = (lastWaypoint.id == leftTarget.id && leftTarget.id >= 0) ||
                              (lastWaypoint.id == rightTarget.id && rightTarget.id >= 0);
        if (isWantedTarget) {
            return NavState::DriveToTarget;
        }
    }

    double dt = sm->getDtSeconds();

    // Recovery watch for search
    if (recovery->getRecoveryState()) {
        std::cout << "Search RECOVERY\n";

        // if no recovery path has been generated, generate one
        if (recovery->getRecoveryPath().empty()) {
            recovery->makeRecoveryPath(rover->odometry(), *rover);
        }
        if (recovery->getPointToFollow().backwards) {
            // drive to first point in recovery path (special drive backwards function)
            if (rover->driveBackwards(recovery->getPointToFollow().odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                // mark point as complete and move to next point otherwise finish recovery manuever
                if (recovery->getRecoveryPath().size() > 1) {
                    recovery->completeCurrentRecoverypoint();
                } else {
                    // manuever is done
                    recovery->reset();
                    recovery->setRecoveryState(false);
                }
            }
        } else {
            // drive to first point in recovery path (drive forwards)
            if (rover->drive(recovery->getPointToFollow().odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                // mark point as complete and move to next point otherwise finish recovery manuever
                if (recovery->getRecoveryPath().size() > 1) {
                    recovery->completeCurrentRecoverypoint();
                } else {
                    // manuever is done
                    recovery->reset();
                    recovery->setRecoveryState(false);
                }
            }
        }
        return NavState::Search;// TODO: check that we always only want to do this?
    }

    Odometry const& nextSearchPoint = mSearchPoints.front();
    if (rover->drive(nextSearchPoint, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
        // We have reached the current search point
        // Start going to next if we have one, else finish
        mSearchPoints.pop_front();

        ProjectedPoints projectedPoints{};
        projectedPoints.points.assign(mSearchPoints.begin(), mSearchPoints.end());
        projectedPoints.pattern_size = static_cast<int32_t>(projectedPoints.points.size());
        projectedPoints.path_type = "search-path";
        std::string gatePathChannel = mConfig["lcmChannels"]["gatePathChannel"].GetString();
        sm->getLCM().publish(gatePathChannel, &projectedPoints);
        if (mSearchPoints.empty()) {
            return NavState::Done;
        }
    }
    return NavState::Search;
}// executeSearch()

NavState SearchStateMachine::executeDriveToTarget() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();
    Recovery* recovery = sm->getRecovery();

    double dt = sm->getDtSeconds();
    Waypoint lastWaypoint = sm->getCourseState()->getLastCompletedWaypoint();
    Target const& leftTarget = env->getLeftTarget();
    Target const& rightTarget = env->getRightTarget();
    double currentBearing = rover->odometry().bearing_deg;

    double distance, bearing;
    if (lastWaypoint.gate) {
        // If we have both pots start the gate search, we are done with searching
        if (env->hasGateLocation()) {
            return NavState::BeginGateSearch;
        }
        if (leftTarget.id >= 0 && rightTarget.id >= 0) {
            // Case: we see both targets but cache is not updated, just drive to closer
            // TODO: remove? should not be needed
            if (leftTarget.distance < rightTarget.distance) {
                distance = leftTarget.distance;
                bearing = leftTarget.bearing + currentBearing;
            } else {
                distance = rightTarget.distance;
                bearing = rightTarget.bearing + currentBearing;
            }
        } else if (leftTarget.id >= 0) {
            // Case: Only left is visible, drive there
            distance = leftTarget.distance;
            bearing = leftTarget.bearing + currentBearing;
        } else if (rightTarget.id >= 0) {
            // Case: Only right is visible, drive there
            distance = rightTarget.distance;
            bearing = rightTarget.bearing + currentBearing;
        } else {
            std::cout << "Lost target" << std::endl;
            return NavState::Search;
        }
    } else {
        //not gate (AR tag)
        if (leftTarget.id == lastWaypoint.id) {
            distance = leftTarget.distance;
            bearing = leftTarget.bearing + currentBearing;
        } else if (rightTarget.id == lastWaypoint.id){
            distance = rightTarget.distance;
            bearing = rightTarget.bearing + currentBearing;
        } else {
            std::cerr << "Lost target" << std::endl;
            return NavState::Search;
        }
    }

    // Recovery watch for DriveToTarget
    if (recovery->getRecoveryState()) {
        std::cout << "DriveToTarget RECOVERY\n";

        // if no recovery path has been generated, generate one
        if (recovery->getRecoveryPath().empty()) {
            recovery->makeTargetRecoveryPath(rover->odometry(), leftTarget, *rover);
        }
        if (recovery->getPointToFollow().backwards) {
            // drive to first point in recovery path (special drive backwards function)
            if (rover->driveBackwards(recovery->getPointToFollow().odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                // mark point as complete and move to next point otherwise finish recovery manuever
                if (recovery->getRecoveryPath().size() > 1) {
                    recovery->completeCurrentRecoverypoint();
                } else {
                    // manuever is done
                    recovery->reset();
                    recovery->setRecoveryState(false);
                }
            }
        } else {
            // drive to first point in recovery path (drive forwards)
            if (rover->drive(recovery->getPointToFollow().odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                // mark point as complete and move to next point otherwise finish recovery manuever
                if (recovery->getRecoveryPath().size() > 1) {
                    recovery->completeCurrentRecoverypoint();
                } else {
                    // manuever is done
                    recovery->reset();
                    recovery->setRecoveryState(false);
                }
            }
        }
        return NavState::DriveToTarget;
    }

    Odometry targetPoint = createOdom(rover->odometry(), bearing, distance, rover);

    if (rover->drive(targetPoint, mConfig["navThresholds"]["targetDistance"].GetDouble(), dt)) {
        if (sm->getCourseState()->getLastCompletedWaypoint().gate) {
            // This means we have only seen one gate post,
            // So clear the search points and start a diamond search
            // This should hopefully guarantee we find the other one
            // and thus start the gate traverse state
            // TODO: put this in some other function
            mSearchPoints.clear();

            Vector2d points[4] = {{0.5, 0.5},
                                  {1.0, 0.0},
                                  {0.5, -0.5},
                                  {0.0, 0.0}};
            double diamondScale = 4;
            for (auto& p: points) {
                p *= diamondScale;
                // Rotate points to be relative to the front of the rover, +x is forward, +y is right
                p = Rotation2Dd{degreeToRadian(currentBearing)} * p;
                // Convert to global latitude/longitude and add to search paths
                mSearchPoints.push_back(createOdom(rover->odometry(), p, rover));

//                ProjectedPoints proj{4, std::vector<Odometry>(mSearchPoints.begin(), mSearchPoints.end()), "gate-path"};
//                sm->getLCM().publish(mConfig["lcmChannels"]["gatePathChannel"].GetString(), &proj);
            }
            mDrivenToFirstPost = true;
            return NavState::Search;
//            return NavState::Done;
        } else {
            return NavState::Done;
        }
    }

    return NavState::DriveToTarget;
} // executeDriveToTarget()

/**
 * Add intermediate points between the existing search points in a path generated by a search algorithm.
 * The maximum separation between any points in the search point list is determined by the rover's sight distance.
 * TODO: commented out since it does not really help
 */
void SearchStateMachine::insertIntermediatePoints() {
//    double visionDistance = mConfig["computerVision"]["visionDistance"].GetDouble();
//    const double maxDifference = 2 * visionDistance;
//
//    for (int i = 0; i < int(mSearchPoints.size()) - 1; ++i) {
//        Odometry point1 = mSearchPoints.at(i);
//        Odometry point2 = mSearchPoints.at(i + 1);
//        double distance = estimateDistance(point1, point2);
//        if (distance > maxDifference) {
//            int numPoints = int(ceil(distance / maxDifference) - 1);
//            double newDifference = distance / (numPoints + 1);
//            double bearing = estimateBearing(point1, point2);
//            for (int j = 0; j < numPoints; ++j) {
//                Odometry startPoint = mSearchPoints.at(i);
//                Odometry newOdom = createOdom(startPoint, bearing, newDifference, mStateMachine.lock()->getRover());
//                auto insertPosition = mSearchPoints.begin() + i + 1;
//                mSearchPoints.insert(insertPosition, newOdom);
//                ++i;
//            }
//        }
//    }
} // insertIntermediatePoints()

std::shared_ptr<SearchStateMachine>
SearchFactory(const std::weak_ptr<StateMachine>& sm, SearchType type, const std::shared_ptr<Rover>& rover, const rapidjson::Document& roverConfig) {
    std::shared_ptr<SearchStateMachine> search = nullptr;
    switch (type) {
        case SearchType::FROM_PATH_FILE:
            search = std::make_shared<SearchFromPathFile>(sm, roverConfig, "jetson/nav/search/spiral_search_points.txt");
            break;
        case SearchType::FROM_PATH_FILE_GATE:
            search = std::make_shared<SearchFromPathFile>(sm, roverConfig, "jetson/nav/search/gate_search_points.txt");
            break;
        default:
            std::cerr << "Unknown Search Type. Defaulting to Spiral\n";
            break;
    }
    return search;
} // SearchFactory
