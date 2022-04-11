#include "searchStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "searchFromPathFile.hpp"

#include <cmath>
#include <utility>
#include <iostream>

// Constructs an SearchStateMachine object with mStateMachine, mConfig, and mRover
SearchStateMachine::SearchStateMachine(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig)
        : mStateMachine(move(sm)), mConfig(roverConfig) {}

// Runs the search state machine through one iteration. This will be called by
// StateMachine  when NavState is in a search state. This will call the corresponding
// function based on the current state and return the next NavState
NavState SearchStateMachine::run() {
    publishSearchPoints();
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
    } // switch
} // run()

NavState SearchStateMachine::executeSearch() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    Target const& leftTarget = env->getLeftTarget();
    Target const& rightTarget = env->getLeftTarget();
    Waypoint const& lastWaypoint = sm->getCourseState()->getLastCompletedWaypoint();
    bool isGate = lastWaypoint.gate;
    if (isGate) {
        if (env->hasGateLocation()) {
            return NavState::BeginGateSearch;
        } else {
            //only drive to the target if we don't have any posts
            if (!drivenToFirstPost) {
                if (leftTarget.id >= 0 || rightTarget.id >= 0) {
                    return NavState::DriveToTarget;
                }
            }
        }
    } else {
        // Either post works
        bool isWantedTarget = lastWaypoint.id == leftTarget.id || lastWaypoint.id + 1 == leftTarget.id;
        if (leftTarget.distance >= 0.0 && isWantedTarget) {
            return NavState::DriveToTarget;
        }
    }

    Odometry const& nextSearchPoint = mSearchPoints.front();
    double dt = sm->getDtSeconds();
    if (drivenToFirstPost) {
        std::cout << mSearchPoints.size() << std::endl;
    }
    if (rover->drive(nextSearchPoint, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
        mSearchPoints.pop_front();
        if (mSearchPoints.empty()) {
            return NavState::Done;
        }
    }
    return NavState::Search;
} // executeSearch()

NavState SearchStateMachine::executeDriveToTarget() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    double dt = sm->getDtSeconds();
    Waypoint lastWaypoint = sm->getCourseState()->getLastCompletedWaypoint();
    Target const& leftTarget = env->getLeftTarget();
    Target const& rightTarget = env->getRightTarget();
    double currentBearing = rover->odometry().bearing_deg;

    double distance, bearing;
    if (lastWaypoint.gate) {
        if (env->hasGateLocation()) {
            return NavState::BeginGateSearch;
        }
        if (leftTarget.id >= 0 && rightTarget.id >= 0) {
            if (leftTarget.distance < rightTarget.distance) {
                distance = leftTarget.distance;
                bearing = leftTarget.bearing + currentBearing;
            } else {
                distance = rightTarget.distance;
                bearing = rightTarget.bearing + currentBearing;
            }
        } else if (leftTarget.id >= 0) {
            distance = leftTarget.distance;
            bearing = leftTarget.bearing + currentBearing;
        } else if (rightTarget.id >= 0) {
            distance = rightTarget.distance;
            bearing = rightTarget.bearing + currentBearing;
        } else {
            std::cout << "lost target" << std::endl;
            return NavState::Search;
        }
    } else {
        if (leftTarget.id >= 0 && rightTarget.id >= 0) {
            if (leftTarget.id == lastWaypoint.id) {
                distance = leftTarget.distance;
                bearing = leftTarget.bearing + currentBearing;
            } else {
                distance = rightTarget.distance;
                bearing = rightTarget.bearing + currentBearing;
            }
        } else if (leftTarget.id >= 0) {
            distance = leftTarget.distance;
            bearing = leftTarget.bearing + currentBearing;
        } else {
            std::cerr << "Lost the target" << std::endl;
            return NavState::Search;
        }
    }

    Odometry targetPoint = createOdom(rover->odometry(), bearing, distance, rover);

    if (rover->drive(targetPoint, mConfig["navThresholds"]["targetDistance"].GetDouble(), dt)) {
        if (sm->getCourseState()->getLastCompletedWaypoint().gate) {
//                //We have to clear the search points and start a diamond search
//                //put this in some other function
//                mSearchPoints.clear();
//
//                Vector2d deltas[4] = {{0.5, 0.5}, {-0.5, 0.5}, {-0.5, -0.5}, {0.5, -0.5}};
//                double diamondDist = 3;
//                for (int i = 0; i < 4; ++i){
//                    deltas[i] /= deltas[i].norm();
//                    deltas[i] *= diamondDist;
//                    mSearchPoints.push_back(createOdom(rover->odometry(), deltas[i], rover));
//                }
//                drivenToFirstPost = true;
//                return NavState::Search;
            return NavState::Done;
        } else {
            return NavState::Done;
        }
    }

    return NavState::DriveToTarget;
} // executeDriveToTarget()

// Sends search path rover takes when trying to find posts
void SearchStateMachine::publishSearchPoints() {
    // Construct vector from deque
    std::vector<Odometry> arr(mSearchPoints.begin(), mSearchPoints.end());
    SearchPoints searchPoints{
        .search_pattern_size  = (int32_t)mSearchPoints.size(),
        .points = arr
    };

    std::string searchPointsChannel = mConfig["lcmChannels"]["searchPointsChannel"].GetString();
    mStateMachine.lock()->getLCM().publish(searchPointsChannel, &searchPoints);

} // publishSearchPoints()

// add intermediate points between the existing search points in a path generated by a search algorithm.
// The maximum separation between any points in the search point list is determined by the rover's sight distance.
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

// The search factory allows for the creation of search objects and
// an ease of transition between search algorithms
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

/******************/
/* TODOS */
/******************/
// save location of target then go around object? ( Execute drive to target )
// look into removing the waiting when turning to target or at least doing this a better way. This should at very least be it's own state
