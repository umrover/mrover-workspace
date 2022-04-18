#include "stateMachine.hpp"

#include <thread>
#include <utility>
#include <iostream>
#include <unordered_map>

#include "utilities.hpp"
#include "rover_msgs/NavStatus.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuration file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine(
        rapidjson::Document& config,
        std::shared_ptr<Rover> rover, std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseProgress,
        lcm::LCM& lcmObject
) : mConfig(config),
    mRover(move(rover)), mEnv(move(env)), mCourseProgress(move(courseProgress)), mLcmObject(lcmObject),
    mTimePoint(std::chrono::high_resolution_clock::now()),
    mPrevTimePoint(mTimePoint) {
    // TODO: fix, weak_from_this() should not be called in ctor, will always be null
    mObstacleAvoidanceStateMachine = ObstacleAvoiderFactory(weak_from_this(),
                                                            ObstacleAvoidanceAlgorithm::SimpleAvoidance, mRover,
                                                            mConfig);
} // StateMachine()

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run() {
    mPrevTimePoint = mTimePoint;
    auto now = std::chrono::high_resolution_clock::now();
    mTimePoint = now;

    // Diagnostic info: take average loop time
    static std::array<double, 256> readings{};
    static int i = 0;
    readings[i] = getDtSeconds();
    if (++i % readings.size() == 0) {
        double avgDt = std::accumulate(readings.begin(), readings.end(), 0.0) / readings.size();
        std::cout << "Update rate: " << 1.0 / avgDt << std::endl;
        i = 0;
    }

    mEnv->updatePost(mRover, mCourseProgress);

    publishNavState();
    NavState nextState = NavState::Unknown;

    if (mRover->autonState().is_auton) {
        switch (mRover->currentState()) {
            case NavState::Off: {
                nextState = executeOff();
                break;
            }

            case NavState::Done: {
                nextState = executeDone();
                break;
            }

            case NavState::DriveWaypoints: {
                nextState = executeDrive();
                break;
            }

            case NavState::Search:
            case NavState::DriveToTarget: {
                nextState = mSearchStateMachine->run();
                break;
            }

            case NavState::TurnAroundObs:
            case NavState::SearchTurnAroundObs:
            case NavState::DriveAroundObs:
            case NavState::SearchDriveAroundObs: {
                nextState = mObstacleAvoidanceStateMachine->run();
                break;
            }

            case NavState::BeginSearch: {
                setSearcher(SearchType::FROM_PATH_FILE);
                nextState = NavState::Search;
                break;
            }

            case NavState::BeginGateSearch: {
                setGateSearcher();
                nextState = mGateStateMachine->run();
                break;
            }
            case NavState::GateTraverse: {
                nextState = mGateStateMachine->run();
                break;
            }

            case NavState::Unknown: {
                throw std::runtime_error("Entered unknown state.");
            }
        } // switch

        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
            mRover->bearingPid().reset();
        }
    } else {
        nextState = NavState::Off;
        mRover->setState(executeOff()); // Turn off immediately
        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
        }
    }

    std::this_thread::sleep_until(mTimePoint + LOOP_DURATION);
} // run()

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const {
    NavStatus navStatus{
            .nav_state_name = stringifyNavState(),
            .completed_wps = static_cast<int32_t>(mCourseProgress->getRemainingWaypoints().size()),
            .total_wps = mCourseProgress->getCourse().num_waypoints
    };
    const std::string& navStatusChannel = mConfig["lcmChannels"]["navStatusChannel"].GetString();
    mLcmObject.publish(navStatusChannel, &navStatus);
} // publishNavState()

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoint. Else the
// rover is still off.
NavState StateMachine::executeOff() {
    if (mRover->autonState().is_auton) {
        NavState nextState = mCourseProgress->getCourse().num_waypoints ? NavState::DriveWaypoints : NavState::Done;
        mCourseProgress->clearProgress();
        return nextState;
    }
    mRover->stop();
    return NavState::Off;
} // executeOff()

// Executes the logic for the done state. Stops and turns off the rover.
NavState StateMachine::executeDone() {
    mRover->stop();
    return NavState::Done;
} // executeDone()

/**
 * Drive through the waypoints defined by course progress.
 *
 * @return Next state
 */
NavState StateMachine::executeDrive() {
    Waypoint const& currentWaypoint = mCourseProgress->getCurrentWaypoint();
    mEnv->setBaseGateID(currentWaypoint.id);
    double dt = getDtSeconds();
    if (mRover->drive(currentWaypoint.odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
        mCourseProgress->completeCurrentWaypoint();
        std::cout << "Completed waypoint" << std::endl;
        if (currentWaypoint.search) {
            return NavState::BeginSearch;
        } else if (mCourseProgress->getRemainingWaypoints().empty()) {
            return NavState::Done;
        }
    }
    return NavState::DriveWaypoints;
} // executeDrive()

// Gets the string representation of a nav state.
std::string StateMachine::stringifyNavState() const {
    static const std::unordered_map<NavState, std::string> navStateNames =
            {
                    {NavState::Off,                  "Off"},
                    {NavState::Done,                 "Done"},
                    {NavState::DriveWaypoints,       "Drive Waypoints"},
                    {NavState::BeginSearch,          "Change Search Algorithm"},
                    {NavState::Search,               "Search"},
                    {NavState::DriveToTarget,        "Drive to Target"},
                    {NavState::TurnAroundObs,        "Turn Around Obstacle"},
                    {NavState::DriveAroundObs,       "Drive Around Obstacle"},
                    {NavState::SearchTurnAroundObs,  "Search Turn Around Obstacle"},
                    {NavState::SearchDriveAroundObs, "Search Drive Around Obstacle"},
                    {NavState::BeginGateSearch,      "Gate Prepare"},
                    {NavState::GateTraverse,         "Gate Traverse"},

                    {NavState::Unknown,              "Unknown"}
            };

    return navStateNames.at(mRover->currentState());
} // stringifyNavState()

void StateMachine::setSearcher(SearchType type) {
    mSearchStateMachine = SearchFactory(weak_from_this(), type, mRover, mConfig);
    mSearchStateMachine->initializeSearch(mConfig, mConfig["computerVision"]["visionDistance"].GetDouble());
}

void StateMachine::setGateSearcher() {
    mGateStateMachine = GateFactory(weak_from_this(), mConfig);
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleDistance(double distance) {
    mObstacleAvoidanceStateMachine->updateObstacleDistance(distance);
}

std::shared_ptr<Environment> StateMachine::getEnv() {
    return mEnv;
}

std::shared_ptr<CourseProgress> StateMachine::getCourseState() {
    return mCourseProgress;
}

std::shared_ptr<Rover> StateMachine::getRover() {
    return mRover;
}

lcm::LCM& StateMachine::getLCM() {
    return mLcmObject;
}

double StateMachine::getDtSeconds() {
    return std::chrono::duration<double>(mTimePoint - mPrevTimePoint).count();
}
