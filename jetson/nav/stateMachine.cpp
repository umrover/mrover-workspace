#include "stateMachine.hpp"

#include <iostream>
#include <thread>
#include <unordered_map>
#include <utility>

#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "rover_msgs/NavStatus.hpp"
#include "utilities.hpp"

StateMachine::StateMachine(
        rapidjson::Document& config,
        std::shared_ptr<Rover> rover, std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseProgress,
        lcm::LCM& lcmObject) : mConfig(config),
                               mRover(move(rover)), mEnv(move(env)), mCourseProgress(move(courseProgress)), mLcmObject(lcmObject),
                               mTimePoint(std::chrono::high_resolution_clock::now()),
                               mPrevTimePoint(mTimePoint) {
    // TODO weak_from_this() should not be called in ctor, will always be null. Fine at the moment since obstacle detection is not used
    mObstacleAvoidanceStateMachine = ObstacleAvoiderFactory(weak_from_this(),
                                                            ObstacleAvoidanceAlgorithm::SimpleAvoidance, mRover,
                                                            mConfig);
}// StateMachine()

/**
 * Run a single iteration of the top level state machine.
 * Executes the correct function or sub state machine based on the current state machine to calculate the next state.
 * In this way the state machine can be seen as a tree, with search and gate search being subtrees.
 */
void StateMachine::run() {
    mPrevTimePoint = mTimePoint;
    auto now = std::chrono::high_resolution_clock::now();
    mTimePoint = now;

    // Diagnostic info: take average loop time
    // TODO move static to instance variable. Fine at the moment since we don't have multiple instances of the state machine
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

    // a timepoint but from the steady_clock
    auto steady_now = std::chrono::steady_clock::now();
    // updating recovery object
    if (mRover->autonState().enabled && mRover->currentState() != NavState::Done) {

        mRecovery->update(mRover->odometry(), steady_now, mRover->isTurning());

        if (mRecovery->isStuck()) {
            mRecovery->setRecoveryState(true);
            std::cout << "ROVER STUCK\n";
        }
    }


    if (mRover->autonState().enabled) {
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
        }// switch

        // Avoid old PID values from previous states
        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
            mRover->turningBearingPid().reset();
            mRover->drivingBearingPid().reset();
        }
    } else {
        mRecovery->reset();// TODO: possibly find another spot to put
        nextState = NavState::Off;
        mRover->setState(executeOff());
        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
        }
    }

    // TODO no longer needed after switching to waiting for LCM messages?
    std::this_thread::sleep_until(mTimePoint + LOOP_DURATION);
}// run()

/*** @brief Publishes the current navigation state to the nav status lcm channel. */
void StateMachine::publishNavState() const {
    NavStatus navStatus{
            .nav_state_name = stringifyNavState(),
            .completed_wps = static_cast<int32_t>(mCourseProgress->getRemainingWaypoints().size()),
            .total_wps = mCourseProgress->getCourse().num_waypoints};
    const std::string& navStatusChannel = mConfig["lcmChannels"]["navStatusChannel"].GetString();
    mLcmObject.publish(navStatusChannel, &navStatus);
}// publishNavState()

NavState StateMachine::executeOff() {
    if (mRover->autonState().enabled) {
        NavState nextState = mCourseProgress->getCourse().num_waypoints ? NavState::DriveWaypoints : NavState::Done;
        mCourseProgress->clearProgress();
        return nextState;
    }
    mRover->stop();
    return NavState::Off;
}// executeOff()

NavState StateMachine::executeDone() {
    mRecovery->reset();
    mRover->stop();
    return NavState::Done;
}// executeDone()

/**
 * Drive through the waypoints defined by course progress.
 *
 * @return Next state
 */
NavState StateMachine::executeDrive() {
    Waypoint const& currentWaypoint = mCourseProgress->getCurrentWaypoint();
    mEnv->setBaseGateID(currentWaypoint.id);
    mEnv->setShouldLookForGate(currentWaypoint.gate);
    double dt = getDtSeconds();

    // recovery functionality
    if (mRecovery->getRecoveryState()) {
        std::cout << "DriveWaypoints RECOVERY\n";
        // if no recovery path has been generated, generate one
        if (mRecovery->getRecoveryPath().empty()) {
            mRecovery->makeRecoveryPath(mRover->odometry(), *mRover);
        }
        if (mRecovery->getPointToFollow().backwards) {
            // drive to first point in recovery path (special drive backwards function)
            if (mRover->driveBackwards(mRecovery->getPointToFollow().odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                // mark point as complete and move to next point otherwise finish recovery manuever
                if (mRecovery->getRecoveryPath().size() > 1) {
                    mRecovery->completeCurrentRecoverypoint();
                } else {
                    // manuever is done
                    mRecovery->reset();
                    mRecovery->setRecoveryState(false);
                }
            }
        } else {
            // drive to first point in recovery path (drive forwards)
            if (mRover->drive(mRecovery->getPointToFollow().odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                // mark point as complete and move to next point otherwise finish recovery manuever
                if (mRecovery->getRecoveryPath().size() > 1) {
                    mRecovery->completeCurrentRecoverypoint();
                } else {
                    // manuever is done
                    mRecovery->reset();
                    mRecovery->setRecoveryState(false);
                }
            }
        }
        return NavState::DriveWaypoints;
    }

    if (mRover->drive(currentWaypoint.odom, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
        //messy but in a rush
        std::cout << "Completed waypoint" << std::endl;
        mCourseProgress->completeCurrentWaypoint();
        if (currentWaypoint.search) {
            return NavState::BeginSearch;
        } else if (mCourseProgress->getRemainingWaypoints().empty()) {
            return NavState::Done;
        }
    }
    double distanceToWaypoint = estimateDistance(currentWaypoint.odom, mRover->odometry());
    if (mEnv->hasPostOneLocation() && distanceToWaypoint <= mConfig["navThresholds"]["waypointRadius"].GetDouble()) {
        mCourseProgress->completeCurrentWaypoint();
        return NavState::BeginSearch;
    }
    return NavState::DriveWaypoints;
}// executeDrive()

std::string StateMachine::stringifyNavState() const {
    static const std::unordered_map<NavState, std::string> navStateNames =
            {
                    {NavState::Off, "Off"},
                    {NavState::Done, "Done"},
                    {NavState::DriveWaypoints, "Drive Waypoints"},
                    {NavState::BeginSearch, "Change Search Algorithm"},
                    {NavState::Search, "Search"},
                    {NavState::DriveToTarget, "Drive to Target"},
                    {NavState::TurnAroundObs, "Turn Around Obstacle"},
                    {NavState::DriveAroundObs, "Drive Around Obstacle"},
                    {NavState::SearchTurnAroundObs, "Search Turn Around Obstacle"},
                    {NavState::SearchDriveAroundObs, "Search Drive Around Obstacle"},
                    {NavState::BeginGateSearch, "Gate Prepare"},
                    {NavState::GateTraverse, "Gate Traverse"},

                    {NavState::Unknown, "Unknown"}};

    return navStateNames.at(mRover->currentState());
}// stringifyNavState()

void StateMachine::setSearcher(SearchType type) {
    mSearchStateMachine = SearchFactory(weak_from_this(), type, mRover, mConfig);
    mSearchStateMachine->initializeSearch(mConfig, mConfig["computerVision"]["visionDistance"].GetDouble());
}// setSearcher()

void StateMachine::setGateSearcher() {
    mGateStateMachine = GateFactory(weak_from_this(), mConfig);
}//setGateSearcher()

std::shared_ptr<Environment> StateMachine::getEnv() {
    return mEnv;
}//getEnv()

std::shared_ptr<CourseProgress> StateMachine::getCourseState() {
    return mCourseProgress;
}//getCourseState()

std::shared_ptr<Rover> StateMachine::getRover() {
    return mRover;
}//getRover()

lcm::LCM& StateMachine::getLCM() {
    return mLcmObject;
}//getLCM()

double StateMachine::getDtSeconds() {
    return std::chrono::duration<double>(mTimePoint - mPrevTimePoint).count();
}//getDtSeconds()

Recovery* StateMachine::getRecovery() {
    return mRecovery.get();
}//getRecovery()
