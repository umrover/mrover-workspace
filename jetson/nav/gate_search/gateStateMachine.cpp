#include "gateStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "./gate_search/diamondGateSearch.hpp"
#include <cmath>
#include <iostream>
#include <utility>

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig)
        : mStateMachine(move(stateMachine)), mRoverConfig(roverConfig) {}

GateStateMachine::~GateStateMachine() = default;

// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    auto rover = mStateMachine.lock()->getRover();
    switch (rover->currentState()) {
        case NavState::GateSpin: {
            return executeGateSpin();
        }

        case NavState::GateSpinWait: {
            return executeGateSpinWait();
        }

        case NavState::GateTurn: {
            return executeGateTurn();
        }

        case NavState::GateDrive: {
            return executeGateDrive();
        }

        case NavState::GateTurnToCentPoint: {
            return executeGateTurnToCentPoint();
        }

        case NavState::GateFace: {
            return executeGateFace();
        }

        case NavState::GateDriveToCentPoint: {
            return executeGateDriveToCentPoint();
        }

        case NavState::GateTurnToFarPost: {
            return executeGateTurnToFarPost();
        }

        case NavState::GateDriveToFarPost: {
            return executeGateDriveToFarPost();
        }

        case NavState::GateTurnToGateCenter: {
            return executeGateTurnToGateCenter();
        }

        case NavState::GateDriveThrough: {
            return executeGateDriveThrough();
        }

        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Perform spin search for a waypoint
NavState GateStateMachine::executeGateSpin() {
    auto rover = mStateMachine.lock()->getRover();
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig["search"]["searchWaitStepSize"].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call

    if (rover->rightCacheTarget().distance >= 0 ||
        (rover->leftCacheTarget().distance >= 0 && rover->leftCacheTarget().id != lastKnownRightPost.id)) {
        rover->resetMisses();
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if (nextStop == 0) {
        // get current angle and set as origAngle
        mOriginalSpinAngle = rover->odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if (rover->turn(nextStop)) {
        if (nextStop - mOriginalSpinAngle >= 360) {
            nextStop = 0;
            return NavState::GateTurn;
        }
        nextStop += waitStepSize;
        return NavState::GateSpinWait;
    }
    return NavState::GateSpin;
} // executeGateSpin()

// Wait for predetermined time before performing GateSpin
NavState GateStateMachine::executeGateSpinWait() {
    auto rover = mStateMachine.lock()->getRover();
    static bool started = false;
    static time_t startTime;

    if (rover->rightCacheTarget().distance >= 0 ||
        (rover->leftCacheTarget().distance >= 0 && rover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if (!started) {
        rover->stop();
        startTime = time(nullptr);
        started = true;
    }
    double waitTime = mRoverConfig["search"]["searchWaitTime"].GetDouble();
    if (difftime(time(nullptr), startTime) > waitTime) {
        started = false;
        return NavState::GateSpin;
    }
    return NavState::GateSpinWait;
} // executeGateSpinWait()

// Turn to determined waypoint
NavState GateStateMachine::executeGateTurn() {
    auto rover = mStateMachine.lock()->getRover();
    if (mGateSearchPoints.empty()) {
        initializeSearch();
    }

    if (rover->rightCacheTarget().distance >= 0 ||
        (rover->leftCacheTarget().distance >= 0 && rover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    Odometry& nextSearchPoint = mGateSearchPoints.front();
    if (rover->turn(nextSearchPoint)) {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateTurn()

// Drive to determined waypoint
NavState GateStateMachine::executeGateDrive() {
    auto rover = mStateMachine.lock()->getRover();
    if (rover->rightCacheTarget().distance >= 0 ||
        (rover->leftCacheTarget().distance >= 0 && rover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    const Odometry& nextSearchPoint = mGateSearchPoints.front();
    DriveStatus driveStatus = rover->drive(nextSearchPoint);

    if (driveStatus == DriveStatus::Arrived) {
        mGateSearchPoints.pop_front();
        return NavState::GateSpin;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateDrive()

// Turn to center of the two gate posts
NavState GateStateMachine::executeGateTurnToCentPoint() {
    if (mStateMachine.lock()->getRover()->turn(centerPoint1)) {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateTurnToCentPoint()

// Drive to the center point defined by the two posts
NavState GateStateMachine::executeGateDriveToCentPoint() {
    DriveStatus driveStatus = mStateMachine.lock()->getRover()->drive(centerPoint1);

    if (driveStatus == DriveStatus::Arrived) {
        return NavState::GateFace;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateDriveToCentPoint()

// Turn to the face of the gate posts 
NavState GateStateMachine::executeGateFace() {
    if (mStateMachine.lock()->getRover()->turn(centerPoint2)) {
        return NavState::GateTurnToFarPost;
    }
    return NavState::GateFace;
} // executeGateFace()

// Turn to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateTurnToFarPost() {
    auto rover = mStateMachine.lock()->getRover();
    if (rover->rightCacheTarget().distance > 0) {
        if (rover->leftCacheTarget().distance < rover->rightCacheTarget().distance) {
            if (rover->turn(rover->rightCacheTarget().bearing + rover->odometry().bearing_deg)) {
                return NavState::GateDriveToFarPost;
            }
        } else {
            if (rover->turn(rover->leftCacheTarget().bearing + rover->odometry().bearing_deg)) {
                return NavState::GateDriveToFarPost;
            }
        }
    } else {
        if (rover->turn(rover->leftCacheTarget().bearing + rover->odometry().bearing_deg)) {
            return NavState::GateDriveToFarPost;
        }
    }
    return NavState::GateTurnToFarPost;
} // executeGateTurnToFarPost()

// Drive to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateDriveToFarPost() {
    auto rover = mStateMachine.lock()->getRover();
    // Minor adjustment to gate targeting, due to issue of driving through a 
    // post when driving through the wrong direction
    double gateAdjustmentDist = mRoverConfig["gateAdjustment"]["adjustmentDistance"].GetDouble();

    // Set to first target, since we should have atleast one in sight/detected
    double distance = rover->leftCacheTarget().distance - gateAdjustmentDist;
    double bearing = rover->leftCacheTarget().bearing + rover->odometry().bearing_deg;

    if (rover->rightCacheTarget().distance > 0 &&
        rover->leftCacheTarget().distance < rover->rightCacheTarget().distance) {
        // Set our variables to drive to target/post 2, which is farther away
        distance = rover->rightCacheTarget().distance - gateAdjustmentDist;
        bearing = rover->rightCacheTarget().bearing + rover->odometry().bearing_deg;
    }

    DriveStatus driveStatus = rover->drive(distance, bearing, true);

    if (driveStatus == DriveStatus::Arrived) {
        return NavState::GateTurnToGateCenter;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveToFarPost;
    }
    return NavState::GateDriveToFarPost;
} // executeGateDriveToFarPost()

// Execute turn back to center point for driving through the gate
NavState GateStateMachine::executeGateTurnToGateCenter() {
    if (mStateMachine.lock()->getRover()->turn(centerPoint2)) {
        return NavState::GateDriveThrough;
    }
    return NavState::GateTurnToGateCenter;
} // executeGateTurnToGateCenter()

// Drive through gate posts
NavState GateStateMachine::executeGateDriveThrough() {
    DriveStatus driveStatus = mStateMachine.lock()->getRover()->drive(centerPoint2);

    if (driveStatus == DriveStatus::Arrived) {
        if (!isCorrectGateDir) // Check if we drove through the incorrect direction
        {
            const Odometry temp = centerPoint1;
            centerPoint1 = centerPoint2;
            centerPoint2 = temp;
            isCorrectGateDir = true;
            return NavState::GateSpin;
        }
        mStateMachine.lock()->getCourseState()->completeCurrentWaypoint();
        return NavState::Turn;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveThrough;
    }
    return NavState::GateDriveThrough;
} // executeGateDriveThrough()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info() {
    auto rover = mStateMachine.lock()->getRover();
    Odometry const& odometry = rover->odometry();
    if (rover->rightCacheTarget().distance >= 0 && rover->leftCacheTarget().id == lastKnownRightPost.id) {
        const double targetAbsAngle = mod(odometry.bearing_deg + rover->rightCacheTarget().bearing, 360);
        lastKnownLeftPost.odom = createOdom(odometry, targetAbsAngle, rover->rightCacheTarget().distance, rover);
        lastKnownLeftPost.id = rover->rightCacheTarget().id;
    } else {
        const double targetAbsAngle = mod(odometry.bearing_deg + rover->leftCacheTarget().bearing, 360);
        lastKnownLeftPost.odom = createOdom(odometry, targetAbsAngle, rover->leftCacheTarget().distance, rover);
        lastKnownLeftPost.id = rover->leftCacheTarget().id;
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint() {
    auto rover = mStateMachine.lock()->getRover();
    const Odometry& currOdom = rover->odometry();
    const double distFromGate = 3;
    const double gateWidth = mStateMachine.lock()->getCourseState()->getRemainingWaypoints().front().gate_width;
    const double tagToPointAngle = radianToDegree(atan2(distFromGate, gateWidth / 2));
    const double gateAngle = calcBearing(lastKnownRightPost.odom, lastKnownLeftPost.odom);
    const double absAngle1 = mod(gateAngle + tagToPointAngle, 360);
    const double absAngle2 = mod(absAngle1 + 180, 360);
    const double tagToPointDist = sqrt(pow(gateWidth / 2, 2) + pow(distFromGate, 2));

    // Assuming that CV works well enough that we don't pass through the gate before
    // finding the second post. Thus, centerPoint1 will always be closer.
    centerPoint1 = createOdom(lastKnownRightPost.odom, absAngle1, tagToPointDist, rover);
    centerPoint2 = createOdom(lastKnownLeftPost.odom, absAngle2, tagToPointDist, rover);
    const double cp1Dist = estimateNoneuclid(currOdom, centerPoint1);
    const double cp2Dist = estimateNoneuclid(currOdom, centerPoint2);
    if (lastKnownRightPost.id % 2) {
        isCorrectGateDir = true;
    } else {
        isCorrectGateDir = false;
    }
    if (cp1Dist > cp2Dist) {
        const Odometry temp = centerPoint1;
        centerPoint1 = centerPoint2;
        centerPoint2 = temp;
        isCorrectGateDir = !isCorrectGateDir;
    }

} // calcCenterPoint()

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig) {
    return std::make_shared<DiamondGateSearch>(stateMachine, roverConfig);
} // GateFactory()