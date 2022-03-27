#include "gateStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "./gate_search/diamondGateSearch.hpp"
#include <cmath>
#include <iostream>
#include <utility>

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(weak_ptr<StateMachine> stateMachine, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
        : mStateMachine(move(stateMachine)), mRoverConfig(roverConfig), mRover(move(rover)) {}

GateStateMachine::~GateStateMachine() = default;

// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    switch (mRover->currentState()) {
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
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Perform spin search for a waypoint
NavState GateStateMachine::executeGateSpin() {
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig["search"]["searchWaitStepSize"].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call

    if (mRover->rightCacheTarget().distance >= 0 ||
        (mRover->leftCacheTarget().distance >= 0 && mRover->leftCacheTarget().id != lastKnownRightPost.id)) {
        mRover->resetMisses();
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if (nextStop == 0) {
        // get current angle and set as origAngle
        mOriginalSpinAngle = mRover->odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if (mRover->turn(nextStop)) {
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
    static bool started = false;
    static time_t startTime;

    if (mRover->rightCacheTarget().distance >= 0 ||
        (mRover->leftCacheTarget().distance >= 0 && mRover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if (!started) {
        mRover->stop();
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
    if (mGateSearchPoints.empty()) {
        initializeSearch();
    }

    if (mRover->rightCacheTarget().distance >= 0 ||
        (mRover->leftCacheTarget().distance >= 0 && mRover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    Odometry& nextSearchPoint = mGateSearchPoints.front();
    if (mRover->turn(nextSearchPoint)) {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateTurn()

// Drive to determined waypoint
NavState GateStateMachine::executeGateDrive() {
    if (mRover->rightCacheTarget().distance >= 0 ||
        (mRover->leftCacheTarget().distance >= 0 && mRover->leftCacheTarget().id != lastKnownRightPost.id)) {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    const Odometry& nextSearchPoint = mGateSearchPoints.front();
    DriveStatus driveStatus = mRover->drive(nextSearchPoint);

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
    if (mRover->turn(centerPoint1)) {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateTurnToCentPoint()

// Drive to the center point defined by the two posts
NavState GateStateMachine::executeGateDriveToCentPoint() {
    DriveStatus driveStatus = mRover->drive(centerPoint1);

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
    if (mRover->turn(centerPoint2)) {
        return NavState::GateTurnToFarPost;
    }
    return NavState::GateFace;
} // executeGateFace()

// Turn to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateTurnToFarPost() {
    if (mRover->rightCacheTarget().distance > 0) {
        if (mRover->leftCacheTarget().distance < mRover->rightCacheTarget().distance) {
            if (mRover->turn(mRover->rightCacheTarget().bearing + mRover->odometry().bearing_deg)) {
                return NavState::GateDriveToFarPost;
            }
        } else {
            if (mRover->turn(mRover->leftCacheTarget().bearing + mRover->odometry().bearing_deg)) {
                return NavState::GateDriveToFarPost;
            }
        }
    } else {
        if (mRover->turn(mRover->leftCacheTarget().bearing + mRover->odometry().bearing_deg)) {
            return NavState::GateDriveToFarPost;
        }
    }
    return NavState::GateTurnToFarPost;
} // executeGateTurnToFarPost()

// Drive to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateDriveToFarPost() {
    // Minor adjustment to gate targeting, due to issue of driving through a 
    // post when driving through the wrong direction
    double gateAdjustmentDist = mRoverConfig["gateAdjustment"]["adjustmentDistance"].GetDouble();

    // Set to first target, since we should have atleast one in sight/detected
    double distance = mRover->leftCacheTarget().distance - gateAdjustmentDist;
    double bearing = mRover->leftCacheTarget().bearing + mRover->odometry().bearing_deg;

    if (mRover->rightCacheTarget().distance > 0 &&
        mRover->leftCacheTarget().distance < mRover->rightCacheTarget().distance) {
        // Set our variables to drive to target/post 2, which is farther away
        distance = mRover->rightCacheTarget().distance - gateAdjustmentDist;
        bearing = mRover->rightCacheTarget().bearing + mRover->odometry().bearing_deg;
    }

    DriveStatus driveStatus = mRover->drive(distance, bearing, true);

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
    if (mRover->turn(centerPoint2)) {
        return NavState::GateDriveThrough;
    }
    return NavState::GateTurnToGateCenter;
} // executeGateTurnToGateCenter()

// Drive through gate posts
NavState GateStateMachine::executeGateDriveThrough() {
    DriveStatus driveStatus = mRover->drive(centerPoint2);

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
        mStateMachine.lock()->updateCompletedPoints();
        return NavState::Turn;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::GateDriveThrough;
    }
    return NavState::GateDriveThrough;
} // executeGateDriveThrough()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info() {
    Odometry const& odometry = mRover->odometry();
    if (mRover->rightCacheTarget().distance >= 0 && mRover->leftCacheTarget().id == lastKnownRightPost.id) {
        const double targetAbsAngle = mod(odometry.bearing_deg + mRover->rightCacheTarget().bearing, 360);
        lastKnownLeftPost.odom = createOdom(odometry, targetAbsAngle, mRover->rightCacheTarget().distance, mRover);
        lastKnownLeftPost.id = mRover->rightCacheTarget().id;
    } else {
        const double targetAbsAngle = mod(odometry.bearing_deg + mRover->leftCacheTarget().bearing, 360);
        lastKnownLeftPost.odom = createOdom(odometry, targetAbsAngle, mRover->leftCacheTarget().distance, mRover);
        lastKnownLeftPost.id = mRover->leftCacheTarget().id;
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint() {
    const Odometry& currOdom = mRover->odometry();
    const double distFromGate = 3;
    const double gateWidth = mStateMachine.lock()->getCourseState()->getRemainingWaypoints().front().gate_width;
    const double tagToPointAngle = radianToDegree(atan2(distFromGate, gateWidth / 2));
    const double gateAngle = calcBearing(lastKnownRightPost.odom, lastKnownLeftPost.odom);
    const double absAngle1 = mod(gateAngle + tagToPointAngle, 360);
    const double absAngle2 = mod(absAngle1 + 180, 360);
    const double tagToPointDist = sqrt(pow(gateWidth / 2, 2) + pow(distFromGate, 2));

    // Assuming that CV works well enough that we don't pass through the gate before
    // finding the second post. Thus, centerPoint1 will always be closer.
    centerPoint1 = createOdom(lastKnownRightPost.odom, absAngle1, tagToPointDist, mRover);
    centerPoint2 = createOdom(lastKnownLeftPost.odom, absAngle2, tagToPointDist, mRover);
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
shared_ptr<GateStateMachine> GateFactory(weak_ptr<StateMachine> stateMachine, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) {
    return make_shared<DiamondGateSearch>(stateMachine, rover, roverConfig);
} // GateFactory()