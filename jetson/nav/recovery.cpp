#include "recovery.hpp"
#include "utilities.hpp"

Recovery::Recovery(std::chrono::duration<int> offset, double dispThresh,
                   std::chrono::duration<int> maxTurnTime) : offsetAmount(offset), dispThresh(dispThresh), maxTurnTime(maxTurnTime) {

}// Recovery()

bool Recovery::isStuck() {
    // check displacement and or turn time to detect if rover is stuck
    // if turning and time_turning > maxTurnTime: true
    // if not turning and displacement < threshold: return true
    // else return false

    if (wasTurning) {
        return getDuration(odoms.front().time, turnStartTime) > maxTurnTime;
    } else {

        // distance based
        double distance = estimateDistance(odoms.front().odom, odoms.back().odom);

        // note:  the displacement time stamp to statisfy condition must be >= offsetAmount
        // but we also clear all odoms > offsetAmount
        return (!odoms.empty() && getDuration(odoms.front().time, odoms.back().time) >= offsetAmount && distance < dispThresh);
    }


}// isStuck()

void Recovery::update(Odometry currOdom, time_point currTime, bool isTurning) {
    // updates odoms deque with new data and remove older data
    // updates turning state (switch between time based and displacement based stuck detection)

    // TODO: add fail safe for overloading deque with data and making program fail
    //       may be better to just make a circular buffer instead of using deque

    // update turning state
    // if we were not turning but we are now turning
    if (!wasTurning && isTurning) {
        wasTurning = true;
        turnStartTime = currTime;
    }

    // if we were turning and are now not turning
    else if (wasTurning && !isTurning) {
        //reset displacement (as we were rotating not being displaced)
        wasTurning = false;
        odoms.clear();
    }

    // add latest odom
    OdomTimePoint newOdom = {currOdom, currTime};
    odoms.push_front(newOdom);

    // check old odoms for removal (if newer odoms that are within offset time)
    // check oldest odom (and second to last odom to see if we should remove the oldest odom)
    while (odoms.size() > 2 && (getDuration(odoms.front().time, odoms[odoms.size() - 2].time)) > offsetAmount) {
        odoms.pop_back();
    }

}// update()


// get duration between two points in seconds
// let p1 be the newer of the time points and p2 be the later of the time points
std::chrono::duration<int> Recovery::getDuration(time_point p1, time_point p2) {
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(p1 - p2);
    return seconds;
}// getDuration()

// returns the point the rover should be follwing for the recovery path
RecoveryPoint Recovery::getPointToFollow() {
    return currentRecoverypoint;
}// getPointToFollow()

// generate recovery path J turn
void Recovery::makeRecoveryPath(const std::shared_ptr<Rover>& rover) {
    // TODO: add different recovery paths or put in config

    Odometry cur = rover->odometry();

    double distP1 = 3;
    double distP2 = 2;
    double distP3 = 2;
    double distP4 = 2;

    // Quick and dirty method for making points
    // can change later for less magic numbers (TODO)

    // point that is straight back (rover relative)
    RecoveryPoint back = makeRecoveryPoint(cur, mod(cur.bearing_deg + 180, 360), distP1, rover, true);

    // point that is further back and left (rover relative)
    RecoveryPoint backLeft = makeRecoveryPoint(back.odom, mod(cur.bearing_deg + 200, 360), distP2, rover, true);

    // another point that is back and further left (rover relative)
    RecoveryPoint backLeftLeft = makeRecoveryPoint(backLeft.odom, mod(cur.bearing_deg + 240, 360), distP3, rover, true);

    // another point that is back and even further left (rover relative)
    RecoveryPoint backLeftLeftLeft = makeRecoveryPoint(backLeftLeft.odom, mod(cur.bearing_deg + 280, 360), distP4, rover, true);

    mPath.clear();

    mPath.push_back(back);
    mPath.push_back(backLeft);
    mPath.push_back(backLeftLeft);
    mPath.push_back(backLeftLeftLeft);

    currentRecoverypoint = mPath.front();
}// makeRecoveryPath()

// make recovery path J turn
// (used when driving to target so that we drive back to the target later)
void Recovery::makeTargetRecoveryPath(const std::shared_ptr<Rover>& rover, Target const& target) {

    Odometry cur = rover->odometry();

    double distP1 = 3;
    double distP2 = 2;
    double distP3 = 2;
    double distP4 = 2;

    // Quick and dirty method for making points
    // can change later for less magic numbers (TODO)

    // point that is straight back (rover relative)
    RecoveryPoint back = makeRecoveryPoint(cur, mod(cur.bearing_deg + 180, 360), distP1, rover, true);

    // point that is further back and left (rover relative)
    RecoveryPoint backLeft = makeRecoveryPoint(back.odom, mod(cur.bearing_deg + 200, 360), distP2, rover, true);

    // another point that is back and further left (rover relative)
    RecoveryPoint backLeftLeft = makeRecoveryPoint(backLeft.odom, mod(cur.bearing_deg + 240, 360), distP3, rover, true);

    // another point that is back and even further left (rover relative)
    RecoveryPoint backLeftLeftLeft = makeRecoveryPoint(backLeftLeft.odom, mod(cur.bearing_deg + 280, 360), distP4, rover, true);

    // bearing to target
    double bearing = mod(rover->odometry().bearing_deg + target.bearing, 360);

    // add two points to place the rover back in front of the target but from the other side
    // first target rendezvous point
    double distR1 = 4;
    double distR2 = 1;

    Odometry targetOdom = createOdom(cur, bearing, target.distance, rover);

    RecoveryPoint targetRendevous1 = makeRecoveryPoint(targetOdom, mod(bearing - 90, 360), distR1, rover, false);

    // second target rendezvous point
    RecoveryPoint targetRendevous2 = makeRecoveryPoint(targetOdom, mod(bearing - 90, 360), distR2, rover, false);

    mPath.clear();

    mPath.push_back(back);
    mPath.push_back(backLeft);
    mPath.push_back(backLeftLeft);
    mPath.push_back(backLeftLeftLeft);
    mPath.push_back(targetRendevous1);
    mPath.push_back(targetRendevous2);

    currentRecoverypoint = mPath.front();
}// makeTargetRecoveryPath()

// resets the recovery object
void Recovery::reset() {
    // reset the odom stream (deque)
    odoms.clear();
    // reset recovery state
    recoveryState = false;
    // reset mPath
    mPath.clear();
    // reset turning state
    wasTurning = false;
}// reset()

// returns the deque of recovery odoms / path
std::deque<RecoveryPoint> Recovery::getRecoveryPath() {
    return mPath;
}// getRecoveryPath()

// returns if rover is in recovery or not
bool Recovery::getRecoveryState() {
    return recoveryState;
}

// set the recovery state
void Recovery::setRecoveryState(bool state) {
    recoveryState = state;
}// setRecoveryState()

void Recovery::completeCurrentRecoverypoint() {
    mPath.pop_front();

    if (!mPath.empty()) {
        currentRecoverypoint = mPath.front();
    }
}// completeCurrentRecoverypoint()

RecoveryPoint Recovery::makeRecoveryPoint(const Odometry& current, double absoluteBearing, double distance,
                                          const std::shared_ptr<Rover>& rover, bool backwards) {

    Odometry odom = createOdom(current, absoluteBearing, distance, rover);
    RecoveryPoint point = {odom, backwards};
    return point;
}// makeRecoveryPoint()