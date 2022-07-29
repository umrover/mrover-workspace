#include "recovery.hpp"
#include "utilities.hpp"

Recovery::Recovery(std::chrono::duration<int> offset, double dispThresh,
                   std::chrono::duration<int> maxTurnTime) :
                    offsetAmount(offset), dispThresh(dispThresh), maxTurnTime(maxTurnTime) {

}//Recovery()

bool Recovery::isStuck(){
    // check displacement and or turn time to detect if rover is stuck
    // if turning and time_turning > maxTurnTime: true
    // if not turning and displacement < threshold: return true
    // else return false

    if (wasTurning){
        return getDuration(odoms.front().time,turnStartTime) > maxTurnTime;
    }
    else {

        // distance based
        double distance = estimateDistance(odoms.front().odom,odoms.back().odom);

        // note:  the displacement time stamp to statisfy condition must be >= offsetAmount
        // but we also clear all odoms > offsetAmount
        return (!odoms.empty() && getDuration(odoms.front().time,odoms.back().time) >= offsetAmount && distance < dispThresh);

    }



}//isStuck()

void Recovery::update(Odometry currOdom, time_point currTime, bool isTurning){
    // updates odoms deque with new data and remove older data
    // updates turning state (switch between time based and displacement based stuck detection)

    // TODO: add fail safe for overloading deque with data and making program fail
    //       may be better to just make a circular buffer instead of using deque

    // update turning state
    // if we were not turning but we are now turning
    if (!wasTurning && isTurning){
        wasTurning = true;
        turnStartTime = currTime;
    }

    // if we were turning and are now not turning
    else if (wasTurning && !isTurning){
        //reset displacement (as we were rotating not being displaced)
        wasTurning = false;
        odoms.clear();
    }

    // add latest odom
    OdomTimePoint newOdom = {currOdom,currTime};
    odoms.push_front(newOdom);

    // check old odoms for removal (if newer odoms that are within offset time)
    // check oldest odom (and second to last odom to see if we should remove the oldest odom)
    while ( odoms.size() > 2 && (getDuration(odoms.front().time,odoms[odoms.size()-2].time)) > offsetAmount){
        odoms.pop_back();
    }

}//update()


std::chrono::duration<int> Recovery::getDuration(time_point p1, time_point p2){
    // get duration between two points in seconds (rounded to nearest second)
    // let p1 be the newer of the time points and p2 be the later of the time points
    // for our use case
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(p1 - p2);
    return seconds;
}//getDuration

Odometry Recovery::getPointToFollow(){
    // returns the point the rover should be follwing for the recovery path
    return currentRecoverypoint;
}

void Recovery::makeRecoveryPath(const std::shared_ptr<Rover>& rover){
    // generate recovery path (J turn)
    // TODO: add different recovery paths or put in config

    Odometry cur = rover->odometry();

    double distBack = 4;
    double distBackLeft = 3;
    double distBackLeftLeft = 3;

    // Quick and dirty method for making points
    // can change later for less magic numbers (TODO)

    // point that is straight back (rover relative)
    Odometry back = createOdom(cur,mod(cur.bearing_deg+180,360),distBack,rover);

    // point that is further back and left (rover relative)
    Odometry backLeft = createOdom(back,mod(cur.bearing_deg+225,360),distBackLeft,rover);

    // another point that is back and further left (rover relative)
    Odometry backLeftLeft = createOdom(backLeft,mod(cur.bearing_deg+270,360),distBackLeftLeft,rover);

    mPath.clear();

    mPath.push_back(back);
    mPath.push_back(backLeft);
    mPath.push_back(backLeftLeft);

    currentRecoverypoint = mPath.front();

}

void Recovery::reset(){
    // reset the odom stream (deque)
    odoms.clear();
    // reset recovery state
    recoveryState = false;
    // reset mPath
    mPath.clear();
    // reset turning state
    wasTurning = false;
}

std::deque<Odometry> Recovery::getRecoveryPath(){
    // returns the deque of recovery odoms / path
    return mPath;
}

bool Recovery::getRecoveryState(){
    // returns if rover is in recovery or not
    return recoveryState;
}

void Recovery::setRecoveryState(bool state){
    // set the recovery state
    recoveryState = state;
}

void Recovery::completeCurrentRecoverypoint(){

    mPath.pop_front();

    if (!mPath.empty()){
        currentRecoverypoint = mPath.front();
    }

}
