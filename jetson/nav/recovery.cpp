#include "recovery.hpp"
#include "utilities.hpp"

Recovery::Recovery(int offset, double dispThresh) :
                    offsetAmount(offset), dispThresh(dispThresh){

}//Recovery()

bool Recovery::isStuck(){
    // check displacement (TODO: have condition for if we are turning! or just don't call if we are turning?)
    // if displacement < threshold: return true
    // else return false

    double distance = estimateDistance(odoms.front().odom,odoms.back().odom);
    if (!odoms.empty() && getDuration(odoms.front().time,odoms.back().time) >= offsetAmount && distance < dispThresh){
        return true;
    }
    else {
        return false;
    }


}//isStuck()

void Recovery::update(Odometry currOdom, time_point currTime){
    // updates odoms deque with new data
    // and remove older data

    // add latest odom
    OdomTimePoint newOdom = {currOdom,currTime};
    odoms.push_front(newOdom);

    // check old odoms for removal (if newer odoms that are within offset time)
    // check oldest odom (and second to last odom to see if we should remove the oldest odom)
    while ( odoms.size() > 2 && (getDuration(odoms.front().time,odoms[odoms.size()-2].time)) > offsetAmount){
        odoms.pop_back();
    }

}//update()


int Recovery::getDuration(time_point p1, time_point p2){
    // get duration between two points in seconds (rounded to nearest second)
    // let p1 be the newer of the time points and p2 be the later of the time points
    // for our use case
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(p1 - p2);
    return int((milliseconds.count()/1000.0) + 0.5);
}//getDuration

Odometry Recovery::getPointToFollow(){
    return mPath[mPathIndex];
}

void Recovery::makeRecoveryPath(const std::shared_ptr<Rover>& rover){
    // generate recovery path (J turn)
    // TODO: add different recovery paths or put in config

    Odometry cur = rover->odometry();

    double distBack = 3;
    double distBackLeft = 2;
    double distBackLeftLeft = 2;

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

}