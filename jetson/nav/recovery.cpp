#include "recovery.hpp"
#include "utilities.hpp"

Recovery::Recovery(int offset, double dispThresh) :
                    dispThresh(dispThresh){

    offset = std::chrono::seconds(offset);
}//Recovery()

bool Recovery::isStuck(int offset, double threshold){
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


std::chrono::duration Recovery::getDuration(time_point p1, time_point p2){
    // get duration between two points in seconds
    // let p1 be the newer of the time points and p2 be the later of the time points
    // for our use case
    return std::chrono::duration_cast<std::chrono::seconds>(p1-p2).count();
}//getDuration

