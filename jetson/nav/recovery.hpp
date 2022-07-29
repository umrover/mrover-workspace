#include <chrono>
#include <ctime>
#include <deque>
#include <cmath>
#include <memory>

#include <eigen3/Eigen/Core>

#include "rover_msgs/Odometry.hpp"
#include "rover.hpp"

using Eigen::Vector2d;
using namespace rover_msgs;
using time_point = std::chrono::high_resolution_clock::time_point;

struct OdomTimePoint{
    Odometry odom;
    time_point time;
};


/***
 * Recovery Watchdog
 * Class used to detect if Rover has become stuck
 * and generate recovery manuevers to get unstuck.
 */
class Recovery {
public:
    Recovery(std::chrono::duration<int> offsetAmount, double dispThresh,
             std::chrono::duration<int> maxTurnTime);

    bool isStuck();

    void update(Odometry currOdom, time_point currTime, bool isTurning);

    void makeRecoveryPath(const std::shared_ptr<Rover>& rover);

    void reset();

    std::deque<Odometry> getRecoveryPath();

    Odometry getPointToFollow();

    bool getRecoveryState();

    void setRecoveryState(bool state);

    void completeCurrentRecoverypoint();

private:
    std::chrono::duration<int> getDuration(time_point p1, time_point p2);


    // how long in seconds between the offset/lagging odom
    // and the current odom
    std::chrono::duration<int> offsetAmount;

    // maximum turn time in seconds before rover will attempt to recover
    std::chrono::duration<int> maxTurnTime;

    // what minimum threshold in displacement
    // before we say rover is stuck (in meters? TODO)
    double dispThresh;

    // deque of running odoms (front is the more recent odoms)
    std::deque<OdomTimePoint> odoms;

    // recovery points
    std::deque<Odometry> mPath;

    Odometry currentRecoverypoint;

    // whether the rover is currently in recovery
    // meaning: if we are following the recovery path (after detecting being stuck)
    // false -> not stuck and not recovering
    // true -> were or are stuck and are trying to recover
    bool recoveryState = false;

    // state if the rover was turning
    bool wasTurning = false;
    // mark the start time of a turn
    // if __ time passes and rover is still turning attempt recovery
    time_point turnStartTime;
};
