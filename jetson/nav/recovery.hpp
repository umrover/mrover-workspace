#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <memory>

#include <eigen3/Eigen/Core>

#include "rover.hpp"
#include "rover_msgs/Odometry.hpp"

using Eigen::Vector2d;
using namespace rover_msgs;
using duration =  std::chrono::duration<long long>;
using time_point_steady = std::chrono::steady_clock::time_point;

struct OdomTimePoint {
    Odometry odom;
    time_point_steady time;
};

struct RecoveryPoint {
    Odometry odom;
    // if rover should drive backwards to point
    bool backwards;
};


/***
 * Recovery Watchdog
 * Class used to detect if Rover has become stuck
 * and generate recovery manuevers to get unstuck.
 */
class Recovery {
public:
    Recovery(duration offsetAmount, double dispThresh, duration maxTurnTime);

    bool isStuck();

    void update(Odometry currOdom, time_point_steady currTime, bool isTurning);

    void makeRecoveryPath(const Odometry & currOdom, const Rover & rover);

    void makeTargetRecoveryPath(const Odometry & currOdom, Target const& target, const Rover & rover);

    void reset();

    std::deque<RecoveryPoint> getRecoveryPath();

    RecoveryPoint getPointToFollow();

    bool getRecoveryState();

    void setRecoveryState(bool state);

    void completeCurrentRecoverypoint();

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/

    duration getDuration(time_point_steady p1, time_point_steady p2);

    RecoveryPoint makeRecoveryPoint(const Odometry& current, double absoluteBearing, double distance,
                                    const Rover & rover, bool backwards);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // how long in seconds between the offset/lagging odom
    // and the current odom
    duration offsetAmount;

    // what minimum threshold in displacement
    // before we say rover is stuck (in meters? TODO)
    double dispThresh;

    // maximum turn time in seconds before rover will attempt to recover
    duration maxTurnTime;

    // deque of running odoms (front is the more recent odoms)
    std::deque<OdomTimePoint> odoms;

    // recovery points
    std::deque<RecoveryPoint> mPath;

    // current recovery point to follow
    RecoveryPoint currentRecoverypoint;

    // whether the rover is currently in recovery
    // meaning: if we are following the recovery path (after detecting being stuck)
    // false -> not stuck and not recovering
    // true -> were or are stuck and are trying to recover
    bool recoveryState = false;

    // state if the rover was turning
    bool wasTurning = false;

    // mark the start time of a turn
    // if __ time passes and rover is still turning attempt recovery
    time_point_steady turnStartTime;
};
