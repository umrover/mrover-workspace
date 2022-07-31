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
using time_point = std::chrono::high_resolution_clock::time_point;

struct OdomTimePoint {
    Odometry odom;
    time_point time;
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
    Recovery(std::chrono::duration<int> offsetAmount, double dispThresh,
             std::chrono::duration<int> maxTurnTime);

    bool isStuck();

    void update(Odometry currOdom, time_point currTime, bool isTurning);

    void makeRecoveryPath(const std::shared_ptr<Rover>& rover);

    void makeTargetRecoveryPath(const std::shared_ptr<Rover>& rover, Target const& target);

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

    std::chrono::duration<int> getDuration(time_point p1, time_point p2);

    RecoveryPoint makeRecoveryPoint(const Odometry& current, double absoluteBearing, double distance,
                                    const std::shared_ptr<Rover>& rover, bool backwards);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // how long in seconds between the offset/lagging odom
    // and the current odom
    std::chrono::duration<int> offsetAmount;

    // what minimum threshold in displacement
    // before we say rover is stuck (in meters? TODO)
    double dispThresh;

    // maximum turn time in seconds before rover will attempt to recover
    std::chrono::duration<int> maxTurnTime;

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
    time_point turnStartTime;
};
