
#include <chrono>
#include <ctime>
#include <deque>

#include "rover_msgs/Odometry.hpp"

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
    Recovery(int offsetAmount, double dispThresh);

    bool isStuck();

    void update(Odometry currOdom, time_point currTime);



private:
    // how long in seconds between the offset/lagging odom
    // and the current odom
    std::chrono::duration offsetAmount;

    // what minimum threshold in displacement
    // before we say rover is stuck (in meters? TODO)
    double dispThresh;

    // deque of running odoms
    std::deque<OdomTimePoint> odoms;

    std::chrono::duration getDuration(time_point p1, time_point p2);
};
