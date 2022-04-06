#pragma once

#include <deque>
#include "rover_msgs/Course.hpp"


using namespace rover_msgs;

/**
 * @brief Represents the state of us traversing a course.
 */
class CourseProgress {
private:
    Course mCourse;
    std::deque<Waypoint> mRemainingWaypoints;

public:
    void setCourse(Course const& course);

    std::deque<Waypoint> const& getRemainingWaypoints() const;

    Waypoint completeCurrentWaypoint();

    void clearProgress();

    Course const& getCourse() const;

    int32_t getCompletedWaypointCount() const;
};
