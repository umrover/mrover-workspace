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
    void update(Course const& course);

    std::deque<Waypoint> const& getRemainingWaypoints() const;

    void completeCurrentWaypoint();

    void clearProgress();

    Course const& getCourse() const;
};
