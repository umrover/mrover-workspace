#pragma once

#include <deque>
#include "rover_msgs/Course.hpp"


using namespace rover_msgs;

/**
 * @brief Represents the state of the rover traversing a course.
 */
class CourseProgress {
private:
    Course mCourse;
    std::deque<Waypoint> mRemainingWaypoints;

public:
    void setCourse(Course const& course);

    Waypoint completeCurrentWaypoint();

    void clearProgress();

    [[nodiscard]] std::deque<Waypoint> const& getRemainingWaypoints() const;

    [[nodiscard]] Waypoint const& getCurrentWaypoint() const;

    [[nodiscard]] Waypoint const& getLastCompletedWaypoint() const;

    [[nodiscard]] Course const& getCourse() const;

    [[nodiscard]] int32_t getCompletedWaypointCount() const;
};
