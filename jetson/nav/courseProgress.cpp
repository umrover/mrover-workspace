#include "courseProgress.hpp"

void CourseProgress::setCourse(Course const& course) {
    if (course.hash != mCourse.hash) {
        mCourse = course;
        clearProgress();
    }
}

std::deque<Waypoint> const& CourseProgress::getRemainingWaypoints() const {
    return mRemainingWaypoints;
}

Waypoint const& CourseProgress::getNextWaypoint() const {
    return mRemainingWaypoints.front();
}

Waypoint CourseProgress::completeCurrentWaypoint() {
    Waypoint waypoint = mRemainingWaypoints.front();
    mRemainingWaypoints.pop_front();
    return waypoint;
}

void CourseProgress::clearProgress() {
    mRemainingWaypoints.assign(mCourse.waypoints.begin(), mCourse.waypoints.end());
}

Course const& CourseProgress::getCourse() const {
    return mCourse;
}

int32_t CourseProgress::getCompletedWaypointCount() const {
    return static_cast<int32_t>(mCourse.num_waypoints - mRemainingWaypoints.size());
}

Waypoint const& CourseProgress::getLastCompletedWaypoint() const {
    return mCourse.waypoints.at(getCompletedWaypointCount() - 1);
}
