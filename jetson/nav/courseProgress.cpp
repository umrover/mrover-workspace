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

void CourseProgress::completeCurrentWaypoint() {
    mRemainingWaypoints.pop_front();
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