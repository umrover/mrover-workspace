#include "courseProgress.hpp"

void CourseProgress::setCourse(Course const& course) {
    bool isDifferent = course.hash != mCourse.hash;
    if (isDifferent) {
        mCourse = course;
        clearProgress();
    }
}// setCourse()

std::deque<Waypoint> const& CourseProgress::getRemainingWaypoints() const {
    return mRemainingWaypoints;
}// getRemainingWaypoints()

Waypoint const& CourseProgress::getCurrentWaypoint() const {
    return mRemainingWaypoints.front();
}// getCurrentWaypoint()

Waypoint CourseProgress::completeCurrentWaypoint() {
    Waypoint waypoint = mRemainingWaypoints.front();
    mRemainingWaypoints.pop_front();
    return waypoint;
}// completeCurrentWaypoint()

void CourseProgress::clearProgress() {
    mRemainingWaypoints.assign(mCourse.waypoints.begin(), mCourse.waypoints.end());
}// clearProgress()

Course const& CourseProgress::getCourse() const {
    return mCourse;
}// getCourse()

int32_t CourseProgress::getCompletedWaypointCount() const {
    return static_cast<int32_t>(mCourse.num_waypoints - mRemainingWaypoints.size());
}// getCompletedWaypointCount()

Waypoint const& CourseProgress::getLastCompletedWaypoint() const {
    return mCourse.waypoints.at(getCompletedWaypointCount() - 1);
}// getLastCompletedWaypoint()
