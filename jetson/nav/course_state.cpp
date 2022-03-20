#include "course_state.hpp"

void CourseProgress::update(Course const& course) {
    mCourse = course;
}

std::deque<Waypoint> const& CourseProgress::getRemainingWaypoints() const {
    return mRemainingWaypoints;
}

void CourseProgress::completeCurrentWaypoint() {
    mRemainingWaypoints.pop_front();
}

void CourseProgress::clearProgress() {
    mRemainingWaypoints.clear();
}

Course const& CourseProgress::getCourse() const {
    return mCourse;
}
