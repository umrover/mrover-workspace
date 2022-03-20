#pragma once

#include <deque>
#include "rover_msgs/Course.hpp"

using namespace rover_msgs;

class CourseState {
private:
    Course mCourse;
    std::deque<Waypoint> mPath;

public:
    void update(Course const& course);
};
