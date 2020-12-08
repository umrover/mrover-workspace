#include "pid.hpp"

PidLoop::PidLoop(double Kp, double Ki, double Kd) :
    Kp_(Kp),
    Ki_(Ki),
    Kd_(Kd),
    first_(true),
    accumulated_error_(0.0),
    last_error_(0.0)
{
}

double PidLoop::update(double current, double desired) {
    double err = this->error(current, desired);
    accumulated_error_ += err;
    double effort = Kp_*err + Ki_*accumulated_error_;
    if (!first_) {
        effort += Kd_*(err - last_error_);
    }
    last_error_ = err;
    first_ = false;

    if (effort < sat_min_out_) effort = sat_min_out_;
    if (effort > sat_max_out_) effort = sat_max_out_;

    return effort;
}

void PidLoop::reset() {
    first_ = true;
    accumulated_error_ = 0.0;
    last_error_ = 0.0;
}

double PidLoop::error(double current, double desired) {
    // TODO add support for modular PID here
    return desired - current;
}
