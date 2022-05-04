#include "pid.hpp"

#include "utilities.hpp"

#include <cmath>

PidLoop::PidLoop(double p, double i, double d) :
        mP(p),
        mI(i),
        mD(d) {
}

double PidLoop::error(double current, double desired) const {
    if (mMaxInputBeforeWrap) {
        double maxInput = mMaxInputBeforeWrap.value();
        current = mod(current, maxInput);
        desired = mod(desired, maxInput);
        double error = desired - current;
        if (std::fabs(error) > maxInput / 2) {
            if (error > 0) error -= maxInput;
            else error += maxInput;
        }
        return error;
    } else {
        return desired - current;
    }
}

double PidLoop::update(double current, double desired, double dt) {
    if (dt < 1e-6) {
        dt = 1e-6;
    }

    double error = this->error(current, desired);

    mTotalError += error * dt;
    double effort = mP * error + mI * mTotalError;
    if (!mFirst) {
        effort += mD * (error - mLastError) / dt;
    }
    mLastError = error;
    mFirst = false;

    if (effort < mMinOut) effort = mMinOut;
    if (effort > mMaxOut) effort = mMaxOut;

    return effort;
}

void PidLoop::reset() {
    mFirst = true;
    mTotalError = 0.0;
    mLastError = 0.0;
}

PidLoop& PidLoop::withThreshold(double threshold) {
    mThreshold = threshold;
    return *this;
}

PidLoop& PidLoop::withMaxInput(double maxInputBeforeWrap) {
    mMaxInputBeforeWrap = maxInputBeforeWrap;
    return *this;
}

PidLoop& PidLoop::withOutputRange(double minOut, double maxOut) {
    mMinOut = minOut;
    mMaxOut = maxOut;
    return *this;
}

bool PidLoop::isOnTarget(double current, double desired) const {
    return std::fabs(error(current, desired)) < mThreshold;
}
