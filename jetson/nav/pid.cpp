#include "pid.hpp"

#include <cmath>

PidLoop::PidLoop(double p, double i, double d) :
        mP(p),
        mI(i),
        mD(d),
        mFirst(true),
        mTotalError(0.0),
        mLastError(0.0) {
}

PidLoop::PidLoop(double p, double i, double d, double maxInputBeforeWrap) : PidLoop(p, i, d) {
    mMaxInputBeforeWrap = {maxInputBeforeWrap};
}

double PidLoop::update(double current, double desired, double dt) {
    if (dt < 1e-6) {
        dt = 1e-6;
    }

    double error = desired - current;
    if (mMaxInputBeforeWrap) {
        double maxInput = mMaxInputBeforeWrap.value();
        if (std::fabs(error) > maxInput / 2) {
            if (error > 0) error -= maxInput;
            else error += maxInput;
        }
    }

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
