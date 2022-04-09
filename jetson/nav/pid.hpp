#pragma once

class PidLoop {
public:
    PidLoop(double p, double i, double d);

    double update(double current, double desired, double dt);

    void reset();

private:
    double mP;
    double mI;
    double mD;

    const double mMinOut = -1.0;
    const double mMaxOut = +1.0;

    bool mFirst;
    double mTotalError;
    double mLastError;
};
