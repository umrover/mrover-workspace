#pragma once

#include <optional>

class PidLoop {
public:
    PidLoop(double p, double i, double d);

    double update(double current, double desired, double dt);

    void reset();

    PidLoop& withMaxInput(double maxInputBeforeWrap);

    PidLoop& withThreshold(double threshold);

    PidLoop& withOutputRange(double minOut, double maxOut);

    [[nodiscard]] double error(double current, double desired) const;

    [[nodiscard]] bool isOnTarget(double current, double desired) const;

private:
    double mP, mI, mD;
    // Some inputs, such as angles, are cyclic in nature.
    // Set this to account for that properly so computations are correct on boundaries.
    std::optional<double> mMaxInputBeforeWrap;

    bool mFirst = true;
    double mTotalError = 0.0;
    double mLastError = 0.0;
    double mThreshold = 0.0;
    double mMinOut = -1.0;
    double mMaxOut = +1.0;
};
