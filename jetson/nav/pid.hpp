#pragma once

class PidLoop {
    public:
        PidLoop(double Kp, double Ki, double Kd);

        double update(double current, double desired);
        void reset();

    private:
        double error(double current, double desired);

        double Kp_;
        double Ki_;
        double Kd_;

        const double sat_min_out_ = -1.0;
        const double sat_max_out_ = +1.0;

        bool first_;
        double accumulated_error_;
        double last_error_;
};
