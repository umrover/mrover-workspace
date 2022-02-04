#pragma once

#include <chrono>
#include <string>
#include <iostream>
#include <utility>

class Timer {
    public:
        explicit Timer(std::string name) : name(std::move(name)) {
            reset();
        }

        friend std::ostream& operator<<(std::ostream& out, const Timer& timer) {
            long long time = timer.getTime();
            out << timer.name << " timer recorded: " << time << " ms.\n";
            return out;
        }

        long long getTime() const {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            return duration;
        }

        long long reset() {
            long long time = getTime();
            start = std::chrono::high_resolution_clock::now();
            return time;
        }

    private:
        std::string name;
        std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

class RollingAverage {
    private:
        long long rolling;
        size_t count;
    public:
        void add(long long value) {
            rolling += value;
            count++;
        }

        void reset() {
            rolling = 0;
            count = 0;
        }

        long long getAverage() const {
            if (count == 0) {
                return 0;
            }
            return static_cast<long long>(rolling / count);
        }
};