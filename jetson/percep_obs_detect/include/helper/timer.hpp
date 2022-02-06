#include <chrono>
#include <string>
#include <iostream>

class Timer {
    public:
        Timer(std::string name) : name(name) {
            reset();
        }

        friend std::ostream& operator<<(std::ostream& out, const Timer& timer);

        float getTime() const {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            return duration.count() / 1.0e3;
        }

        void reset() {
            start = std::chrono::high_resolution_clock::now();
        }

    private:
        std::string name;
        std::chrono::time_point<std::chrono::high_resolution_clock> start;

};

std::ostream& operator<<(std::ostream& out, const Timer& timer) {
    float time = timer.getTime();
    out << timer.name << " timer recorded: " << time << " ms." << std::endl;
    return out;
}