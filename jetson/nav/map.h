#ifndef MAP_H
#define MAP_H
#include <vector>
#include <string>
#include <utility>

class Map  {
    public:
        Map(std::string inputPath); // populates inputMap and navigationMap

        std::vector<uint8_t> ReadInputImage(std::string inputPath);

        void updatePosition(double time, double joystickMagnitude, double joystickHeading);

        void displayLCM();

        std::vector<uint8_t> extractObstacles();

        void colorMap();

        void saveMapToImage();

    private:
        std::string inputPath;
        std::vector<uint8_t> inputMap;
        std::vector<uint8_t> navigationMap;
        std::pair <int,int> startIndices;
        std::pair <int,int> currentIndices;
        double xTotalDisplacement;
        double yTotalDisplacement;
        double currentHeading;
        const static double cellLength = 0.4;

};

#endif