#ifndef MAP_H
#define MAP_H
#include <vector>
#include <string>
#include <utility>
#include "rover_msgs/Joystick.hpp"

class Map  {
    public:
        // populates inputMap and navigationMap
        Map(const vector< vector <vector<uint8_t> > > &inputMap, uint32_t row_start, uint32_t col_start); 

        vector< vector <vector<uint8_t> > >& updateNavMap(double time, Joystick joystick);

        double updateHeading(const double &oldHeading, const double &joystickHeading);

        void displayLCM();

        double getHeading() {
            return currentHeading; 
        }

    private:
        struct Square {
            int32_t x; 
            int32_t y; 

            Square(int32_t x, int32_t y) : x(x), y(y) {}
        };
        struct Color {
            uint8_t r = 128;
            uint8_t g = 128;
            uint8_t b = 128;
        };
        //RGB vector of squares
        std::vector< std::vector <std::vector<uint8_t> > > inputMap;
        std::vector< std::vector <std::vector<uint8_t> > > navigationMap;
        Square startIndices;
        Square currentIndices;
        double xTotalDisplacement;
        double yTotalDisplacement;
        double currentHeading;
        const static double cellLength = 0.4;
        const static double widthTriangle = 10;
        const static double speed = 1.6; 

        Joystick joystick;

        const static int dimension = 10000; 
        int ContourX[dimension][2];

        void SetPixel(int x, int y, Color color);

        void ScanLine(int x1, int x2, int y1, int y2); 

        void DrawTriangle(Square p0, Square p1, Square p2);

        std::pair<Square, Square> getTriangleVertex(Square& start);
};

#endif