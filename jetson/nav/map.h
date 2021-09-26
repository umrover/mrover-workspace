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

        double updateHeading(const double &oldHeading, const double &joystickHeading);

        void displayLCM();

        void colorMap();

        void saveMapToImage();

        double getHeading() {
            return currentHeading; 
        }

    private:
        struct Square {
            int x, y; 
            unsigned char color; 
        };

        std::string inputPath;
        std::vector< std::vector<Square> > inputMap;
        std::vector< std::vector<Square> > navigationMap;
        std::pair <int,int> startIndices;
        std::pair <int,int> currentIndices;
        double xTotalDisplacement;
        double yTotalDisplacement;
        double currentHeading;
        const static double cellLength = 0.4;
        const static double widthTriangle = 10;

        const static int dimension = 1000; 
        int ContourX[dimension][2];

        void SetPixel(int x, int y, char color) {
            if ((x < 0) || (x >= dimension) || (y < 0) || (y >= dimension)) { return; }

            if (navigationMap[y][x].color == ' ') {
                navigationMap[y][x].color = color;
                navigationMap[y][x].x = x; 
                navigationMap[y][x].x = y;
            }
        }

        void ScanLine(int x1, int x2, int y1, int y2); 

        void DrawTriangle(Square p0, Square p1, Square p2);

        Square getTriangleVertex(Square& start);
};

#endif