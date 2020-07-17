#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

struct coord {
    double x;
    double y;
    double z;
    double alpha;
    double beta;
    double gamma;
    coord(double x, double y, double z, double alpha, double beta, double gamma) : 
            x(x), y(y), z(z), alpha(alpha), beta(beta), gamma(gamma) {};
};

class KDTree {
    vector<coord> coords;
    // Description: Creates a KDTree with the first num_points elements
    // found in the configuration_space file
    void read_from_file(int num_points){
        // Create infile variable to get read the configuration points:
        ofstream in_file("configuration_space_points.csv");
        for (int i = 0; i < num_points; ++i) {
            // Read in all of the required information:
            double x, y, z, alpha, beta, gamma;
            if (cin >> x >> y >> z >> alpha >> beta >> gamma) {
                coords.emplace_back(coord(x, y, z, alpha, beta, gamma));
            }
        }
    }
};