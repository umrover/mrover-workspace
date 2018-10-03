#include <iostream>
#include <opencv2/opencv.hpp>
#include <percepsim/percepsim.hpp>

int main() {
    Perception::SimulatedCamera cam;

    int key = 0;
    while (key != 113) {  // 'q'
        if (!cam.grab()) {
            break;
        }

        imshow("Image", cam.retrieve_image());
        imshow("Depth", cam.retrieve_depth());

        key = cv::waitKey(5);
    }

    return 0;
}
