#include "percepsim.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <cassert>
#include <thread>
#include <mutex>
#include <condition_variable>

int main() {
    std::mutex m;

    Perception::Simulator sim;
    Perception::SimulatedCamera cam;

    std::condition_variable run_test_cv;
    bool run_test = false;

    std::condition_variable test_done;
    bool receiver_finished = false;

    const int width = 4;
    const int height = 3;

    std::thread sender([&](){
        {
            std::unique_lock<std::mutex> lock_(m);
            run_test_cv.wait(lock_, [&]() { return run_test; });
        }

        cv::Mat img = cv::Mat::ones(height, width, CV_32F);
        cv::Mat depth = cv::Mat::zeros(height, width, CV_8UC1);

        sim.publish(img, depth);
        std::cerr << "published image" << std::endl;

        {
            std::unique_lock<std::mutex> lock_(m);
            test_done.wait(lock_, [&]() { return receiver_finished; });
        }

        std::cerr << "deallocating simulator" << std::endl;
    });

    std::thread receiver([&](){
        {
            std::unique_lock<std::mutex> lock_(m);
            run_test_cv.wait(lock_, [&]() { return run_test; });
        }

        assert(cam.grab());
        cv::Mat img = cam.retrieve_image();
        cv::Mat depth = cam.retrieve_depth();

        assert(img.cols == width && img.rows == height && img.type() == CV_32F);
        assert(depth.cols == width && depth.rows == height && depth.type() == CV_8UC1);

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                assert(img.at<float>(y, x) == 1.0f);
                assert(depth.at<uchar>(y, x) == 0);
            }
        }

        {
            std::unique_lock<std::mutex> lock_(m);
            receiver_finished = true;
            test_done.notify_one();
        }
    });

    // Run the test
    {
        std::unique_lock<std::mutex> lock_(m);
        run_test = true;
        run_test_cv.notify_all();
    }

    receiver.join();
    sender.join();
    return 0;
}
