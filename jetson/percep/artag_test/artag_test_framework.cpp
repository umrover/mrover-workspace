#include "artag_test_framework.hpp"

using namespace cv;

ArtagTest::ArtagTest(Mat image_) : image(image_) {}

float ArtagTest::intersectionOverUnion(const vector<Point2f> &predicted_corners, const vector<Point2f> &label_corners) {

    Mat predicted(image.size(), CV_8UC1, 0);
    Mat label(image.size(), CV_8UC1, 0);

    fillConvexPoly(predicted, predicted_corners, 255);
    fillConvexPoly(label, label_corners, 255);

    Mat intersection_matrix;
    Mat union_matrix;

    bitwise_and(predicted, label, intersection_matrix);
    bitwise_or(predicted, label, union_matrix);

    float intersection_polygons = countNonZero(intersection_matrix);
    float union_polygons = countNonZero(union_matrix);

    return intersection_polygons/union_polygons;
}

void ArtagTest::run(TagDetector * detector) {
    Mat rgb;
    Mat depth_src;
    detector->findARTags(image, depth_src, rgb);
    pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> >cornersAndIds = detector->getCornersAndIds();
}

ArtagTestSuite::ArtagTestSuite() 
    : detector(new TagDetector()) {
        std::cout << "created test suite\n";
    }


