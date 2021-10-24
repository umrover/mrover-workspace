#include "artag_test_framework.hpp"

using namespace cv;

ArtagTest::ArtagTest(Mat image_) : image(image_) {}

float ArtagTest::intersectionOverUnion(const vector<Point2f> &predicted_corners, const vector<Point2f> &label_corners) {

    Mat predicted(image.size(), CV_8UC1, 0);
    Mat label(image.size(), CV_8UC1, 0);

    fillConvexPoly(predicted, predicted_corners, 255);
    fillConvexPoly(label, label_corners, 255);

    float intersection_polygons = countNonZero(bitwise_and(predicted, label));
    float union_polygons = countNonZero(bitwise_or(predicted, label));

    return intersection_polygons/union_polygons;
}

void ArtagTest::run(TagDetector * detector) {
    Mat rgb;
    Mat depth_src;
    detector->findARTags(image, depth_src, rgb);
    pair<std::vector<cv::Point2f>, std::vector<int> > cornersAndIds = detector->getCornersAndIds();
}

ArtagTestSuite::ArtagTestSuite() 
    : detector(new TagDetector()) {}


