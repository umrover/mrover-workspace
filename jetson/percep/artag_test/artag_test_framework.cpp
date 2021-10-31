#include "artag_test_framework.hpp"

using namespace cv;

ArtagTest::ArtagTest(string & image_, string & filename_, string & testcase_name_) {
    image = imread(image_, IMREAD_UNCHANGED);
    filename = filename_;
    testcase_name = testcase_name_;
}

float ArtagTest::intersectionOverUnion(const vector<Point2f> &predicted_corners, const vector<Point2f> &label_corners) {

    Mat predicted(image.size(), CV_8UC1, Scalar(0));
    Mat label(image.size(), CV_8UC1, Scalar(0));

    const Point predicted_array[4] = {predicted_corners[0], predicted_corners[1], predicted_corners[2], predicted_corners[3]};
    const Point label_array[4] = {label_corners[0], label_corners[1], label_corners[2], label_corners[3]};

    fillConvexPoly(predicted, predicted_array, 4, Scalar(255));
    fillConvexPoly(label, label_array, 4, Scalar(255));

    Mat intersection_matrix;
    Mat union_matrix;

    bitwise_and(predicted, label, intersection_matrix);
    bitwise_or(predicted, label, union_matrix);

    float intersection_polygons = countNonZero(intersection_matrix);
    float union_polygons = countNonZero(union_matrix);

    return intersection_polygons/union_polygons;
}

/*std::vector<pair<pair<std::vector<cv::Point2f>, int>, pair<std::vector<cv::Point2f>, int> > > ArtagTest::matchPredictionsAndLabels(
    pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> > &corners_and_ids,
    pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> > &ground_truth
) {
    return {};
}*/

std::vector< pair<std::vector<cv::Point2f>, int> > ArtagTest::getLabels() {
    int label_id = 0;
    float corner1_x, corner1_y, corner2_x, corner2_y, corner3_x, corner3_y, corner4_x, corner4_y = 0;
    std::vector< pair<std::vector<cv::Point2f>, int> > labels;
    vector<cv::Point2f> corners;
    std::ifstream label_file(filename);
    while(label_file >> label_id >> corner1_x >> corner1_y >> corner2_x >> corner2_y >> corner3_x >> corner3_y >> corner4_x >> corner4_y) {
        corners.push_back(Point2f(corner1_x, corner1_y));
        corners.push_back(Point2f(corner2_x, corner2_y));
        corners.push_back(Point2f(corner3_x, corner3_y));
        corners.push_back(Point2f(corner4_x, corner4_y));
        labels.push_back({corners, label_id});
    }
    return labels;
}

void ArtagTest::run(TagDetector * detector) {
    Mat rgb;
    Mat depth_src;
    std::cout << testcase_name << "\n";
    detector->findARTags(image, depth_src, rgb);
    pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> >cornersAndIds = detector->getCornersAndIds();
    std::vector< pair<std::vector<cv::Point2f>, int> > labels = getLabels();
    std::cout << "IoU - " << intersectionOverUnion(cornersAndIds.first[0], labels[0].first) << "\n";
}

ArtagTestSuite::ArtagTestSuite() 
    : detector(new TagDetector()) {
        std::cout << "created test suite\n";
    }

void ArtagTestSuite::run() {
    string image = "jetson/percep/artag_test/images/image1.jpg";
    string label = "jetson/percep/artag_test/labels/image1.tag";
    string testcase_name = "Test 1:";
    ArtagTest t1(image, label, testcase_name);
    t1.run(detector);
}


