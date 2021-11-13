#include "artag_test_framework.hpp"

using namespace cv;

ArtagTest::ArtagTest(string & image_, string & filename_) {
    image = imread(image_, IMREAD_UNCHANGED);
    filename = filename_;
    testcase_name = image_;
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


pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> > ArtagTest::getLabels() {
    
    int label_id = 0;
    float corner1_x, corner1_y, corner2_x, corner2_y, corner3_x, corner3_y, corner4_x, corner4_y = 0;

    vector<vector<cv::Point2f> > allCorners;
    vector<int> ids;

    std::ifstream label_file(filename);

    if(!label_file.is_open()) throw FileError(filename);

    while(label_file >> label_id >> corner1_x >> corner1_y >> corner2_x >> corner2_y >> corner3_x >> corner3_y >> corner4_x >> corner4_y) {
        vector<cv::Point2f> corners;
        corners.push_back(Point2f(corner1_x, corner1_y));
        corners.push_back(Point2f(corner2_x, corner2_y));
        corners.push_back(Point2f(corner3_x, corner3_y));
        corners.push_back(Point2f(corner4_x, corner4_y));
        allCorners.push_back(corners);
        ids.push_back(label_id);
    }

    return {allCorners, ids};
}

void ArtagTest::run(TagDetector * detector) {
    try {
        std::cout << testcase_name << "\n";

        Mat rgb;
        Mat depth_src;

        detector->findARTags(image, depth_src, rgb);

        pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> > cornersAndIds = detector->getCornersAndIds();
        pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> > labels = getLabels();

        for(size_t i = 0; i < std::max(cornersAndIds.first.size(), labels.first.size()); ++i) {
            
            if (i == cornersAndIds.first.size() || i == labels.first.size()) throw CountingError(TestError::ErrorString::artag, cornersAndIds.first.size(), labels.first.size());
            
            std::cout << " Tag " << i << ":\n";

            float IoU = intersectionOverUnion(cornersAndIds.first[i], labels.first[i]);
            if(IoU < IoUThreshold) throw ThresholdError(TestError::ErrorString::intersection, IoU, IoUThreshold);
            else std::cout << "  IoU - " << intersectionOverUnion(cornersAndIds.first[i], labels.first[i]) << "\n";

            if(cornersAndIds.second[i] != labels.second[i]) throw CountingError(TestError::ErrorString::id, cornersAndIds.second[i], labels.second[i]);
            else std::cout << "  Correct Id: " << labels.second[i] << " Detected Id: " << cornersAndIds.second[i] << "\n";

        }

        std::cout << "\n";

    } catch(TestError & err) {
        err.print();
        std::cout << "\n";
    }
}

ArtagTestSuite::ArtagTestSuite() 
    : detector(new TagDetector()) {
        std::cout << "Test Suite Results\n\n";
    }

void ArtagTestSuite::run() {

    const string image_path = "jetson/percep/artag_test/images/";
    const string label_path = "jetson/percep/artag_test/labels/";
    const string label_postfix = ".tag";

    for(const auto & image_file : std::experimental::filesystem::directory_iterator(image_path)) {
        string image_file_string = image_file.path().string();
        string label_file = label_path + image_file_string.substr(image_path.length(), ((image_file_string.length() - label_postfix.length()) - image_path.length())) + label_postfix;
        ArtagTest test(image_file_string, label_file);
        test.run(detector);
    }
}

ArtagTestSuite::~ArtagTestSuite() {
    delete detector;
}


