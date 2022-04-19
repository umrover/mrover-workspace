#include "perception.hpp"

static Mat HSV;
static Mat DEPTH;

/* For debug use: print the HSV values at mouseclick locations */
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONUP) {
        Vec3b p = HSV.at<Vec3b>(y, x);
        float d = DEPTH.at<float>(y, x);
        printf("Get mouse click at (%d, %d), HSV value is H: %d, S: %d, V:%d, "
               "depth is %.2f meters \n",
               y, x, p.val[0], p.val[1], p.val[2], d);
    }
}

//initializes detector object with pre-generated dictionary of tags 
TagDetector::TagDetector(const rapidjson::Document& mRoverConfig) :

//Populate Constants from Config File
        BUFFER_ITERATIONS{mRoverConfig["ar_tag"]["buffer_iterations"].GetInt()},
        MARKER_BORDER_BITS{mRoverConfig["alvar_params"]["marker_border_bits"].GetInt()},
        DO_CORNER_REFINEMENT{mRoverConfig["alvar_params"]["do_corner_refinement"].GetBool()},
        POLYGONAL_APPROX_ACCURACY_RATE{mRoverConfig["alvar_params"]["polygonal_approx_accuracy_rate"].GetDouble()},
        MM_PER_M{mRoverConfig["mm_per_m"].GetInt()},
        DEFAULT_TAG_VAL{mRoverConfig["ar_tag"]["default_tag_val"].GetInt()} {

    cv::FileStorage fsr("jetson/percep/alvar_dict.yml", cv::FileStorage::READ);
    if (!fsr.isOpened()) {  //throw error if dictionary file does not exist
        std::cerr << "ERR: \"alvar_dict.yml\" does not exist! Create it before running main\n";
        throw Exception();
    }

    // read dictionary from file
    int mSize, mCBits;
    cv::Mat bits;
    fsr["MarkerSize"] >> mSize;
    fsr["MaxCorrectionBits"] >> mCBits;
    fsr["ByteList"] >> bits;
    fsr.release();
    alvarDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // initialize other special parameters that we need to properly detect the URC (Alvar) tags
    alvarParams = new cv::aruco::DetectorParameters();
    alvarParams->markerBorderBits = MARKER_BORDER_BITS;
    //alvarParams->doCornerRefinement = DO_CORNER_REFINEMENT;
    alvarParams->polygonalApproxAccuracyRate = POLYGONAL_APPROX_ACCURACY_RATE;
}

Point2f TagDetector::getAverageTagCoordinateFromCorners(const vector<Point2f>& corners) {  //gets coordinate of center of tag
    // RETURN:
    // Point2f object containing the average location of the 4 corners
    // of the passed-in tag
    Point2f avgCoord;
    for (auto& corner: corners) {
        avgCoord.x += corner.x;
        avgCoord.y += corner.y;
    }
    avgCoord.x /= corners.size();
    avgCoord.y /= corners.size();
    return avgCoord;
}

pair<Tag, Tag> TagDetector::findARTags(Mat& src, Mat& depth_src, Mat& rgb) {  //detects AR tags in source Mat and outputs Tag objects for use in LCM
    // RETURN:
    // pair of target objects- each object has an x and y for the center,
    // and the tag ID number return them such that the "leftmost" (x
    // coordinate) tag is at index 0
    cvtColor(src, rgb, COLOR_RGBA2RGB);
    // clear ids and corners vectors for each detection
    ids.clear();
    corners.clear();

    // Find tags
    cv::aruco::detectMarkers(rgb, alvarDict, corners, ids, alvarParams);
#if AR_RECORD
    cv::aruco::drawDetectedMarkers(rgb, corners, ids);
#endif

#if PERCEPTION_DEBUG
    // Draw detected tags
    cv::aruco::drawDetectedMarkers(rgb, corners, ids);
    cv::imshow("AR Tags", rgb);

    // on click debugging for color
    DEPTH = depth_src;
    cvtColor(rgb, HSV, COLOR_RGB2HSV);
    setMouseCallback("Obstacle", onMouse);
#endif

    // create Tag objects for the detected tags and return them
    pair<Tag, Tag> discoveredTags;
    if (ids.empty()) {
        // no tags found, return invalid objects with tag set to -1
        discoveredTags.first.id = DEFAULT_TAG_VAL;
        discoveredTags.first.loc = Point2f();
        discoveredTags.second.id = DEFAULT_TAG_VAL;
        discoveredTags.second.loc = Point2f();

    } else if (ids.size() == 1) {  // exactly one tag found
        discoveredTags.first.id = ids[0];
        discoveredTags.first.loc = getAverageTagCoordinateFromCorners(corners[0]);
        // set second tag to invalid object with tag as -1
        discoveredTags.second.id = DEFAULT_TAG_VAL;
        discoveredTags.second.loc = Point2f();
    } else if (ids.size() == 2) {  // exactly two tags found
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t1.id = ids[1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    } else {  // detected >=3 tags
        // return leftmost and rightmost detected tags to account for potentially seeing 2 of each tag on a post
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t1.id = ids[ids.size() - 1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[ids.size() - 1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    }
    return discoveredTags;
}

void TagDetector::updateDetectedTagInfo(rover_msgs::Target* arTags, pair<Tag, Tag>& tagPair, Mat& depth_img, Mat& xyz_img, Mat& src) const {
    struct tagPairs {
        vector<int> ids;
        vector<int> x_pixels;
        vector<int> y_pixels;
        vector<int> buf_counts;
    };
    tagPairs tags;

    tags.ids.push_back(tagPair.first.id);
    tags.x_pixels.push_back(static_cast<int>(tagPair.first.loc.x));
    tags.y_pixels.push_back(static_cast<int>(tagPair.first.loc.y));
    tags.ids.push_back(tagPair.second.id);
    tags.x_pixels.push_back(static_cast<int>(tagPair.second.loc.x));
    tags.y_pixels.push_back(static_cast<int>(tagPair.second.loc.y));
    tags.buf_counts.push_back(0);
    tags.buf_counts.push_back(0);

    for (size_t i = 0; i < 2; i++) {
        if (tags.ids[i] == DEFAULT_TAG_VAL) { // no tag found
            if (tags.buf_counts[i] <= BUFFER_ITERATIONS) { // send buffered tag until tag is found
                ++tags.buf_counts[i];
            } else { // if still no tag found, set all stats to -1
                arTags[i].distance = DEFAULT_TAG_VAL;
                arTags[i].bearing = DEFAULT_TAG_VAL;
                arTags[i].id = DEFAULT_TAG_VAL;
            }
        } else { // tag found
            int y_pixel = tags.y_pixels.at(i);
            int x_pixel = tags.x_pixels.at(i);
            float x, y, z;
            {
                auto xyz = xyz_img.at<Vec4f>(y_pixel, x_pixel);
                x = xyz[0];
                y = xyz[1];
                z = xyz[2];
            }
            auto raw_depth = depth_img.at<float>(y_pixel, x_pixel);
            if (!isnan(raw_depth)) {
                arTags[i].distance = sqrt(x * x + y * y + z * z) * static_cast<float>(MM_PER_M);
            }
            // +z is forward, +x is right, all relative to camera
            arTags[i].bearing = atan2(x, z);
            arTags[i].id = tags.ids.at(i);
            tags.buf_counts[i] = 0;
        }
    }
}
