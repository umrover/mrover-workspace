#include "perception.hpp"

static cv::Mat HSV;
static cv::Mat DEPTH;

/* For debug use: print the HSV values at mouseclick locations */
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONUP) {
        auto p = HSV.at<cv::Vec3b>(y, x);
        float d = DEPTH.at<float>(y, x);
        printf("Get mouse click at (%d, %d), HSV value is H: %d, S: %d, V:%d, "
               "depth is %.2f meters \n",
               y, x, p.val[0], p.val[1], p.val[2], d);
    }
}

// initializes detector object with pre-generated dictionary of tags
TagDetector::TagDetector(const rapidjson::Document& mRoverConfig) :
        BUFFER_ITERATIONS{mRoverConfig["ar_tag"]["buffer_iterations"].GetInt()},
        MARKER_BORDER_BITS{mRoverConfig["alvar_params"]["marker_border_bits"].GetInt()},
        DO_CORNER_REFINEMENT{mRoverConfig["alvar_params"]["do_corner_refinement"].GetBool()},
        POLYGONAL_APPROX_ACCURACY_RATE{mRoverConfig["alvar_params"]["polygonal_approx_accuracy_rate"].GetDouble()},
        MM_PER_M{mRoverConfig["mm_per_m"].GetInt()},
        DEFAULT_TAG_VAL{mRoverConfig["ar_tag"]["default_tag_val"].GetInt()} {

    cv::FileStorage fsr("jetson/percep/alvar_dict.yml", cv::FileStorage::READ);
    if (!fsr.isOpened()) {  //throw error if dictionary file does not exist
        std::cerr << "ERR: \"alvar_dict.yml\" does not exist! Create it before running main" << std::endl;
        throw std::runtime_error("Dictionary does not exist");
    }

    // read dictionary from file
    int size, correctionBits;
    cv::Mat bits;
    fsr["MarkerSize"] >> size;
    fsr["MaxCorrectionBits"] >> correctionBits;
    fsr["ByteList"] >> bits;
    fsr.release();
    mAlvarDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // initialize other special parameters that we need to properly detect the URC (Alvar) tags
    mAlvarParams = new cv::aruco::DetectorParameters();
    mAlvarParams->markerBorderBits = MARKER_BORDER_BITS;
    //alvarParams->doCornerRefinement = DO_CORNER_REFINEMENT;
    mAlvarParams->polygonalApproxAccuracyRate = POLYGONAL_APPROX_ACCURACY_RATE;
}

/***
 * Given the four corners of an AR tag, calculate the center.
 *
 * @param corners Corners in camera pixel space of AR tag.
 * @return        AR tag center in camera pixel space.
 */
cv::Point2f TagDetector::getAverageTagCoordinateFromCorners(const std::vector<cv::Point2f>& corners) {  //gets coordinate of center of tag
    cv::Point2f avgCoord;
    for (auto& corner: corners) {
        avgCoord.x += corner.x;
        avgCoord.y += corner.y;
    }
    avgCoord.x /= static_cast<float>(corners.size());
    avgCoord.y /= static_cast<float>(corners.size());
    return avgCoord;
}

/***
 * Try and detect AR tag markers from a camera frame.
 *
 * @param src       Raw camera RGBA data.
 * @param depth_src Camera depth information, this is the z component of the vector representing the pixel in 3D space.
 * @param rgb       Camera RGB data.
 * @return          Pair of target objects, each object has an ID and pixel x and y position for the center of the tag.
 *                  The leftmost tag is always the first item in the pair,
 */
std::pair<Tag, Tag> TagDetector::findARTags(cv::Mat& src, cv::Mat& depth_src, cv::Mat& rgb) {
    cv::cvtColor(src, rgb, cv::COLOR_RGBA2RGB);
    // clear ids and corners vectors for each detection
    mIds.clear();
    mCorners.clear();

    // find tags
    cv::aruco::detectMarkers(rgb, mAlvarDict, mCorners, mIds, mAlvarParams);
#if AR_RECORD
    cv::aruco::drawDetectedMarkers(rgb, corners, ids);
#endif

#if PERCEPTION_DEBUG
    // Draw detected tags
    cv::aruco::drawDetectedMarkers(rgb, mCorners, mIds);
    cv::imshow("AR Tags", rgb);

    // on click debugging for color
    DEPTH = depth_src;
    cv::cvtColor(rgb, HSV, cv::COLOR_RGB2HSV);
    cv::setMouseCallback("Obstacle", onMouse);
#endif

    // create Tag objects for the detected tags and return them
    std::pair<Tag, Tag> discoveredTags;
    if (mIds.empty()) {
        // no tags found, return invalid objects with tag set to -1
        discoveredTags.first.id = DEFAULT_TAG_VAL;
        discoveredTags.second.id = DEFAULT_TAG_VAL;
    } else if (mIds.size() == 1) {
        // exactly one tag found
        discoveredTags.first.id = mIds[0];
        discoveredTags.first.loc = getAverageTagCoordinateFromCorners(mCorners[0]);
        // set second tag to invalid object with tag as -1
        discoveredTags.second.id = DEFAULT_TAG_VAL;
    } else if (mIds.size() == 2) {
        // exactly two tags found
        Tag t0, t1;
        t0.id = mIds[0];
        t0.loc = getAverageTagCoordinateFromCorners(mCorners[0]);
        t1.id = mIds[1];
        t1.loc = getAverageTagCoordinateFromCorners(mCorners[1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    } else {
        // detected >=3 tags
        // return leftmost and rightmost detected tags to account for potentially seeing 2 of each tag on a post
        Tag t0, t1;
        t0.id = mIds[0];
        t0.loc = getAverageTagCoordinateFromCorners(mCorners[0]);
        t1.id = mIds[mIds.size() - 1];
        t1.loc = getAverageTagCoordinateFromCorners(mCorners[mIds.size() - 1]);
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

/***
 * Calculate values of final messages to broadcast: AR tag distances and bearings relative to the rover.
 * We know the pixel positions of the AR tags, use those to retrieve the XYZ values from the point cloud.
 * Note: Output is not filtered whatsoever, do not expect consistent readings.
 *
 * @param outTags       LCM network message to fill, used mainly by navigation
 * @param tagPair       AR tag pixel positions and IDs
 * @param depth_img     TODO needed?
 * @param xyz_img       Point cloud XYZ data calculated from rectifying both images.
 */
void TagDetector::updateDetectedTagInfo(
        rover_msgs::Target* outTags, std::pair<Tag, Tag> const& tagPair,
        cv::Mat const& depth_img, cv::Mat const& xyz_img
) const {
    std::array<Tag, 2> tags{tagPair.first, tagPair.second};
    for (size_t i = 0; i < 2; ++i) {
        Tag const& tag = tags[i];
        rover_msgs::Target& outArTag = outTags[i];
        if (tag.id == DEFAULT_TAG_VAL) {
            // no tag found
            outArTag.distance = DEFAULT_TAG_VAL;
            outArTag.bearing = DEFAULT_TAG_VAL;
            outArTag.id = DEFAULT_TAG_VAL;
        } else {
            // tag found
            int xPixel = static_cast<int>(std::lround(tag.loc.x));
            int yPixel = static_cast<int>(std::lround(tag.loc.y));
            // +z is forward, +x is right, all in millimeters and relative to camera
            float x, y, z;
            {
                auto xyz = xyz_img.at<cv::Vec4f>(yPixel, xPixel);
                x = xyz[0];
                y = xyz[1];
                z = xyz[2];
            }
            // ensure we have valid values to work with
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                // put no tag found since we cannot find out any information
                outArTag.distance = DEFAULT_TAG_VAL;
                outArTag.bearing = DEFAULT_TAG_VAL;
                outArTag.id = DEFAULT_TAG_VAL;
            } else {
                // use Euclidean method to calculate distance, convert to meters
                const float MM_TO_M = 0.001f;
                outArTag.distance = std::sqrt(x * x + y * y + z * z) * MM_TO_M;
                outArTag.bearing = std::atan2(x, z) * 180.0 / PI;
                outArTag.id = tag.id;
            }
        }
    }
}
