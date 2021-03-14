#include "perception.hpp"

static Mat HSV;
static Mat DEPTH;
const int MM_PER_M = 1000; // millimeters per meter conversation factor

/* For debug use: print the HSV values at mouseclick locations */
void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event == EVENT_LBUTTONUP) {
        Vec3b p = HSV.at<Vec3b>(y, x);
        float d = DEPTH.at<float>(y, x);
        printf(
            "Get mouse click at (%d, %d), HSV value is H: %d, S: %d, V:%d, "
            "depth is %.2f meters \n",
            y, x, p.val[0], p.val[1], p.val[2], d);
    }
}

TagDetector::TagDetector() {  //initializes detector object with pre-generated dictionary of tags

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
    alvarDict = new cv::aruco::Dictionary(bits, mSize, mCBits);

    // initialize other special parameters that we need to properly detect the URC (Alvar) tags
    alvarParams = new cv::aruco::DetectorParameters();
    alvarParams->markerBorderBits = 2;
    alvarParams->doCornerRefinement = false;
    alvarParams->polygonalApproxAccuracyRate = 0.08;
}

Point2f TagDetector::getAverageTagCoordinateFromCorners(const vector<Point2f> &corners) {  //gets coordinate of center of tag
    // RETURN:
    // Point2f object containing the average location of the 4 corners
    // of the passed-in tag
    Point2f avgCoord;
    for (auto &corner : corners) {
        avgCoord.x += corner.x;
        avgCoord.y += corner.y;
    }
    avgCoord.x /= corners.size();
    avgCoord.y /= corners.size();
    return avgCoord;
}

Point2f TagDetector::getTagCoordRelativeToCenter(const vector<Point2f> &corners, Mat &src, Mat &rgb, Point2f &tagLoc) {
    //RETURN:
    //Point3f containing coordinates of tag from center of image in meters
    Point2f centerCoord;
    double heightImage = src.size().height;
    double widthImage = src.size().width;
    centerCoord.x = widthImage / 2;
    centerCoord.y = heightImage / 2;
    
    Point2f tagCoord;
    tagCoord.x = abs(centerCoord.x - tagLoc.x);
    tagCoord.y = abs(centerCoord.y - tagLoc.y);
    //if tag x coordinate is less than center x, tag x should be negative
    //if tag y coord is greater than center y, tag y should be negative
    if(tagLoc.x < centerCoord.x || tagLoc.y > centerCoord.y) {
        if(tagLoc.x < centerCoord.x && tagLoc.y > centerCoord.y) {
            tagCoord.x = -tagCoord.x;
            tagCoord.y = -tagCoord.y;
        }
        else if(tagLoc.y > centerCoord.y) {
            tagCoord.y = -tagCoord.y;
        }
        else if(tagLoc.x < centerCoord.x) {
            tagCoord.x = -tagCoord.x;
        }
    }

    //draw horizontal and vertical lines from center of tag to edge of frame
    cv::line(rgb, tagLoc, Point(centerCoord.x, tagLoc.y), Scalar(255,255,0), 4, LINE_8);
    //cv::line(rgb, tagLoc, Point(tagLoc.x, heightImage), Scalar(255,255,0), 4, LINE_8);
    cv::line(rgb, tagLoc, Point(tagLoc.x, centerCoord.y), Scalar(255,255,0), 4, LINE_8);

    //label tags with coordinates for debugging
    string tagCoordLabel = "(" + to_string(tagCoord.x) + " " + to_string(tagCoord.y) + ")";
    cv::putText(rgb, tagCoordLabel, Point(tagLoc.x, tagLoc.y+20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

    //draw gridlines through origin
    cv::line(rgb, Point(centerCoord.x, 0), Point(centerCoord.x, heightImage), Scalar(255,255,255), 2, LINE_8);
    cv::line(rgb, Point(0, centerCoord.y), Point(widthImage, centerCoord.y), Scalar(255,255,255), 2, LINE_8);

    //convert to meters using height of tag to scale
    double heightTagM = .20;
    double widthTagM = .20;
    double heightTagPixels = abs(corners[0].y - corners[3].y);
    double widthTagPixels = abs(corners[0].x - corners[1].x);
    tagCoord.x = (tagCoord.x / widthTagPixels) * widthTagM;
    tagCoord.y = (tagCoord.y / heightTagPixels) * heightTagM; 

    //convert to ft for testing
    tagCoord.x = tagCoord.x * 100 / 2.54 / 12;
    tagCoord.y *= 100 / 2.54 / 12;

    //cerr << "tag x and center x " << tagLoc.x << " " << centerCoord.x << " " << tagLoc.x - centerCoord.x << endl;
    //cerr << "tag y and center y " << tagLoc.y << " " << centerCoord.y << " " << tagLoc.y - centerCoord.y << endl;
    cerr << "tag x coord :" << tagCoord.x << endl; 
    cerr << "tag y coord :" << tagCoord.y << endl; 
    return tagCoord;
}

pair<Tag, Tag> TagDetector::findARTags(Mat &src, Mat &depth_src, Mat &rgb) {  //detects AR tags in source Mat and outputs Tag objects for use in LCM
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

    // create Tag objects for the detected tags and return them
    pair<Tag, Tag> discoveredTags;
    if (ids.size() == 0) {
        // no tags found, return invalid objects with tag set to -1
        discoveredTags.first.id = -1;
        discoveredTags.first.loc = Point2f();
        discoveredTags.second.id = -1;
        discoveredTags.second.loc = Point2f();

    } else if (ids.size() == 1) {  // exactly one tag found
        discoveredTags.first.id = ids[0];
        discoveredTags.first.loc = getAverageTagCoordinateFromCorners(corners[0]);
        discoveredTags.first.loc = getTagCoordRelativeToCenter(corners[0], src, rgb, discoveredTags.first.loc);
        
        //cerr << "tag coordinates " << ids[0] << " " << getTagCoordRelativeToCenter(corners[0],src,rgb,discoveredTags.first.loc).x 
        //<< " " << getTagCoordRelativeToCenter(corners[0],src,rgb,discoveredTags.first.loc).y <<endl;

        // set second tag to invalid object with tag as -1
        discoveredTags.second.id = -1;
        discoveredTags.second.loc = Point2f();
    } else if (ids.size() == 2) {  // exactly two tags found
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t0.loc = getTagCoordRelativeToCenter(corners[0], src, rgb, t0.loc);
        t1.id = ids[1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[1]);
        t1.loc = getTagCoordRelativeToCenter(corners[1], src, rgb, t1.loc);

        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    } else {  // detected >=3 tags
        // return leftmost and rightsmost detected tags to account for potentially seeing 2 of each tag on a post
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t0.loc = getTagCoordRelativeToCenter(corners[0], src, rgb, t0.loc);

        t1.id = ids[ids.size() - 1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[ids.size() - 1]);
        t1.loc = getTagCoordRelativeToCenter(corners[ids.size() - 1], src, rgb,  t1.loc);

        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    }

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

    return discoveredTags;
}

double TagDetector::getAngle(float xPixel, float wPixel){
    return atan((xPixel - wPixel/2)/(wPixel/2)* tan(fieldofView/2))* 180.0 /PI;
}


void TagDetector::updateDetectedTagInfo(rover_msgs::TargetPosition *arTags, pair<Tag, Tag> &tagPair, Mat &depth_img, Mat &src, Mat &rgb){
    struct tagPairs {
        vector<int> id;
        vector<double> locx;
        vector<double> locy;
        vector<int> buffer;
    }; 
    tagPairs tags;

    tags.id.push_back(tagPair.first.id);
    tags.locx.push_back(tagPair.first.loc.x);
    tags.locy.push_back(tagPair.first.loc.y);
    tags.id.push_back(tagPair.second.id);
    tags.locx.push_back(tagPair.first.loc.x);
    tags.locy.push_back(tagPair.first.loc.y);
    tags.buffer.push_back(0);
    tags.buffer.push_back(0);

  for (uint i=0; i<2; i++){
    if(tags.id[i] == -1){//no tag found
        if(tags.buffer[i] <= 20){//send buffered tag until tag is found
            ++tags.buffer[i];
        } else {//if still no tag found, set all stats to -1
            arTags[i].z = -1;
            //arTags[i].bearing = -1;
            arTags[i].target_id = -1;
        }
    } 
    else {//one tag found
        if(!isnan(depth_img.at<float>(tags.locy.at(i), tags.locx.at(i)))){
            arTags[i].x = tags.locx.at(i);
            arTags[i].y = tags.locy.at(i);
            arTags[i].z = depth_img.at<float>(tags.locy.at(i), tags.locx.at(i)) / MM_PER_M;
            //arTags[i].bearing = getAngle((int)tags.locx.at(i), src.cols);
            arTags[i].target_id = tags.id.at(i);
            tags.buffer[i] = 0;
        } 
    }
  }
    cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Location Sent: " << arTags[0].x << " " << arTags[0].y << "\n";
    cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ID Sent: " << arTags[0].target_id << "\n";
    
    //print out second tag if found
    if(tags.id[1] != -1) {
        cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Location Sent: " << arTags[1].x << " " << arTags[1].y << "\n";
        cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ID Sent: " << arTags[1].target_id << "\n";
    }

}


