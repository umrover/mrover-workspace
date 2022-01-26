#include "zed.hpp"

namespace Source {
Zed::Zed(const rapidjson::Document &config) {
    // Read Config file variables
    res = sl::Resolution(
                config["camera"]["resolution_width"].GetInt(),
                config["camera"]["resolution_height"].GetInt());
    init_params.coordinate_units = sl::UNIT::MILLIMETER; // TODO: Read from config
    init_params.camera_resolution = sl::RESOLUTION::VGA; // TODO: This only works for 180x320
    init_params.camera_fps = config["camera"]["zed_params"]["camera_fps"].GetInt();

    // Allocate sl::Mats
    image_zed.alloc(res.width, res.height, sl::MAT_TYPE::U8_C4);
    depth_zed.alloc(res.width, res.height, sl::MAT_TYPE::F32_C1);
    frame.alloc(res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

    // Map sl::Mats to cv::Mat, so update to sl::Mat updates the cv::Mat
    image = cv::Mat(res.height, res.width, CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
	depth = cv::Mat(res.height, res.width, CV_32FC1, depth_zed.getPtr<sl::uchar1>(sl::MEM::CPU));

    auto error = zed.open(init_params);
    if (error != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error("Error opening ZED camera");
    }
}

bool Zed::grab_frame() {
    return zed.grab() == sl::ERROR_CODE::SUCCESS;
}

GPU_Cloud Zed::get_cloud() {
    GPU_Cloud pc;
    zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, res);
    getRawCloud(pc, frame);
    return pc;
}

cv::Mat& Zed::get_image() {
    zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, res); // Update sl::Mat
	return image;
}

cv::Mat& Zed::get_depth() {
    zed.retrieveMeasure(depth_zed, sl::MEASURE::DEPTH,  sl::MEM::CPU, res);
	return depth;
}

void Zed::write_data() {

}

}
