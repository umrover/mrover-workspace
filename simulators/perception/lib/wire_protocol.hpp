#pragma once

#include <cstdint>

namespace Protocol {
    struct Request {
    };

    struct Reply {
        uint16_t width;
        uint16_t height;

        int32_t image_type;
        int32_t depth_type;
        int32_t pointcloud_type;

        uint32_t image_size;
        uint32_t depth_size;
        uint32_t pointcloud_size;
        uint8_t data[0];
    };
}
