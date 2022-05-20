# Provided for convenience (syntax highlighting for CLion)
# Use Meson in production as it is the officially suported build system
cmake_minimum_required(VERSION 3.8)
project(perception)
set(CMAKE_CXX_STANDARD 17)

find_package(lcm REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(ZED 2 REQUIRED)
find_package(CUDA 11 REQUIRED)
find_package(PCL REQUIRED)
include(${LCM_USE_FILE})
include_directories(/usr/local/include/lcm)
include_directories(/usr/include/rapidjson)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

file(GLOB_RECURSE LCM_FILES CONFIGURE_DEPENDS "../../rover_msgs/*.lcm")
lcm_wrap_types(
        C_EXPORT rover_lcm_types
        C_SOURCES c_sources
        C_HEADERS c_headers
        CPP_HEADERS cpp_headers
        ${LCM_FILES}
)

lcm_add_library(lcm_types CPP ${cpp_headers})
target_include_directories(lcm_types INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE REMOVE_CMAKE "cmake-build*/*" "deprecated/*" "percep_test/*")
list(REMOVE_ITEM SOURCE_FILES ${REMOVE_CMAKE})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} lcm_types lcm Eigen3::Eigen ${OpenCV_LIBS} ${ZED_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})