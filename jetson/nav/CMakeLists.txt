cmake_minimum_required(VERSION 3.8)
project(navigation)
set(CMAKE_CXX_STANDARD 14)

find_package(lcm REQUIRED)
include(${LCM_USE_FILE})
include_directories(/usr/local/include/lcm)
include_directories(/usr/include/rapidjson)

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
file(GLOB_RECURSE REMOVE_CMAKE "cmake-build-debug/*")
list(REMOVE_ITEM SOURCE_FILES ${REMOVE_CMAKE})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} lcm_types lcm)