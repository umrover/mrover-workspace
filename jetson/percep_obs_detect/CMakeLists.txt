cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
option(VIEWER_ONLY "run without CUDA" OFF)
option(WITH_JARVIS "compile with jarvis" OFF)
project(obs_detection)
set(CMAKE_CXX_STANDARD 11)
set(OpenGL_GL_PREFERENCE GLVND)
add_compile_options(-DROOT_DIR="${CMAKE_CURRENT_LIST_DIR}")
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
if (VIEWER_ONLY)
    add_compile_options(-DVIEWER_ONLY)
    include_directories(SYSTEM ${GLFW_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/include/viewer
            ${CMAKE_CURRENT_SOURCE_DIR}/include/reader
            ${CMAKE_CURRENT_SOURCE_DIR}/include/viewer/imgui)
    file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/*.cpp")
    add_executable(viewer ${SOURCE_FILES})
    set_target_properties(viewer PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(viewer ${CMAKE_DL_LIBS} ${SPECIAL_OS_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
            ${GLEW_LIBRARIES}
            glfw)
else ()
    SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda/bin/nvcc)
    SET(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    enable_language(CUDA)
    find_package(ZED 3 REQUIRED)
    include_directories(SYSTEM ${GLFW_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    include_directories(/usr/local/cuda/samples/common/inc)
    include_directories($ENV{HOME}/.mrover/build_env/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/include/viewer
            ${CMAKE_CURRENT_SOURCE_DIR}/include/viewer/imgui
            ${CMAKE_CURRENT_SOURCE_DIR}/include/obstacle
            ${CMAKE_CURRENT_SOURCE_DIR}/include/helper
            ${CMAKE_CURRENT_SOURCE_DIR}/include/camera
            ${CMAKE_CURRENT_SOURCE_DIR}/include/reader
            ${CMAKE_CURRENT_SOURCE_DIR}/include/ar_tag)
    # TODO: Update this
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories($ENV{HOME}/mrover-workspace/3rdparty/rapidjson)
    # END
    link_directories($ENV{HOME}/.mrover/build_env/lib)
    link_directories($ENV{HOME}/.mrover/build_env/lib64)
    if (WITH_JARVIS)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{HOME}/.mrover/build_env/bin/)
    endif ()
    file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu")
    add_executable(jetson_percep_obs_detect ${SOURCE_FILES})
    link_directories(/snap/code/85/usr/lib/x86_64-linux-gnu)
    target_link_libraries(jetson_percep_obs_detect ${CMAKE_DL_LIBS} ${SPECIAL_OS_LIBS} ${ZED_LIBRARIES} ${OPENGL_LIBRARIES}
            ${GLUT_LIBRARY}
            ${GLEW_LIBRARIES}
            glfw
            ${OpenCV_LIBS}
            )
    if (WITH_JARVIS)
        target_link_libraries(jetson_percep_obs_detect lcm)
    else ()
        find_package(lcm REQUIRED)
        include(${LCM_USE_FILE})
        include_directories(${LCM_INCLUDE_DIRS})
        set(LIBS ${LIBS} ${LCM_LIBRARIES})
        file(GLOB_RECURSE LCM_FILES "../../rover_msgs/*.lcm")
        lcm_wrap_types(
                CPP_HEADERS cpp_headers
                ${LCM_FILES}
        )
        lcm_add_library(my_lcmtypes-cpp CPP ${cpp_headers})
        target_include_directories(my_lcmtypes-cpp INTERFACE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
        target_link_libraries(jetson_percep_obs_detect lcm my_lcmtypes-cpp ${LIBS})
    endif ()
    if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
        target_compile_options(jetson_percep_obs_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
                --use_fast_math
                >
                -g
                )
    else ()
        target_compile_options(jetson_percep_obs_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
                #-res-usage
                --use_fast_math>)
    endif ()

endif ()
