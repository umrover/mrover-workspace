cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
option(VIEWER_ONLY "run without CUDA" OFF)
option(NO_JARVIS "compile without jarvis" OFF)

project(obs_detection)
set(CMAKE_CXX_STANDARD 11)
#add_compile_definitions(ROOT_DIR=${CMAKE_CURRENT_LIST_DIR})
add_compile_options(-DROOT_DIR="${CMAKE_CURRENT_LIST_DIR}")

find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

if (VIEWER_ONLY)
    #add_compile_definitions(VIEWER_ONLY)
    add_compile_options(-DVIEWER_ONLY)

    include_directories(SYSTEM ${GLFW_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/imgui)
    file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "src/*.cpp")
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
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/imgui)
    include_directories(/usr/local/cuda-10.0/samples/common/inc)
    include_directories($ENV{HOME}/.mrover/build_env/include)


    link_directories($ENV{HOME}/.mrover/build_env/lib)
    link_directories($ENV{HOME}/.mrover/build_env/lib64)

    if (NO_JARVIS)
        add_compile_options(-DNO_JARVIS)
    else ()
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{HOME}/.mrover/build_env/bin/)
    endif ()

    file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "src/*.cpp" "src/*.cu")
    add_executable(jetson_percep_obs_detect ${SOURCE_FILES})
    link_directories(/snap/code/85/usr/lib/x86_64-linux-gnu)
    target_link_libraries(jetson_percep_obs_detect ${CMAKE_DL_LIBS} ${SPECIAL_OS_LIBS} ${ZED_LIBRARIES} ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    ${GLEW_LIBRARIES}
    glfw
    )
    if (NO_JARVIS)
    else ()
        target_link_libraries(jetson_percep_obs_detect lcm)
    endif ()

    target_compile_options(jetson_percep_obs_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
            #-res-usage
            --use_fast_math
            >)
endif ()
