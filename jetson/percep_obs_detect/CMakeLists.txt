cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
option(VIEWER_ONLY "run without CUDA" OFF)
option(NO_JARVIS "compile without jarvis" OFF)

project(obs_detection)
#add_compile_definitions(ROOT_DIR=${CMAKE_CURRENT_LIST_DIR})
add_compile_options(-DROOT_DIR="${CMAKE_CURRENT_LIST_DIR}")

if(VIEWER_ONLY)
#add_compile_definitions(VIEWER_ONLY)
add_compile_options(-DVIEWER_ONLY)

find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})
include_directories(SYSTEM ${GLUT_INCLUDE_PATH})
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_executable (viewer src/viewer.cpp)
set_target_properties(viewer PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(viewer 
                        ${SPECIAL_OS_LIBS} 
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARY}
                        ${GLEW_LIBRARIES})


else() 
SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda/bin/nvcc)
SET(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)


find_package(ZED 3 REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)


include_directories(SYSTEM ${ZED_INCLUDE_DIRS})
include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})
include_directories(SYSTEM ${GLUT_INCLUDE_PATH})
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/usr/local/cuda-10.0/samples/common/inc)
include_directories($ENV{HOME}/.mrover/build_env/include)


link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories($ENV{HOME}/.mrover/build_env/lib)
link_directories($ENV{HOME}/.mrover/build_env/lib64)

if(NO_JARVIS)
add_compile_options(-DNO_JARVIS)
else()
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{HOME}/.mrover/build_env/bin/)
endif()

add_executable (jetson_percep_obs_detect src/filter.cu src/obs-detector.cu src/viewer.cpp src/common.cu src/voxel-grid.cu src/plane-ransac.cu src/euclidean-cluster.cu src/pass-through.cu src/find-clear-path.cu)
target_link_libraries (jetson_percep_obs_detect ${ZED_LIBRARIES})
target_link_libraries(jetson_percep_obs_detect
                        ${SPECIAL_OS_LIBS}
                        ${ZED_LIBS}
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARY}
                        ${GLEW_LIBRARIES})
if(NO_JARVIS)
else()
target_link_libraries (jetson_percep_obs_detect lcm)
endif()

target_compile_options(jetson_percep_obs_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
                       #-res-usage
                       --use_fast_math
                       >)
endif()
