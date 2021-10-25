cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda/bin/nvcc)
SET(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc) 
enable_language(CUDA)

project(obs_detection)

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

link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})


add_executable (driver src/filter.cu src/obs-detector.cu src/viewer.cu src/common.cu src/voxel-grid.cu src/plane-ransac.cu src/euclidean-cluster.cu src/pass-through.cu src/find-clear.cu)
target_link_libraries (driver ${ZED_LIBRARIES})
target_link_libraries(driver 
                        ${SPECIAL_OS_LIBS} 
                        ${ZED_LIBS} 
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARY}
                        ${GLEW_LIBRARIES})
target_compile_options(driver PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
                       #-res-usage
                       --use_fast_math
                       >)
