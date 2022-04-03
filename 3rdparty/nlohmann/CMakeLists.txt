CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
if(POLICY CMP0025)
  # detect Apple's Clang
  cmake_policy(SET CMP0025 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

PROJECT(nlohmann CXX)

# compile in release with debug info mode by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

# Build all binaries in a separate directory
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(NLOHMANN_BUILD_CXX11 "Build nlohmann with C++11 (gcc/clang)" ON)
if(NLOHMANN_BUILD_CXX11)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
endif()

option(NLOHMANN_BUILD_ASAN "Build nlohmann with address sanitizer (gcc/clang)" OFF)
option(NLOHMANN_BUILD_UBSAN "Build nlohmann with undefined behavior sanitizer (gcc/clang)" OFF)

option(NLOHMANN_ENABLE_INSTRUMENTATION_OPT "Build nlohmann with -march or -mcpu options" ON)

option(NLOHMANN_HAS_STDSTRING "" OFF)
if(NLOHMANN_HAS_STDSTRING)
    add_definitions(-DNLOHMANN_HAS_STDSTRING)
endif()

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments -fcolor-diagnostics")
    endif()
endif(CCACHE_FOUND)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(NLOHMANN_ENABLE_INSTRUMENTATION_OPT AND NOT CMAKE_CROSSCOMPILING)
        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "powerpc" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64le")
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=native")
        else()
          #FIXME: x86 is -march=native, but doesn't mean every arch is this option. To keep original project's compatibility, I leave this except POWER.
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
        endif()
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
    set(EXTRA_CXX_FLAGS -Weffc++ -Wswitch-default -Wfloat-equal -Wconversion -Wsign-conversion)
    if (NLOHMANN_BUILD_CXX11 AND CMAKE_VERSION VERSION_LESS 3.1)
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.7.0")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        endif()
    endif()
    if (NLOHMANN_BUILD_ASAN)
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.8.0")
            message(FATAL_ERROR "GCC < 4.8 doesn't support the address sanitizer")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
        endif()
    endif()
    if (NLOHMANN_BUILD_UBSAN)
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.9.0")
            message(FATAL_ERROR "GCC < 4.9 doesn't support the undefined behavior sanitizer")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
        endif()
    endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(NOT CMAKE_CROSSCOMPILING)
      if(CMAKE_SYSTEM_PROCESSOR STREQUAL "powerpc" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "ppc64le")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=native")
      else()
        #FIXME: x86 is -march=native, but doesn't mean every arch is this option. To keep original project's compatibility, I leave this except POWER.
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
      endif()
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-missing-field-initializers")
    set(EXTRA_CXX_FLAGS -Weffc++ -Wswitch-default -Wfloat-equal -Wconversion -Wimplicit-fallthrough)
    if (NLOHMANN_BUILD_CXX11 AND CMAKE_VERSION VERSION_LESS 3.1)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
    if (NLOHMANN_BUILD_ASAN)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
    endif()
    if (NLOHMANN_BUILD_UBSAN)
        if (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined-trap -fsanitize-undefined-trap-on-error")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
        endif()
    endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS=1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "XL")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -qarch=auto")
endif()

#add extra search paths for libraries and includes
SET(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "The directory the headers are installed in")
SET(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE STRING "Directory where lib will install")

IF(UNIX OR CYGWIN)
    SET(_CMAKE_INSTALL_DIR "${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}")
ELSEIF(WIN32)
    SET(_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/cmake")
ENDIF()
SET(CMAKE_INSTALL_DIR "${_CMAKE_INSTALL_DIR}" CACHE PATH "The directory cmake files are installed in")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_custom_target(travis_doc)
add_custom_command(TARGET travis_doc
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/travis-doxygen.sh)

install(DIRECTORY include/nlohmann
    DESTINATION "${INCLUDE_INSTALL_DIR}"
    COMPONENT dev)

# Provide config and version files to be used by other applications
# ===============================

################################################################################
# Export package for use from the build tree
EXPORT( PACKAGE ${PROJECT_NAME} )

# Create the RapidJSONConfig.cmake file for other cmake projects.
# ... for the build tree
SET( CONFIG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET( CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR})
SET( ${PROJECT_NAME}_INCLUDE_DIR "\${${PROJECT_NAME}_SOURCE_DIR}/include" )

CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY )
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}ConfigVersion.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

# ... for the install tree
SET( CMAKECONFIG_INSTALL_DIR lib/cmake/${PROJECT_NAME} )
FILE( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include" )

SET( ${PROJECT_NAME}_INCLUDE_DIR "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
SET( CONFIG_SOURCE_DIR )
SET( CONFIG_DIR )
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake @ONLY )

INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

# Install files
INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "${CMAKE_INSTALL_DIR}"
    COMPONENT dev)
