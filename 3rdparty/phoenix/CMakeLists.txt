cmake_minimum_required(VERSION 2.8)

PROJECT(phoenix CXX)

set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")

if (${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*aarch64.*")
    set(ARCH "arm")
else()
    set(ARCH "x86_64")
endif()

install(DIRECTORY "include/"
        DESTINATION "${INCLUDE_INSTALL_DIR}"
        COMPONENT dev)

install(DIRECTORY "lib/${ARCH}/"
        DESTINATION "${LIB_INSTALL_DIR}"
        COMPONENT dev)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc.in
               ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
               @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
        DESTINATION "${LIB_INSTALL_DIR}/pkgconfig"
        COMPONENT pkgconfig)
