cmake_minimum_required(VERSION 3.10)

project(viewer-test)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(viewer_test viewer_test.cpp)
target_link_libraries(viewer_test ${VISP_LIBRARIES})
