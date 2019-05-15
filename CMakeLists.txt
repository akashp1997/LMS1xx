cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)

# Build ROS-independent library.
find_package(pybind11 REQUIRED)
include_directories(include)
add_library(lms1xx MODULE src/LMS1xx.cpp)
target_link_libraries(lms1xx PRIVATE pybind11::module)

set_target_properties(lms1xx PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}" SUFFIX "${PYTHON_MODULE_EXTENSION}")
