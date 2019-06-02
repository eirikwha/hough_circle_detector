cmake_minimum_required(VERSION 2.8.3)
project(hough_circle_detector)

## Use C++11
add_definitions(--std=c++11)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    sensor_msgs
    cv_bridge
)

find_package( OpenCV REQUIRED )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake matching_config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
#  LIBRARIES
    CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    cv_bridge
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/hough_circle_detector/*.cpp")

## Declare a C++ library
add_library(${PROJECT_NAME}
            ${SOURCES}
        )

## Declare cpp executables

add_executable(
        circle_detector_class_node
        src/circle_detector_node.cpp
)

add_executable(
    circle_detector
        src/circle_detector.cpp
)

add_executable(
        circle_detector_param_tuner
        src/circle_detector_param_tuner.cpp
)

## Specify libraries to link executable targets against

target_link_libraries(
    circle_detector
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
        circle_detector_class_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

target_link_libraries(
        circle_detector_param_tuner
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
