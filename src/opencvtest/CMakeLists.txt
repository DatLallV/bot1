cmake_minimum_required(VERSION 3.0.2)
project(opencvtest)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

## Find OpenCV
find_package(OpenCV REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4)
include_directories(${OpenCV_INCLUDE_DIRS})

## Add custom message files
add_message_files(
  FILES
  pixel_point0.msg
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS OpenCV
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Executable
add_executable(sorting src/sorting.cpp)

## Link libraries
target_link_libraries(sorting
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
