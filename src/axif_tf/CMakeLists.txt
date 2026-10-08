cmake_minimum_required(VERSION 3.0.2)
project(axif_tf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  image_transport
  sensor_msgs
  geometry_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf2_ros tf2_geometry_msgs image_transport sensor_msgs geometry_msgs cv_bridge
)

add_executable(getmarker src/getmarker.cpp)
target_link_libraries(getmarker 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)
