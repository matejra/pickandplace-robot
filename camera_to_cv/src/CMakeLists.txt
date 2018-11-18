cmake_minimum_required(VERSION 2.8.3)
project(camera_to_cv)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
catkin_package(
CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(camera_to_cv_node src/image_converter.cpp)
target_link_libraries(camera_to_cv_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
