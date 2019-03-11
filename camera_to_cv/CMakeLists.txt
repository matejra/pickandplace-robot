cmake_minimum_required(VERSION 2.8.3)
project(camera_to_cv)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)
add_message_files(
  FILES
  points_array.msg
  table_properties.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)
find_package(OpenCV REQUIRED)
catkin_package(
CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(camera_to_cv_node src/image_converter.cpp)
add_executable(camera_to_cv_node_old src/image_converter_old.cpp)
target_link_libraries(camera_to_cv_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(camera_to_cv_node_old ${OpenCV_LIBS} ${catkin_LIBRARIES})
