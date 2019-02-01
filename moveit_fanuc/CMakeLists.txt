cmake_minimum_required(VERSION 2.8.3)
project(moveit_fanuc)
add_compile_options(-std=c++11)
set(CURL_LIBRARY "-lcurl") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
  camera_to_cv
  set_io
  tf2_ros
  interactive_markers
  moveit_ros_planning
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_eigen
  tf2_geometry_msgs
)
find_package(CURL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
  interactive_markers
  LIBRARIES
  interactivity_utils
  DEPENDS
  EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIR}
)


add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
add_executable(pick_place src/pick_place.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURL_LIBRARIES})
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURL_LIBRARIES})

