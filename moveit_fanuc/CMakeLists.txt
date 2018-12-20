cmake_minimum_required(VERSION 2.8.3)
project(moveit_fanuc)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
  camera_to_cv
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})


