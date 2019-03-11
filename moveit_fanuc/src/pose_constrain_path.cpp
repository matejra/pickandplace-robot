/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <camera_to_cv/points_array.h>
#include <camera_to_cv/table_properties.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <curl/curl.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "set_io/set_io.h"


float table_x;
float table_y;
float camera_x, camera_y;
std::vector<geometry_msgs::Point> objcoord_vec;
std::vector<geometry_msgs::Point> holescoord_vec;
std::vector<geometry_msgs::Point>object_absolute_position_vec;
std::vector<geometry_msgs::Point>holes_absolute_position_vec;
std::vector<geometry_msgs::Point>holes_absolute_position_vec_corrected;
geometry_msgs::Point holes_absolute_position_corrected;
geometry_msgs::Point object_absolute_position;
geometry_msgs::Point holes_absolute_position;
bool table_found_var = false;
bool flag_firsttime = 0;
int num_objects = 0;
int num_holes = 0;
int numobj_belowzero = 0;
int numobj_abovezero = 0;
// Initialize working table width and height constants, if table is found program corrects table size in table_properties_clbk
float pixel_to_m_table = 836; // initialize pixel to m ratio, if table is not found. 839 px = 1m
int pixel_to_m_object = 875; // pixel to m ratio on height of objects
float working_table_w = 418;
float working_table_h = 250;
float center_proximity = 0.005;
int optical_center_x = 227;
int optical_center_y = 209;
const double jump_threshold = 0.0;
const double eef_step = 0.01;
const double eef_step_pickplace = 0.002;
double fraction = 0;
float inv_sc_proj_mat[3][3] = {
     {0.00116,     0,     -0.34389},
     {0,     0.00116,     -0.02583},
     {0,     0,     0.84388}
    }; 
//char* robot_ip = (char*)"robottest32.000webhostapp.com/action_page.php?";
char* robot_ip = (char*)"192.168.1.50/KAREL/ros_cgio?";
char* io_op = (char*)"write";
char* io_type = (char*)"9";
char* io_idx_open = (char*)"7";
char* io_idx_close = (char*)"8";
char* io_val = (char*)"1";
std::string successstring;
std::string responsestring;

void table_found_clbk(const std_msgs::Bool::ConstPtr& found) {
    table_found_var = found->data;
}

void table_properties_clbk(const camera_to_cv::table_properties::ConstPtr& prop) {
    working_table_w = prop->width;
    working_table_h = prop->height;
    pixel_to_m_table = working_table_w/0.50;
}

void objects_centers_clbk(const camera_to_cv::points_array::ConstPtr& msg) {
    //ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
    // clear vector of objects
    objcoord_vec.clear();
    for (int i = 0; i < (msg->points.size()); i++) 
    {
      // add objects center points to vector
      objcoord_vec.push_back(msg->points[i]);
      num_objects = msg->points.size();
    }
}

void holes_centers_clbk(const camera_to_cv::points_array::ConstPtr& msg) {
    //ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
    // clear vector of objects
    holescoord_vec.clear();
    for (int i = 0; i < (msg->points.size()); i++) 
    {
      // add objects center points to vector
      holescoord_vec.push_back(msg->points[i]);
      num_holes = msg->points.size();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = node_handle.subscribe("object_chatter", 1000, objects_centers_clbk);
  ros::Subscriber sub_holes = node_handle.subscribe("holes_chatter", 1000, holes_centers_clbk);
  ros::Subscriber sub_table_found = node_handle.subscribe ("table_found", 1000, table_found_clbk);
  ros::Subscriber sub_table_properties = node_handle.subscribe ("table_properties_chatter", 1000, table_properties_clbk);
  ros::ServiceClient client = node_handle.serviceClient<set_io::set_io>("set_io");
  set_io::set_io srv;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  // Define planning and move group
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::TransformStamped transformStamped_table;
    try{
      transformStamped_table = tfBuffer.lookupTransform("base_link", "working_table_fr",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      //ros::Duration(1.0).sleep();
      //continue;
    }

  table_x = transformStamped_table.transform.translation.x;
  table_y = transformStamped_table.transform.translation.y;

  geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", "camera_frame",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

  camera_x = transformStamped.transform.translation.x;
  camera_y = transformStamped.transform.translation.y;

  if (objcoord_vec.size() != 0 && table_found_var == true && flag_firsttime == 0) 
  {
    object_absolute_position_vec.clear();
    for (int i = 0; i < num_objects; i++)
    {
      object_absolute_position.x = camera_x+inv_sc_proj_mat[1][1]*(objcoord_vec[i].y-optical_center_y);
      object_absolute_position.y = camera_y+inv_sc_proj_mat[0][0]*(objcoord_vec[i].x-optical_center_x);
      object_absolute_position_vec.push_back(object_absolute_position);
    }
  }

  if (holescoord_vec.size() != 0 && table_found_var == true && flag_firsttime == 0) 
  {
    holes_absolute_position_vec.clear();
    for (int i = 0; i < num_holes; i++)
    {
      holes_absolute_position.x = camera_x+(holescoord_vec[i].y-optical_center_y)/pixel_to_m_table;
      holes_absolute_position.y = camera_y+(holescoord_vec[i].x-optical_center_x)/pixel_to_m_table + 0.002;
      holes_absolute_position_vec.push_back(holes_absolute_position);
    }
    flag_firsttime = 1;
  }

  struct is_greater
  {
    bool operator()(const geometry_msgs::Point& x, const geometry_msgs::Point& y) const
    {
      return x.y > y.y;  // x.y > y.y first y higher than second y (sort descending?)
    }
  };



  // correction of coordinates
  holes_absolute_position_vec_corrected.clear();
  float holes_absolute_position_vec_coordinates_x[] = {0.3267, 0.3536, 0.3800, 0.3800, 0.4060, 0.4060, 0.4335, 0.4335, 0.4595, 0.4860};
  float holes_absolute_position_vec_coordinates_y[] = {0.0733, 0.1191, 0.0733, 0.164, 0.1191, 0.2115, 0.0733, 0.164, 0.1191, 0.0733};
  for (int i = 0; i < num_holes; i++)
  {
    holes_absolute_position_corrected.x = holes_absolute_position_vec_coordinates_x[i];
    holes_absolute_position_corrected.y = holes_absolute_position_vec_coordinates_y[i];
    holes_absolute_position_vec_corrected.push_back(holes_absolute_position_corrected);
  }

  for (int i = 0; i < num_holes; i++) 
  {
    for (int j = 0; j < holes_absolute_position_vec_corrected.size(); j++)
    {
      if (holes_absolute_position_vec[i].x  < (holes_absolute_position_vec_corrected[j].x + center_proximity) &&
      holes_absolute_position_vec[i].x  > (holes_absolute_position_vec_corrected[j].x - center_proximity)) 
      {
        holes_absolute_position_vec[i].x = holes_absolute_position_vec_corrected[j].x;
      }

      if (holes_absolute_position_vec[i].y  < (holes_absolute_position_vec_corrected[j].y + center_proximity) &&
      holes_absolute_position_vec[i].y  > (holes_absolute_position_vec_corrected[j].y - center_proximity)) 
      {
        holes_absolute_position_vec[i].y = holes_absolute_position_vec_corrected[j].y;
      }
    }
  }
  std::sort(object_absolute_position_vec.begin(), object_absolute_position_vec.end(), is_greater());
  std::sort(holes_absolute_position_vec.begin(), holes_absolute_position_vec.end(), is_greater());

  for (int i = 0; i < num_holes; i++) // sorted
  {
      std::cout << "Hole[" << i << "]  x: " << holes_absolute_position_vec[i].x << ", y: " << holes_absolute_position_vec[i].y << "\n";
  }
  for (int i = 0; i < num_objects; i++) // sorted z
  {
      std::cout << "Object[" << i << "]  x: " << object_absolute_position_vec[i].x << ", y: " << object_absolute_position_vec[i].y << "\n";
  }

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  //////////////////////////////// start collision defining ///////////////////////////////////
  /// define the objects as collision object to attach
  

  ///////////// RIGHT PART OF THE PROTECTING CASE AS COLLISION OBJECT ROS MESSAGE
  moveit_msgs::CollisionObject collision_object_right;
  collision_object_right.header.frame_id = move_group.getPlanningFrame();
  collision_object_right.id = "rightpart";

  // Define a left-right box to add to the world.
  shape_msgs::SolidPrimitive primitive_leftright;
  primitive_leftright.type = primitive_leftright.BOX;
  primitive_leftright.dimensions.resize(3);
  primitive_leftright.dimensions[0] = 1.1;
  primitive_leftright.dimensions[1] = 0.02;
  primitive_leftright.dimensions[2] = 0.8;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_right;
  box_pose_right.orientation.w = 1.0;
  box_pose_right.position.x = 0.2;
  box_pose_right.position.y = -0.29;
  box_pose_right.position.z = 0.39;

  collision_object_right.primitives.push_back(primitive_leftright);
  collision_object_right.primitive_poses.push_back(box_pose_right);
  collision_object_right.operation = collision_object_right.ADD;
  ///////////// END RIGHT PART

  ///////////// LEFT PART OF THE PROTECTING CASE AS COLLISION OBJECT ROS MESSAGE
  moveit_msgs::CollisionObject collision_object_left;
  collision_object_left.header.frame_id = move_group.getPlanningFrame();
  collision_object_left.id = "leftpart";

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_left;
  box_pose_left.orientation.w = 1.0;
  box_pose_left.position.x = 0.2;
  box_pose_left.position.y = 0.29;
  box_pose_left.position.z = 0.39;

  collision_object_left.primitives.push_back(primitive_leftright);
  collision_object_left.primitive_poses.push_back(box_pose_left);
  collision_object_left.operation = collision_object_left.ADD;
  ///////////// END LEFT PART

  ///////////// TOP PART OF THE PROTECTING CASE AS COLLISION OBJECT ROS MESSAGE
  moveit_msgs::CollisionObject collision_object_top;
  collision_object_top.header.frame_id = move_group.getPlanningFrame();
  collision_object_top.id = "toppart";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_topbottom;
  primitive_topbottom.type = primitive_topbottom.BOX;
  primitive_topbottom.dimensions.resize(3);
  primitive_topbottom.dimensions[0] = 1.1;
  primitive_topbottom.dimensions[1] = 0.60;
  primitive_topbottom.dimensions[2] = 0.02;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_top;
  box_pose_top.orientation.w = 1.0;
  box_pose_top.position.x = 0.2;
  box_pose_top.position.y = 0;
  box_pose_top.position.z = 0.8;

  collision_object_top.primitives.push_back(primitive_topbottom);
  collision_object_top.primitive_poses.push_back(box_pose_top);
  collision_object_top.operation = collision_object_top.ADD;
  ///////////// END TOP PART

  ///////////// BOTTOM PART OF THE PROTECTING CASE AS COLLISION OBJECT ROS MESSAGE
  moveit_msgs::CollisionObject collision_object_bottom;
  collision_object_bottom.header.frame_id = move_group.getPlanningFrame();
  collision_object_bottom.id = "bottompart";

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_bottom;
  box_pose_bottom.orientation.w = 1.0;
  box_pose_bottom.position.x = 0.2;
  box_pose_bottom.position.y = 0;
  box_pose_bottom.position.z = -0.02;

  collision_object_bottom.primitives.push_back(primitive_topbottom);
  collision_object_bottom.primitive_poses.push_back(box_pose_bottom);
  collision_object_bottom.operation = collision_object_bottom.ADD;
  ///////////// END BOTTOM PART

  ///////////// REAR PART OF THE PROTECTING CASE AS COLLISION OBJECT ROS MESSAGE
  moveit_msgs::CollisionObject collision_object_rear;
  collision_object_rear.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_rear.id = "rearpart";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_rearfront;
  primitive_rearfront.type = primitive_rearfront.BOX;
  primitive_rearfront.dimensions.resize(3);
  primitive_rearfront.dimensions[0] = 0.02;
  primitive_rearfront.dimensions[1] = 0.60;
  primitive_rearfront.dimensions[2] = 0.81;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_rear;
  box_pose_rear.orientation.w = 1.0;
  box_pose_rear.position.x = -0.36;
  box_pose_rear.position.y = 0;
  box_pose_rear.position.z = 0.4;

  collision_object_rear.primitives.push_back(primitive_rearfront);
  collision_object_rear.primitive_poses.push_back(box_pose_rear);
  collision_object_rear.operation = collision_object_rear.ADD;
  ///////////// END REAR PART

  ///////////// FRONT PART OF THE PROTECTING CASE AS COLLISION OBJECT ROS MESSAGE
  moveit_msgs::CollisionObject collision_object_front;
  collision_object_front.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_front.id = "frontpart";

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_front;
  box_pose_front.orientation.w = 1.0;
  box_pose_front.position.x = 0.74;
  box_pose_front.position.y = 0;
  box_pose_front.position.z = 0.4;

  collision_object_front.primitives.push_back(primitive_rearfront);
  collision_object_front.primitive_poses.push_back(box_pose_front);
  collision_object_front.operation = collision_object_front.ADD;
  ///////////// END FRONT PART

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object_right);
  collision_objects.push_back(collision_object_left);
  collision_objects.push_back(collision_object_top);
  collision_objects.push_back(collision_object_bottom);
  collision_objects.push_back(collision_object_rear);
  //collision_objects.push_back(collision_object_front);

  std::vector<moveit_msgs::ObjectColor> collision_colors;
  for (int i = 0; i < collision_objects.size(); i++)
  {
    collision_colors.emplace_back();
    collision_colors.back().color.a= 0.2;
    collision_colors.back().color.r= 0;
    collision_colors.back().color.g= 1;
    collision_colors.back().color.b= 0;
  }
  //////////////////////////////// end collision defining ///////////////////////////////////

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.applyCollisionObjects(collision_objects, collision_colors);
  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Planning to a joint-space goal: Endeffector facing down to the table
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions_home;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_home);
  joint_group_positions_home[0] = 0;
  joint_group_positions_home[1] = -0.3879;
  joint_group_positions_home[2] = -0.60349;
  joint_group_positions_home[3] = 0;
  joint_group_positions_home[4] = -1.35499;
  joint_group_positions_home[5] = 0;  // radians  move_group.setJointValueTarget(joint_group_positions);
  move_group.setJointValueTarget(joint_group_positions_home);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing getting to start position %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  move_group.move();
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // Defining orientation constants
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "flange";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 0.00;
  ocm.orientation.y = 0.707;
  ocm.orientation.z = 0.00;
  ocm.orientation.w = 0.707;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  move_group.clearPathConstraints();

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  int j = 0;
  int k = 0;
  geometry_msgs::Pose locate_object;
  geometry_msgs::Pose pick;
  geometry_msgs::Pose locate_hole;
  geometry_msgs::Pose place;
  moveit::planning_interface::MoveGroupInterface::Plan plan_locate_object;
  moveit::planning_interface::MoveGroupInterface::Plan plan_pick;
  moveit::planning_interface::MoveGroupInterface::Plan plan_locate_hole;
  moveit::planning_interface::MoveGroupInterface::Plan plan_place;
  while( j < num_objects )
  {
    robot_state::RobotState start_state(*move_group.getCurrentState());

    
    while (object_absolute_position_vec[j].y > 0) {
      j++; // go forward through object until you get an object that has y below zero
    }
    if (j >= object_absolute_position_vec.size())
    {
      ROS_INFO_NAMED("tutorial", "All objects have positive y coordinate. Aborting.");
      break;
    }

    srv.request.robot_ip = robot_ip;
    srv.request.io_op = io_op;
    srv.request.io_type = io_type;
    srv.request.io_val = io_val;
    srv.request.io_idx = io_idx_open;

    /*if (client.call(srv))
    {
      if (&responsestring[0] != successstring)
      {
        ROS_ERROR("Failed to open/close the gripper");
        return 1;   
        // SLEEP 3 SEC, TRY AGAIN
      } else {
        ROS_INFO("Opening/closing the gripper SUCEEDED");
      }    
    }
    else
    {
      ROS_ERROR("Failed to call service set_io");
      return 1;
    }*/

    move_group.setStartState(start_state);
    std::cout << "Object to pick: x " << object_absolute_position_vec[j].x << "y " << object_absolute_position_vec[j].y;
    //target_pose_pickplace = move_group.getCurrentPose().pose;
    locate_object.position.x = object_absolute_position_vec[j].x;
    locate_object.position.y = object_absolute_position_vec[j].y;
    locate_object.position.z = 0.23;
    locate_object.orientation.x=0.00;
    locate_object.orientation.y=0.707;
    locate_object.orientation.z = 0.00;
    locate_object.orientation.w = 0.707;
    move_group.setPoseTarget(locate_object);
    move_group.setPlanningTime(20.0);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    moveit_msgs::RobotTrajectory trajectory_msg;
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);
    success = iptp.computeTimeStamps(rt);
    rt.getRobotTrajectoryMsg(trajectory_msg);
    plan_locate_object.trajectory_ = trajectory_msg;
    //success = (move_group.plan(plan_locate_object) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing locating the object %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    //bool blocking=false;
    //moveit::core::RobotStateConstPtr robot_state_const = move_group.getCurrentState();
    visual_tools.publishTrajectoryLine(plan_locate_object.trajectory_, joint_model_group);
    //visual_tools.publishTrajectoryPath(plan_locate_object.trajectory_, robot_state_const, blocking);
    std::cout << plan_locate_object.trajectory_;
    visual_tools.trigger();
    visual_tools.prompt("execute trajectory and show the plan for another one");
    move_group.move();

    start_state = *move_group.getCurrentState();
    move_group.setStartState(start_state);
    //start_state = *move_group.getCurrentState();
    pick = locate_object;
    pick.position.z = 0.20;
    move_group.setPoseTarget(pick);
    move_group.setPlanningTime(20.0);
    move_group.setMaxVelocityScalingFactor(0.1);
    success = (move_group.plan(plan_pick) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing picking the object %s",success?"":"FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan_pick.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("execute trajectory and show the plan for another one");
    move_group.move();

    sleep(0.5);

    srv.request.io_idx = io_idx_close;

    successstring = "1";

    /*if (client.call(srv))
    {
      responsestring = srv.response.success.c_str();
      //std::cout << "response" << srv.response.success << "\n";
      if (&responsestring[0] != successstring)
      {
        ROS_ERROR("Failed to open/close the gripper");
        return 1;   
        // SLEEP 3 SEC, TRY AGAIN
      } else {
        ROS_INFO("Opening/closing the gripper SUCEEDED");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service set_io");
      return 1;
    }*/

    start_state = *move_group.getCurrentState();
    move_group.setStartState(start_state);
    //start_state = *move_group.getCurrentState();
    move_group.setPoseTarget(locate_object);
    move_group.setPlanningTime(20.0);
    move_group.setMaxVelocityScalingFactor(1.0);
    success = (move_group.plan(plan_locate_object) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing picking the object %s",success?"":"FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan_locate_object.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("execute trajectory and show the plan for another one");
    move_group.move();

    while (holes_absolute_position_vec[k].y < 0) { // add error if there is no hole that exists
      k++;
    }

    locate_hole.position.x = holes_absolute_position_vec[k].x;
    locate_hole.position.y = holes_absolute_position_vec[k].y;
    locate_hole.position.z = 0.23;
    locate_hole.orientation.x=0.00;
    locate_hole.orientation.y=0.707;
    locate_hole.orientation.z = 0.00;
    locate_hole.orientation.w = 0.707;
    start_state = *move_group.getCurrentState();
    move_group.setStartState(start_state);
    move_group.setPoseTarget(locate_hole);
    move_group.setPlanningTime(20.0);
    success = (move_group.plan(plan_locate_hole) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing picking the object %s",success?"":"FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan_locate_hole.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("execute trajectory and show the plan for another one");
    move_group.move();

    place = locate_hole;
    place.position.z = 0.20;
    start_state = *move_group.getCurrentState();
    move_group.setStartState(start_state);
    move_group.setPoseTarget(place);
    move_group.setPlanningTime(20.0);
    move_group.setMaxVelocityScalingFactor(0.1);
    success = (move_group.plan(plan_place) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing picking the object %s",success?"":"FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan_place.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("execute trajectory and show the plan for another one");
    move_group.move();

    sleep(0.5);
    srv.request.io_idx = io_idx_open;

    /*if (client.call(srv))
    {
      if (&responsestring[0] != successstring)
      {
        ROS_ERROR("Failed to open/close the gripper");
        return 1;   
        // SLEEP 3 SEC, TRY AGAIN
      } else {
        ROS_INFO("Opening/closing the gripper SUCEEDED");
      }    
    }
    else
    {
      ROS_ERROR("Failed to call service set_io");
      return 1;
    }*/

    start_state = *move_group.getCurrentState();
    move_group.setStartState(start_state);
    //start_state = *move_group.getCurrentState();
    move_group.setPoseTarget(locate_hole);
    move_group.setPlanningTime(20.0);
    move_group.setMaxVelocityScalingFactor(1.0);
    success = (move_group.plan(plan_locate_hole) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing going up from locating the hole %s",success?"":"FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(plan_locate_hole.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("execute trajectory and show the plan for another one");
    move_group.move();

    j++;
    k++;

    if (j == num_objects)
    {
      current_state = move_group.getCurrentState();
      std::vector<double> joint_group_positions_home;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_home);
      // Modify the fifth joint to get the manipulator perpendicular to the working table
      joint_group_positions_home[0] = 0;
      joint_group_positions_home[1] = -0.3879;
      joint_group_positions_home[2] = -0.60349;
      joint_group_positions_home[3] = 0;
      joint_group_positions_home[4] = -1.35499;
      joint_group_positions_home[5] = 0;  // radians
      move_group.setJointValueTarget(joint_group_positions_home);

      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
        ROS_INFO_NAMED("tutorial", "Visualizing robot going to start position");
      } else {
        ROS_INFO_NAMED("tutorial", "Visualizing robot going to start position FAILED");
      }

      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal - start position", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      move_group.setJointValueTarget(joint_group_positions_home);
      move_group.move();
    }
  }
  // Define the start state


  /*geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.075;
  target_pose1.position.z = 0.30;
  target_pose1.orientation.x=0.00;
  target_pose1.orientation.y=0.707;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.707;
  // Set the pose target
  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(100.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan (constraints) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan. 

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");
  move_group.move();*/

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();


  // Since we set the start state we have to clear it before planning other paths
  // move_group.setStartStateToCurrentState();  
  

  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
