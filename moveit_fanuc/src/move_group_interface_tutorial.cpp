#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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


  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

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
  float holes_absolute_position_vec_coordinates_x[] = {0.326, 0.3536, 0.3800, 0.3800, 0.4060, 0.4060, 0.4335, 0.4335, 0.4655, 0.4860};
  float holes_absolute_position_vec_coordinates_y[] = {0.0714, 0.1191, 0.0714, 0.164, 0.1191, 0.2115, 0.0714, 0.164, 0.1191, 0.0714};
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


  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  //////////////////////////////// start collision defining ///////////////////////////////////

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

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.applyCollisionObjects(collision_objects, collision_colors);
  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Defining orientation constants
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_6";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.707;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 0.707;
  ocm.absolute_x_axis_tolerance = 0.05;
  ocm.absolute_y_axis_tolerance = 0.05;
  ocm.absolute_z_axis_tolerance = 0.05;
  ocm.weight = 1.0;

  // quaternion goal -> 90 degrees around y-axis, EE parallel to the ground normal
  moveit_msgs::OrientationConstraint ocm_5;
  ocm_5.link_name = "link_5";
  ocm_5.header.frame_id = "base_link";
  ocm_5.orientation.x = 0.0;
  ocm_5.orientation.y = 0.707;
  ocm_5.orientation.z = 0.0;
  ocm_5.orientation.w = 0.707;
  ocm_5.absolute_x_axis_tolerance = 1.0;
  ocm_5.absolute_y_axis_tolerance = 0.05;
  ocm_5.absolute_z_axis_tolerance = 0.05;
  ocm_5.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  test_constraints.orientation_constraints.push_back(ocm_5);

  
  ////////////////////////////////////////////
  // Go to start position  - zero except Joint 5, that makes gripper facing the table
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // Modify the fifth joint to get the manipulator perpendicular to the working table
  for (int i = 0; i < 6; i++)
  {
    joint_group_positions[i] = 0.0;
  }
  joint_group_positions[4] = -1.5708;  // radians
  move_group.setJointValueTarget(joint_group_positions);

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
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();
  //////////////////////////////////////////////
  // count objects with y < 0, pick every object that has y < 0, place them in the max y available
  //numobj_belowzero = countobj_belowzero(num_objects, object_absolute_position_y_vec);
  //numobj_abovezero = countobj_abovezero(num_objects, object_absolute_position_y_vec);

  int j = 0;
  int k = 0;
  while( j < num_objects )
  {
    geometry_msgs::Pose target_pose_pickplace = move_group.getCurrentPose().pose;
    /*if (j == 0)
    {
      waypoints_pick.clear();
      waypoints_place.clear();
    }*/
    std::vector<geometry_msgs::Pose> waypoints_pick;
    std::vector<geometry_msgs::Pose> waypoints_place;
    std::vector<geometry_msgs::Pose> waypoints_up;
    std::vector<geometry_msgs::Pose> waypoints_down_obj;
    std::vector<geometry_msgs::Pose> waypoints_down_hole;

    while (object_absolute_position_vec[j].y > 0) {
      j++; // go forward through object until you get an object that has y below zero
    }
    if (j >= object_absolute_position_vec.size())
    {
      ROS_INFO_NAMED("tutorial", "All objects have positive y coordinate. Aborting.");
      break;
    }

    std::cout << "OBJ x " << object_absolute_position_vec[j].x << "y " << object_absolute_position_vec[j].y;

    target_pose_pickplace.position.z = 0.23;
    waypoints_pick.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.y = object_absolute_position_vec[j].y; 
    waypoints_pick.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.x = object_absolute_position_vec[j].x;
    waypoints_pick.push_back(target_pose_pickplace); 
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    move_group.setPlanningTime(3.0);
    trajectory.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    fraction = move_group.computeCartesianPath(waypoints_pick, eef_step, jump_threshold, trajectory, test_constraints);

    ROS_INFO_NAMED("tutorial", "Visualizing locating the object (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    trajectory.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    plan.trajectory_ = trajectory;
    sleep(1.0);
    // Visualize the plan in RViz
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group.execute(plan);
    /*sleep(1.0);
    geometry_msgs::Pose executed_pose_pickplace = move_group.getCurrentPose().pose;
    if (target_pose_pickplace.position.x > (executed_pose_pickplace.position.x - 0.005) && target_pose_pickplace.position.x < (executed_pose_pickplace.position.x + 0.005))
    {}
    else {
        ROS_INFO_NAMED("tutorial", "Object locating pose not reached!");
        break;
    }*/

    // check if execution is correct!
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      ROS_INFO_NAMED("tutorial", "Plan suceeded");
    } else {
      ROS_INFO_NAMED("tutorial", "Plan FAILED");
      return 1;
    }

    target_pose_pickplace.position.z = 0.165;
    waypoints_down_obj.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.y = object_absolute_position_vec[j].y; 
    waypoints_down_obj.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.x = object_absolute_position_vec[j].x;
    waypoints_down_obj.push_back(target_pose_pickplace); 
    move_group.setMaxVelocityScalingFactor(0.01);
    moveit_msgs::RobotTrajectory trajectory_down;
    move_group.setPlanningTime(3.0);
    trajectory_down.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    fraction = move_group.computeCartesianPath(waypoints_down_obj, eef_step_pickplace, jump_threshold, trajectory_down, test_constraints);

    ROS_INFO_NAMED("tutorial", "Visualizing picking up the object (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan_down;
    trajectory_down.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    plan_down.trajectory_ = trajectory_down;
    sleep(1.0);
    // Visualize the plan in RViz
    visual_tools.trigger();
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group.execute(plan_down);

    // check if execution is correct!
    success = (move_group.plan(plan_down) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      ROS_INFO_NAMED("tutorial", "Plan suceeded");
    } else {
      ROS_INFO_NAMED("tutorial", "Plan FAILED");
      return 1;
    }
    srv.request.robot_ip = robot_ip;
    srv.request.io_op = io_op;
    srv.request.io_type = io_type;
    srv.request.io_idx = io_idx_close;
    srv.request.io_val = io_val;

    successstring = "1";

    if (client.call(srv))
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
    }

    target_pose_pickplace.position.z = 0.23;
    waypoints_up.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.y = object_absolute_position_vec[j].y; 
    waypoints_up.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.x = object_absolute_position_vec[j].x;
    waypoints_up.push_back(target_pose_pickplace); 
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory_up;
    move_group.setPlanningTime(3.0);
    trajectory_up.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    fraction = move_group.computeCartesianPath(waypoints_up, eef_step_pickplace, jump_threshold, trajectory_up, test_constraints);

    ROS_INFO_NAMED("tutorial", "Visualizing going up from the object (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan_up;
    //trajectory_up.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); //was trajectory
    plan_up.trajectory_ = trajectory_up;
    sleep(1.0);
    // Visualize the plan in RViz
    visual_tools.trigger();
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group.execute(plan_up);

        // check if execution is correct!
    success = (move_group.plan(plan_up) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      ROS_INFO_NAMED("tutorial", "Plan suceeded");
    } else {
      ROS_INFO_NAMED("tutorial", "Plan FAILED");
      return 1;
    }


    while (holes_absolute_position_vec[k].y < 0) { // add error if there is no hole that exists
      k++;
    }

    target_pose_pickplace.position.y = holes_absolute_position_vec[k].y;
    waypoints_place.push_back(target_pose_pickplace);
    target_pose_pickplace.position.x = holes_absolute_position_vec[k].x;
    waypoints_place.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.z = 0.23;
    waypoints_place.push_back(target_pose_pickplace);

    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory_2;
    move_group.setPlanningTime(3.0);
    

    fraction = move_group.computeCartesianPath(waypoints_place, eef_step, jump_threshold, trajectory_2, test_constraints);
    ROS_INFO_NAMED("tutorial", "Visualizing finding a hole (%.2f%% acheived)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan_2;
    trajectory_2.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    plan_2.trajectory_ = trajectory_2;
    sleep(1.0);

    visual_tools.trigger();
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group.execute(plan_2);

    // check if execution is correct!
    success = (move_group.plan(plan_2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      ROS_INFO_NAMED("tutorial", "Plan suceeded");
    } else {
      ROS_INFO_NAMED("tutorial", "Plan FAILED");
      return 1;
    }

    std::cout << "numobj: " << num_objects << " j: " << j << " holesabsvecsize: " << holes_absolute_position_vec.size() << " objabsvecsize: " << object_absolute_position_vec.size() << "\n";
    
    target_pose_pickplace.position.z = 0.171;
    waypoints_down_hole.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.y = holes_absolute_position_vec[k].y; 
    waypoints_down_hole.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.x = holes_absolute_position_vec[k].x;
    waypoints_down_hole.push_back(target_pose_pickplace); 
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory_down_hole;
    move_group.setPlanningTime(3.0);
    trajectory_down_hole.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    fraction = move_group.computeCartesianPath(waypoints_down_hole, eef_step_pickplace, jump_threshold, trajectory_down_hole, test_constraints);

    ROS_INFO_NAMED("tutorial", "Visualizing placing the object (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan_down_hole;
    //trajectory_down_hole.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    plan_down_hole.trajectory_ = trajectory_down_hole;
    sleep(1.0);
    // Visualize the plan in RViz
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group.execute(plan_down_hole);
    
        // check if execution is correct!
    success = (move_group.plan(plan_down_hole) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      ROS_INFO_NAMED("tutorial", "Plan suceeded");
    } else {
      ROS_INFO_NAMED("tutorial", "Plan FAILED");
      return 1;
    }

    sleep(1.0);

    srv.request.io_idx = io_idx_open;

    if (client.call(srv))
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
    }

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

      /*if (objcoord_vec.size() != 0 && table_found_var == true) 
      {
        object_absolute_position_vec.clear();
        for (int i = 0; i < num_objects; i++)
        {
          object_absolute_position.x = camera_x+inv_sc_proj_mat[1][1]*(objcoord_vec[i].y-optical_center_y);
          object_absolute_position.y = camera_y+inv_sc_proj_mat[0][0]*(objcoord_vec[i].x-optical_center_x);
          object_absolute_position_vec.push_back(object_absolute_position);
        }
        j = 0;
        while (object_absolute_position_vec[j].y > 0) {
          j++;
        }
        if (j >= object_absolute_position_vec.size())
        {
          current_state = move_group.getCurrentState();
          ROS_INFO_NAMED("tutorial", "All objects have positive y coordinate. Aborting.");
          current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_home);
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
          break;
        }
      }
        
      if (holescoord_vec.size() != 0 && table_found_var == true) 
      {
        holes_absolute_position_vec.clear();
        for (int i = 0; i < num_holes; i++)
        {
          holes_absolute_position.x = camera_x+(holescoord_vec[i].y-optical_center_y)/pixel_to_m_table;
          holes_absolute_position.y = camera_y+(holescoord_vec[i].x-optical_center_x)/pixel_to_m_table + 0.002;
          holes_absolute_position_vec.push_back(holes_absolute_position);
        }
      }

      // correction of coordinates
      holes_absolute_position_vec_corrected.clear();
      float holes_absolute_position_vec_coordinates_x[] = {0.326, 0.3536, 0.3800, 0.3800, 0.4060, 0.4060, 0.4335, 0.4335, 0.4655, 0.4860};
      float holes_absolute_position_vec_coordinates_y[] = {0.0714, 0.1191, 0.0714, 0.164, 0.1191, 0.2115, 0.0714, 0.164, 0.1191, 0.0714};
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
      }*/

    }
    // check if all objects are on + coordinate
    // if not repeat the program

  }
  //////////////////////////////////////////////
  
  ros::shutdown();
  return 0;
}
