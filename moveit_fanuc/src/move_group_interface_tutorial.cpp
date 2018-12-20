#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <camera_to_cv/points_array.h>
#include <camera_to_cv/table_properties.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>


float table_x;
float table_y;
std::vector<double >points_vec_x;
std::vector<double >points_vec_y;
bool table_found_var = false;
int num_objects = 0;
// Initialize working table width and height constants, if table is found program corrects table size in table_properties_clbk
float pixel_to_m = 839; // initialize pixel to m ratio, if table is not found. 839 px = 1m
float working_table_w = 418;
float working_table_h = 250;
std::vector<double >object_absolute_position_x;
std::vector<double >object_absolute_position_y;

void objects_centers_clbk(const camera_to_cv::points_array::ConstPtr& msg) {
    //ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
    // clear vector of objects
    points_vec_x.clear();
    points_vec_y.clear();
    for (int i = 0; i < (msg->points.size()); i++) 
    {
      //std::cout << typeid(msg->points[i].x).name() << " " << typeid(points_vec_x).name() << '\n';
      // add objects center points to vector
      points_vec_x.push_back(msg->points[i].x);
      points_vec_y.push_back(msg->points[i].y);
      num_objects = msg->points.size();
      //std::cout << points_vec_x[0] << ", size: " << points_vec_x.size() << "\n";

    }
}

void table_found_clbk(const std_msgs::Bool::ConstPtr& found) {
    table_found_var = found->data;
}

void table_properties_clbk(const camera_to_cv::table_properties::ConstPtr& prop) {
    working_table_w = prop->width;
    working_table_h = prop->height;
    pixel_to_m = working_table_w/0.498;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = node_handle.subscribe("object_chatter", 1000, objects_centers_clbk);
  ros::Subscriber sub_table_found = node_handle.subscribe ("table_found", 1000, table_found_clbk);
  ros::Subscriber sub_table_properties = node_handle.subscribe ("table_properties_chatter", 1000, table_properties_clbk);
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

  if (points_vec_x.size() != 0 && table_found_var == true) 
  {
    object_absolute_position_x.clear();
    object_absolute_position_y.clear();
    for (int i = 0; i < num_objects; i++)
    {
      object_absolute_position_x.push_back(table_x - ((working_table_h - points_vec_y[i])/pixel_to_m)); //
      object_absolute_position_y.push_back(table_y - ((working_table_w - points_vec_x[i])/pixel_to_m)); //
    }
  }
  std::cout << "obj0 x: " << object_absolute_position_x[0] << ", y: " << object_absolute_position_y[0];


  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Modify the fifth joint to get the manipulator perpendicular to the working table
  joint_group_positions[4] = -1.5708;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // replace with tool frame
  // quaternion goal -> 90 degrees around y-axis, EE parallel to the ground normal
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


  // Cartesian Paths
  // ^^^^^^^^^^^^^^^

  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;

  target_pose3.position.y = object_absolute_position_y[0]; 
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.x = object_absolute_position_x[0];
  waypoints.push_back(target_pose3);  // up and left

  target_pose3.position.z = 0.20;
  waypoints.push_back(target_pose3);  // down

  //move_group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  //trajectory.header.stamp = ros::Time::now();
  move_group.setPlanningTime(10.0);
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, test_constraints);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  sleep(3.0);
  

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  
  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group.execute(plan);



  std::vector<geometry_msgs::Pose> waypoints_2;

  target_pose3.position.y = object_absolute_position_y[1];
  waypoints_2.push_back(target_pose3);  // right

  target_pose3.position.x = object_absolute_position_x[1];
  waypoints_2.push_back(target_pose3);  // up and left

  target_pose3.position.z = 0.20;
  waypoints_2.push_back(target_pose3);  // down

  //move_group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory_2;
  move_group.setPlanningTime(10.0);

  fraction = move_group.computeCartesianPath(waypoints_2, eef_step, jump_threshold, trajectory_2, test_constraints);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan_2;
  plan_2.trajectory_ = trajectory_2;
  sleep(3.0);

  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.execute(plan_2);



  std::vector<geometry_msgs::Pose> waypoints_3;

  target_pose3.position.y = 0.10;
  waypoints_3.push_back(target_pose3);  // right

  target_pose3.position.x = 0.4;
  waypoints_3.push_back(target_pose3);  // up and left

  target_pose3.position.z = 0.20;
  waypoints_3.push_back(target_pose3);  // down


  //move_group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory_3;
  move_group.setPlanningTime(10.0);

  fraction = move_group.computeCartesianPath(waypoints_3, eef_step, jump_threshold, trajectory_3, test_constraints);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan_3;
  plan_3.trajectory_ = trajectory_3;
  sleep(3.0);

  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.execute(plan_3);

  ros::shutdown();
  return 0;
}
