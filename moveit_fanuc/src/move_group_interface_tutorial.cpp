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
#include <curl/curl.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

float table_x;
float table_y;
std::vector<geometry_msgs::Point> objcoord_vec;
std::vector<geometry_msgs::Point> holescoord_vec;
std::vector<geometry_msgs::Point>object_absolute_position_vec;
std::vector<geometry_msgs::Point>holes_absolute_position_vec;
geometry_msgs::Point object_absolute_position;
geometry_msgs::Point holes_absolute_position;
bool table_found_var = false;
bool flag_firsttime = 0;
int num_objects = 0;
int num_holes = 0;
int numobj_belowzero = 0;
int numobj_abovezero = 0;
// Initialize working table width and height constants, if table is found program corrects table size in table_properties_clbk
float pixel_to_m_table = 833; // initialize pixel to m ratio, if table is not found. 839 px = 1m
int pixel_to_m_object = 867; // pixel to m ratio on height of objects
float working_table_w = 418;
float working_table_h = 250;
int optical_center_x = 227;
int optical_center_y = 209;
const double jump_threshold = 0.0;
const double eef_step = 0.01;
const double eef_step_pickplace = 0.002;
double fraction = 0;

void table_found_clbk(const std_msgs::Bool::ConstPtr& found) {
    table_found_var = found->data;
}

void table_properties_clbk(const camera_to_cv::table_properties::ConstPtr& prop) {
    working_table_w = prop->width;
    working_table_h = prop->height;
    pixel_to_m_table = working_table_w/0.498;
}

void objects_centers_clbk(const camera_to_cv::points_array::ConstPtr& msg) {
    //ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
    // clear vector of objects
    objcoord_vec.clear();
    for (int i = 0; i < (msg->points.size()); i++) 
    {
      //std::cout << typeid(msg->points[i].x).name() << " " << typeid(objcoord_vec_x).name() << '\n';
      // add objects center points to vector
      objcoord_vec.push_back(msg->points[i]);
      num_objects = msg->points.size();
      //std::cout << objcoord_vec_x[0] << ", size: " << objcoord_vec_x.size() << "\n";

    }
}

void holes_centers_clbk(const camera_to_cv::points_array::ConstPtr& msg) {
    //ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
    // clear vector of objects
    holescoord_vec.clear();
    for (int i = 0; i < (msg->points.size()); i++) 
    {
      //std::cout << typeid(msg->points[i].x).name() << " " << typeid(holescoord_vec_x).name() << '\n';
      // add objects center points to vector
      holescoord_vec.push_back(msg->points[i]);
      num_holes = msg->points.size();
      //std::cout << holescoord_vec_x[0] << ", size: " << holescoord_vec_x.size() << "\n";

    }
}

/*std::vector<double> obj_belowzero(int n_obj, std::vector<double > obj_abspos_xvec) {
    std::vector<double> obj_belowzero_coor;
    for (int i = 0; i < n_obj; i++)
    {
      if (obj_abspos_yvec[i] < 0)
      {
        obj_belowzero_coor.push_back(obj_abspos_yvec[i]); 
      }
    }
    return obj_belowzero_coor;
}

std::vector<double> holes_abovezero(int n_obj, std::vector<double > holes_abspos_yvec) {
    std::vector<double> holes_abspos_coor;
    for (int i = 0; i < n_obj; i++)
    {
      if (holes_abspos_yvec[i] < 0)
      {
        obj_belowzero_coor.push_back(obj_abspos_yvec[i]); 
      }
    }
    return obj_belowzero_coor;
}*/



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

  if (objcoord_vec.size() != 0 && table_found_var == true && flag_firsttime == 0) 
  {
    object_absolute_position_vec.clear();
    for (int i = 0; i < num_objects; i++)
    {
      object_absolute_position.x = table_x - ((working_table_h - optical_center_y)/pixel_to_m_table) - (optical_center_y-objcoord_vec[i].y)/pixel_to_m_object;
      object_absolute_position.y = table_y - ((working_table_w - optical_center_x)/pixel_to_m_table) - (optical_center_x-objcoord_vec[i].x)/pixel_to_m_object;
      object_absolute_position_vec.push_back(object_absolute_position);
      //std::cout << "Object[" << i << "]  x: " << object_absolute_position.x << ", y: " << object_absolute_position.y << "\n";
      //object_absolute_position_x_vec.push_back(table_x - ((working_table_h - objcoord_vec.y[i])/pixel_to_m_table) + ); //
      //object_absolute_position_y_vec.push_back(table_y - ((working_table_w - objcoord_vec.x[i])/pixel_to_m_table) + ); //
    }
  }

  if (holescoord_vec.size() != 0 && table_found_var == true && flag_firsttime == 0) 
  {
    holes_absolute_position_vec.clear();
    for (int i = 0; i < num_holes; i++)
    {
      holes_absolute_position.x = table_x - ((working_table_h - optical_center_y)/pixel_to_m_table) - (optical_center_y-holescoord_vec[i].y)/pixel_to_m_table;
      holes_absolute_position.y = table_y - ((working_table_w - optical_center_x)/pixel_to_m_table) - (optical_center_x-holescoord_vec[i].x)/pixel_to_m_table; // if y >opt centery: - 0.0215 * (optical_center_x-holescoord_vec[i].x)/pixel_to_m_table
      if (holescoord_vec[i].x > optical_center_x) // shadow correction
      {
        if (holes_absolute_position.y > 0.2)
        {
          holes_absolute_position.y += - 0.017 * (optical_center_x-holescoord_vec[i].x)/pixel_to_m_table;
        } else {
          holes_absolute_position.y += - 0.05 * (optical_center_x-holescoord_vec[i].x)/pixel_to_m_table;
        }
      }
      holes_absolute_position_vec.push_back(holes_absolute_position);
      //std::cout << "Hole[" << i << "]  x: " << holes_absolute_position.x << ", y: " << holes_absolute_position.y << "\n";
      //object_absolute_position_x_vec.push_back(table_x - ((working_table_h - objcoord_vec.y[i])/pixel_to_m_table) + ); //
      //object_absolute_position_y_vec.push_back(table_y - ((working_table_w - objcoord_vec.x[i])/pixel_to_m_table) + ); //
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
  ROS_INFO_NAMED("tutorial", "Visualizing robot going to start position", success ? "" : "FAILED");

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
      j++;
    }

    std::cout << "OBJ x " << object_absolute_position_vec[j].x << "y " << object_absolute_position_vec[j].y;

    target_pose_pickplace.position.z = 0.22;
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

    target_pose_pickplace.position.z = 0.155;
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
    move_group.execute(plan_down);

    CURL *curl;
    CURLcode res;

    curl = curl_easy_init();
    if(curl) 
    {
      curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.1.50/KAREL/ros_cgio?io_op=write&io_type=9&io_idx=8&io_val=1"); // CLOSE gripper
        // Perform the request, res will get the return code  
      res = curl_easy_perform(curl);
      // Check for errors 
      if(res != CURLE_OK)
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));
  
      // always cleanup 
      curl_easy_cleanup(curl);
    }

    target_pose_pickplace.position.z = 0.22;
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
    move_group.execute(plan_up);

    while (holes_absolute_position_vec[k].y < 0) { // add error if there is no hole that exists
      k++;
    }

    target_pose_pickplace.position.y = holes_absolute_position_vec[k].y;
    waypoints_place.push_back(target_pose_pickplace);
    target_pose_pickplace.position.x = holes_absolute_position_vec[k].x;
    waypoints_place.push_back(target_pose_pickplace); 
    target_pose_pickplace.position.z = 0.22;
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

    /*visual_tools.trigger();
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");*/

    move_group.execute(plan_2);
    std::cout << "k: " << k << " j: " << j << " holesabsvecsize: " << holes_absolute_position_vec.size() << " objabsvecsize: " << object_absolute_position_vec.size() << "\n";
    
    target_pose_pickplace.position.z = 0.168;
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
    /*visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");*/
    move_group.execute(plan_down_hole);
    
    
    CURL *curl_2;
    CURLcode res_2;

    curl_2 = curl_easy_init();
    if(curl_2) 
    {
      curl_easy_setopt(curl_2, CURLOPT_URL, "http://192.168.1.50/KAREL/ros_cgio?io_op=write&io_type=9&io_idx=7&io_val=1"); // OPEN gripper
      // Perform the request, res will get the return code  
      res_2 = curl_easy_perform(curl_2);
      // Check for errors 
      if(res != CURLE_OK)
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));
  
      // always cleanup 
      curl_easy_cleanup(curl_2);
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
      ROS_INFO_NAMED("tutorial", "Visualizing robot going to start position", success ? "" : "FAILED");

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
  //////////////////////////////////////////////
  
  ros::shutdown();
  return 0;
}
