#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <camera_to_cv/points_array.h>
#include <camera_to_cv/table_properties.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>

float camera_x;
float camera_y;
float table_x;
float table_y;
// Initialize working table width and height constants, if table is found program corrects table size in table_properties_clbk
float working_table_w = 418;
float working_table_h = 250;
// todo: change constants constants of camera im. size to published camera im. size 
float camera_w = 640;
float camera_h = 480;
float pixel_to_m = 839; // initialize pixel to m ratio, if table is not found. 839 px = 1m
int num_old_objects = 0;

std::vector<double >points_vec_x;
std::vector<double >points_vec_y;
bool table_found_var = false;
int num_objects = 0;

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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Publisher table_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_table", 1);
  ros::Subscriber sub = n.subscribe("object_chatter", 1000, objects_centers_clbk);
  ros::Subscriber sub_table_found = n.subscribe ("table_found", 1000, table_found_clbk); 
  ros::Subscriber sub_table_properties = n.subscribe ("table_properties_chatter", 1000, table_properties_clbk);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  while (ros::ok())
  {
    visualization_msgs::MarkerArray marker_array;
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", "camera_frame",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    camera_x = transformStamped.transform.translation.x;
    camera_y = transformStamped.transform.translation.y;

    geometry_msgs::TransformStamped transformStamped_table;
    try{
      transformStamped_table = tfBuffer.lookupTransform("base_link", "working_table_fr",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    table_x = transformStamped_table.transform.translation.x;
    table_y = transformStamped_table.transform.translation.y;

    /*if (table_found_var == true) {
      marker.action = visualization_msgs::Marker::DELETEALL;
      marker_array.markers.push_back(marker);
    }*/

    

    if (points_vec_x.size() != 0 && table_found_var == true) 
    {
      for (int j = 0; j < num_old_objects; j++) // delete markers from previous iteration
      {
        visualization_msgs::Marker marker;
        marker.ns = "object_shape";
        marker.id = j;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
      }
      num_old_objects = num_objects;
      for (int i = 0; i < num_objects; i++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "object_shape";
        marker.id = i;
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)

        marker.action = visualization_msgs::Marker::ADD;
        // če je miza najdena je referenca spodnji desni rob mize
        // če ni najdena je referenca center kamere

          /*if (table_found_var == true) {*/
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = table_x - ((working_table_h - points_vec_y[i])/pixel_to_m); //
            marker.pose.position.y = table_y - ((working_table_w - points_vec_x[i])/pixel_to_m); //
          /*} else {
            marker.pose.position.x = camera_x - ((camera_h/2 - points_vec_y[i])/pixel_to_m);
            marker.pose.position.y = camera_y - ((camera_w/2 - points_vec_x[i])/pixel_to_m);
          }*/
        

        marker.pose.position.z = 0.02;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the size of the marker
        marker.scale.x = 0.025;
        marker.scale.y = 0.025;
        marker.scale.z = 0.040;

        // Set the color to white
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);
      }
    }
    visualization_msgs::Marker table_marker;
    table_marker.header.frame_id = "/base_link";
    table_marker.header.stamp = ros::Time::now();
    table_marker.ns = "working_table";
    table_marker.id = 0;
    table_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    table_marker.mesh_resource = "package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/working_table.stl";
    table_marker.action = visualization_msgs::Marker::ADD;
    tf2::Quaternion q;
        q.setRPY(0, 3.14159, 1.5708);
    table_marker.pose.position.x = table_x;
    table_marker.pose.position.y = table_y;
    table_marker.pose.position.z = 0;
    table_marker.pose.orientation.x = q.x();
    table_marker.pose.orientation.y = q.y();
    table_marker.pose.orientation.z = q.z();
    table_marker.pose.orientation.w = q.w();
    
    // modeled in mm, convert to m
    table_marker.scale.x = 0.001;
    table_marker.scale.y = 0.001;
    table_marker.scale.z = 0.001;

    table_marker.color.r = 0.0f;
    table_marker.color.g = 0.0f;
    table_marker.color.b = 0.0f;
    table_marker.color.a = 1.0;
    table_marker.lifetime = ros::Duration();
    marker_array.markers.push_back(table_marker);
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker_array);
    //table_pub.publish(table_marker);
    ros::spinOnce();
    //r.sleep();

    }
   
  }