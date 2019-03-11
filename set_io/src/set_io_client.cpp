#include "ros/ros.h"
#include "set_io/set_io.h"
#include <string>


std::string responsestring;
std::string successstring = "1";
int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_io_client");
  if (argc != 6)
  {
    ROS_INFO("usage: robot_ip io_operation[read/write] io_type[port on controller - RDO,...] idx[port index] value[0/1]");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<set_io::set_io>("set_io");
  set_io::set_io srv;
  srv.request.robot_ip = argv[1];
  srv.request.io_op = argv[2];
  srv.request.io_type = argv[3];
  srv.request.io_idx = argv[4];
  srv.request.io_val = argv[5];
      
  if (client.call(srv))
  {
    ROS_INFO("Response: %s", srv.response.success.c_str());
    responsestring = srv.response.success.c_str();
    if (&responsestring[0] != successstring)
    {
      std::cout << "not1";
      std::cout << &responsestring[0];
    } else {
      std::cout << "suceed";
    }
  }
  else
  {
    ROS_ERROR("Failed to call service set_io");
    return 1;
  }

  return 0;
}