#include "ros/ros.h"
#include "set_io/set_io.h"
#include <curl/curl.h>
#include <string>
#include <vector>
#include <sstream>
#include <utility>
#include <typeinfo>

using namespace std;
string robot_ip, final_url, io_op, io_type, io_idx, io_val;
string response_info;

std::vector<std::string> explode(std::string const & s, char delim)
{
    std::vector<std::string> result;
    std::istringstream iss(s);

    for (std::string token; std::getline(iss, token, delim); )
    {
        result.push_back(std::move(token));
    }

    return result;
}

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

bool set_io_service(set_io::set_io::Request  &req,
         set_io::set_io::Response &res)
{
  robot_ip = req.robot_ip;
  io_op = req.io_op;
  io_type = req.io_type;
  io_idx = req.io_idx;
  io_val = req.io_val;
  final_url = "http://" + robot_ip + "io_op=" + io_op + "&io_type=" + io_type + "&io_idx=" + io_idx + "&io_val=" + io_val;
  CURL *curl;
  CURLcode result;
  string readBuffer;

  curl = curl_easy_init();
  if(curl) 
  {
    curl_easy_setopt(curl, CURLOPT_URL, final_url.c_str()); // open/close operation
    //curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.1.50/KAREL/ros_cgio?io_op=write&io_type=9&io_idx=8&io_val=1"); // CLOSE gripper
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    result = curl_easy_perform(curl);
      // Check for errors 
    if(result != CURLE_OK)
    {
        ROS_INFO("curl_easy_perform() failed: %s\n", curl_easy_strerror(result));
    }
    //ROS_INFO("Url to call: %s", final_url.c_str());
    auto v = explode(readBuffer, '"');

    if (v[3] == "success")
    {
        res.success = "1";
        response_info = "Succeeded to put the desired input/output values.";
    } else {
        res.success = "0";
        response_info = "Failed to put the desired input/output values.";
    }
      // always cleanup 
    curl_easy_cleanup(curl);
  }
  
  ROS_INFO("%s [io_op=%s, io_type=%s, io_idx=%s, io_val=%s]", 
  response_info.c_str(), req.io_op.c_str(), req.io_type.c_str(), req.io_idx.c_str(), req.io_val.c_str());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_io_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("set_io", set_io_service);
  ROS_INFO("Ready to control inputs/outputs.");
  ros::spin();

  return 0;
}