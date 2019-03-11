#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <camera_to_cv/points_array.h>
#include <camera_to_cv/table_properties.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

using namespace cv;
using namespace std;

// Constants for image processing
int n_objects = 0;
int sigma = 1.2;
int ksize = 5;
int erosion_size = 2; 
double min_contour_area = 220;
double max_contour_area = 400;
float center_proximity = 7.0;
double contour_area = 0;
double holes_threshold = 100;
double object_threshold = 172;
int max_rectangle_index = 0;
bool working_table_limits_found = 0;
int n_expected_objects = 100;
int surrounding_rectangle_size = 40;
int sum_current_rectangle = 0;
int avg_current_rectangle = 0;
static const std::string OPENCV_WINDOW = "Color image";
Rect working_table, bounding_patch;
Mat im_working_table_gray, image, im_orig, im_gray, im_bw_canny, im_gray_edges, im_bw_holes_, im_blue_channel;
Mat channel[3];
vector<Mat> im_objectholepatch;

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

class ImageConverter
{
  // Image transport class and node handle declaration
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  // Publishers and subscribers declaration
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher object_pub = nh_.advertise<camera_to_cv::points_array>("object_chatter", 1000);
  ros::Publisher holes_pub = nh_.advertise<camera_to_cv::points_array>("holes_chatter", 1000);
  ros::Publisher table_properties_pub = nh_.advertise<camera_to_cv::table_properties>("table_properties_chatter", 1000);
  ros::Publisher table_found_pub = nh_.advertise<std_msgs::Bool>("table_found", 1000);

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
///////// Begin image processing /////////
    image = cv_ptr->image;
    split(image, channel);
    im_blue_channel = channel[0];
    cvtColor(image, im_gray, COLOR_BGR2GRAY);
	  Canny(im_gray, im_bw_canny, 100, 300, 5); // Canny detector is used for table finding
    im_gray = im_blue_channel; // Gray image out of blue channel (to improve contrast)
    GaussianBlur(im_gray, im_gray, Size(ksize, ksize), sigma, sigma);
    
  ///////// Getting working table limits
    // shape detection based on https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
    // Each time the table is found publish a message containing objects centers locations
    vector< vector <Point> > contours_table;
    vector<Point> approx;
    double largest_rectangle_area = 0;

    // Create a table found message and table properties message
    std_msgs::Bool table_found_msg;
    camera_to_cv::table_properties table_properties_msg;

    if (working_table_limits_found == 0)
    {
      // Find rectangles in the image
      findContours(im_bw_canny, contours_table, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 
      for (int i = 0; i < contours_table.size(); i ++)                         
      { 
        approxPolyDP(Mat(contours_table[i]), approx, arcLength(Mat(contours_table[i]), true)*0.02, true);

        if (fabs(contourArea(contours_table[i])) < 100 || !isContourConvex(approx))
              continue;

        if (approx.size() == 4)
        {
          // Number of vertices of polygonal curve 
          int vtc = approx.size();

          // Get the cosines of all corners
          vector<double> cos;
          for (int j = 2; j < vtc+1; j++)
            cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

          // Sort ascending the cosine values
          sort(cos.begin(), cos.end());

          // Get the lowest and the highest cosine
          double mincos = cos.front();
          double maxcos = cos.back();

          // Use the degrees obtained above to check if it is a rectangle
          if (mincos >= -0.1 && maxcos <= 0.3)
          {
            // Find the largest rectangle
            Rect r = boundingRect(contours_table[i]);
            double rectangle_area = r.height*r.width; 
            if (rectangle_area>largest_rectangle_area)
            {
              largest_rectangle_area = rectangle_area;
              max_rectangle_index = i;
              working_table=boundingRect(contours_table[i]);
            }
          }
        }
      }

      // If the working table is found make a new image with just the working table
      
      int table_limit_area = (im_gray.cols/2)*(im_gray.rows/3);
      if (largest_rectangle_area > table_limit_area) { 
        working_table = Rect(0,0,im_gray.cols,im_gray.rows) & working_table;
        im_working_table_gray = im_gray(working_table).clone();
        working_table_limits_found = 1;
        cout << "Working table found \n";
      }
    }
  ///////// End working table limits

    // Fill the table properties message
    if (working_table_limits_found == 1)
    {
      table_properties_msg.width = im_working_table_gray.cols;
      table_properties_msg.height = im_working_table_gray.rows;
    } else {
      table_properties_msg.width = 0;
      table_properties_msg.height = 0;
    }

    table_found_msg.data = working_table_limits_found;

    // If the working table is found work with coordinates of working table, otherwise whole image
    Mat im_gray_, im_bw_objects;
    if (working_table_limits_found == 1)
    {
      im_gray_ = im_gray(working_table).clone();
      image = image(working_table).clone();
    } else {
      im_gray_ = im_gray;
    }
    imshow("Color without label", image);
    

  //////// Find the holes centers
    threshold(im_gray_, im_bw_holes_, holes_threshold, 255.0, THRESH_OTSU);
    // Opening - erosion followed by dilation for noise (glare) removal
    Mat element = getStructuringElement( MORPH_ELLIPSE, 
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ));
    
    erode( im_bw_holes_, im_bw_holes_, element );
    dilate( im_bw_holes_, im_bw_holes_, element );

    vector< vector <Point> > contours_holes;
    vector<Point> approx_holes;

    findContours(im_bw_holes_, contours_holes, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE); // Find the circles in the image based on Canny edge d.
    vector<Point2f>center_holes;
    vector<float>radius_vect_holes;
    vector<Point2f>true_object_center;
    vector<int>surrounding_avg_brightness_level(contours_holes.size());
    center_holes.reserve(contours_holes.size());
    radius_vect_holes.reserve(contours_holes.size());
    true_object_center.reserve(n_expected_objects);
    vector <bool> objects_flag (contours_holes.size());

      for (int i = 0; i < contours_holes.size(); i ++)
      { 
        approxPolyDP(Mat(contours_holes[i]), approx_holes, arcLength(Mat(contours_holes[i]), true)*0.02, true);

        if (fabs(contourArea(contours_holes[i])) < 100 || !isContourConvex(approx_holes))
              continue;

        if (approx_holes.size() > 6) // Detects just circles
        {
          // Detect and label circles
          double area = contourArea(contours_holes[i]);
          Rect r = boundingRect(contours_holes[i]);
          int radius = r.width / 2;

          if (abs(1 - ((double)r.width / r.height)) <= 0.3 &&
              abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.3)
            {
              minEnclosingCircle( (Mat)contours_holes[i], center_holes[i], radius_vect_holes[i] );

              // Begin getting objects/holes surrounding brightness
              if (center_holes[i].x > 20 && center_holes[i].x < (im_gray_.cols-20) 
              && center_holes[i].y > 20 && center_holes[i].y < (im_gray_.rows-20))
              {
                Rect r(int(center_holes[i].x)-20,int(center_holes[i].y)-20,40,40);
                bounding_patch = Rect(0,0,im_gray_.cols,im_gray_.rows) & r;
                im_objectholepatch.push_back(im_gray_(bounding_patch).clone());
                sum_current_rectangle = 0;
                for (int k = 0; k < 2; k++)
                {
                  for(int m = 0; m < surrounding_rectangle_size; m++)
                  {
                    if (k == 0)
                    {
                      sum_current_rectangle += int(im_gray_.at<unsigned char>( (center_holes[i].y-surrounding_rectangle_size/2),
                      (center_holes[i].x+(surrounding_rectangle_size/2 - m)) ) );
                      if (m < (surrounding_rectangle_size-2))
                      {
                        sum_current_rectangle += int(im_gray_.at<unsigned char>( (center_holes[i].y+(surrounding_rectangle_size/2 - m - 1)),
                        (center_holes[i].x-surrounding_rectangle_size/2) ) );

                      }
                    } else if (k == 1)
                    {
                      sum_current_rectangle += int(im_gray_.at<unsigned char>( (center_holes[i].y+surrounding_rectangle_size/2),
                      (center_holes[i].x+(surrounding_rectangle_size/2 - m)) ) );
                      if (m < (surrounding_rectangle_size-2))
                      {
                        sum_current_rectangle += int(im_gray_.at<unsigned char>( (center_holes[i].y+(surrounding_rectangle_size/2 - m - 1)),
                        (center_holes[i].x+surrounding_rectangle_size/2) ) );
                      }
                    }

                  }
                }

              avg_current_rectangle = sum_current_rectangle / (surrounding_rectangle_size*4-4);
              surrounding_avg_brightness_level[i] = avg_current_rectangle;
              cout << "Avg bound rect: " << avg_current_rectangle << "\n";

              }
              // End getting objects/holes surrounding brightness

              /*for (int j = 0; j < center_objects.size(); j++)
              {
                if (center_holes[i].x < (center_objects[j].x + center_proximity) && 
                center_holes[i].x > (center_objects[j].x - center_proximity) && 
                center_holes[i].y < (center_objects[j].y + center_proximity) && 
                center_holes[i].y > (center_objects[j].y - center_proximity))
                {
                  true_object_center.push_back(center_objects[j]);
                  // flag i -> do not draw the circle
                  objects_flag[i] = 1;
                }
              }*/
            }
        }

          // draw red circle if it is a hole
          if (radius_vect_holes[i] > 0 && radius_vect_holes[i] < 30)
          {
            //circle( image, center_holes[i], (int)radius_vect_holes[i], Scalar ( 0,0,255), 2, 8, 0 );
            center_holes.push_back(center_holes[i]); // Fill the vector
            radius_vect_holes.push_back(radius_vect_holes[i]);
          }
        


      }
  //////// End of the holes centers

      
  //////// Begin object detection with adaptive threshold based on the surroundings of object/hole
    Scalar mean;
    Scalar stdev;
    meanStdDev(surrounding_avg_brightness_level,mean,stdev);
    cout << "mean" << mean[0] << "STD" << stdev[0];

    // Construct a vector with  objects/holes image patches
    vector <float> object_threshold_vec(im_objectholepatch.size());
    vector<Mat> im_objectholepatch_bin(im_objectholepatch.size());

    vector <vector <Point> > contours_objects;
    vector<Point> approx_objects;

    for (int i = 0; i < im_objectholepatch.size(); i++)
    {
      object_threshold_vec[i] = object_threshold;
      //object_threshold_vec[i] = object_threshold - (mean[0] - surrounding_avg_brightness_level[i])/2;
      cout << "OBJ THR " << object_threshold_vec[i] << "surr avg br" << surrounding_avg_brightness_level[i] << "\n";
      threshold(im_objectholepatch[i], im_objectholepatch_bin[i], object_threshold_vec[i], 255.0, THRESH_BINARY);
      findContours(im_objectholepatch_bin[i], contours_objects, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image based on threshold
      for (int j = 0; j < contours_objects.size(); j++)
      {
        approxPolyDP(Mat(contours_objects[j]), approx_objects, arcLength(Mat(contours_objects[j]), true)*0.02, true);
        if (fabs(contourArea(contours_objects[j])) < 100 || !isContourConvex(approx_objects))
                continue;
        if (approx_objects.size() > 6) // Detects just circles
        {
          double area_objects = contourArea(contours_objects[j]);
          Rect r_objects = boundingRect(contours_objects[j]);
          int radius_objects = r_objects.width / 2;
          
          if (abs(1 - ((double)r_objects.width / r_objects.height)) <= 0.3 &&
              abs(1 - (area_objects / (CV_PI * pow(radius_objects, 2)))) <= 0.3
              //&& area_objects > min_contour_area && area_objects < max_contour_area
              )
              {
                  objects_flag[i] = 1;
              }
        }
      }

    }
    surrounding_avg_brightness_level.clear();
  //////// End object detection

  //////// Publish holes and objects center coordinates, output coordinates on console, and draw a circle
      camera_to_cv::points_array points_msg;
      camera_to_cv::points_array holes_points_msg;

      int o = 0;
      int h = 0;
      for (int i = 0; i < center_holes.size(); i++)
      {
        if (objects_flag[i] != 1) {
          cout << "Hole [" << h << "] coordinates -> x: " << center_holes[i].x << ", y: " << center_holes[i].y << "\n";
          geometry_msgs::Point point_hole;
          point_hole.x = center_holes[i].x;
          point_hole.y = center_holes[i].y;
          point_hole.z = 0;
          holes_points_msg.points.push_back(point_hole);
          circle( image, center_holes[i], (int)radius_vect_holes[i], Scalar ( 0,0,255), 2, 8, 0 ); // draw red circle around the contour
          h++;
        } 
      }
      for (int i = 0; i < center_holes.size(); i++)
      {
        if (objects_flag[i] != 1) {
          cout << "Object [" << o << "] coordinates -> x: " << center_holes[i].x << ", y: " << center_holes[i].y << "\n";
          geometry_msgs::Point point;
          point.x = center_holes[i].x;
          point.y = center_holes[i].y;
          point.z = 0;
          points_msg.points.push_back(point);
          o++;
          circle( image, center_holes[i], (int)radius_vect_holes[i], Scalar ( 0,255,0), 2, 8, 0 ); // draw green circle around the contour
        }
      }

  //////// End Publish and output
    

      //imshow("Color image", cv_ptr->image);
      imshow(OPENCV_WINDOW, image);

      //imshow(OPENCV_WINDOW, im_gray_);
      imshow("Holes thresholding", im_bw_holes_);
      ///////// End image processing /////////
    waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    if (working_table_limits_found == 1) {
      object_pub.publish(points_msg);
      holes_pub.publish(holes_points_msg);
    }
    working_table_limits_found = 0;
    table_found_pub.publish(table_found_msg);
    table_properties_pub.publish(table_properties_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


