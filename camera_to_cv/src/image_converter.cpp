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
int sigma = 1.2;
int ksize = 3;
int erosion_size = 1; 
double min_contour_area = 220;
double max_contour_area = 400;
float center_proximity = 7.0;
double holes_threshold = 250;
double object_threshold = 160;
int threshold_invredobjects = 175;
int threshold_greenobjects = 170;
int max_rectangle_index = 0;
bool working_table_limits_found = 0;
int n_maxexp_objects = 1000;
static const string OPENCV_WINDOW = "Color image";
Rect working_table, bounding_patch;
Mat im_working_table_gray, image, im_orig, im_adjust, im_holesdetect, im_objectdetect, im_gray, im_bw_canny, im_gray_edges, im_bw_holes_, im_holes_splitedchannels[3];
Mat im_objects_redthr, im_objects_greenthr, im_objects_allchanels, im_objects_splitedchannels[3], im_bw_objects_fromred, im_bw_objects_fromgreen, im_bw_holes_fromred;
Mat channel[3];
Mat HSV, frame_threshold;

// Variables for image adjusting - to find the optimal values for current lightning
int brightness_level = 0;
int max_brightness = 3000;
int contrast_level = 0;
int max_contrast = 100;
int gamma_level = 1;
double gamma_ = 0;
static const std::string original_image_w = "Original image"; // pointer initializes original_image_w to the unnamed array's first element
static const std::string adjusted_image = "Adjusted image";
const String window_detection_name = "Object Detection_HSV";


// Final values of contrast, brightness and gamma for object/hole detection
double gamma_hole = 0;
int brightness_hole = -450;
int contrast_hole = 5;
double gamma_object = 0;
int brightness_object = -2065;
int contrast_object = 12;
const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

// Callback function for parameters contrast, brightness, gamma change
void ParametersChange(int, void*) 
{
  gamma_ = gamma_level/100.0;
  im_adjust = Mat::zeros( im_orig.size(), im_orig.type() );
  for( int y = 0; y < im_orig.rows; y++ ) {
        for( int x = 0; x < im_orig.cols; x++ ) {
            for( int c = 0; c < im_orig.channels(); c++ ) {
                im_adjust.at<Vec3b>(y,x)[c] =
                  saturate_cast<uchar>( pow(im_orig.at<Vec3b>(y,x)[c] / 255.0, gamma_) * 255.0 + contrast_level*im_orig.at<Vec3b>(y,x)[c] - brightness_level );
            }
        }
    }
  imshow(adjusted_image, im_adjust);
}

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

// Function for contour type
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

cv_bridge::CvImagePtr cv_ptr;
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  // Image transport class and node handle declaration
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  // Publishers and subscribers declaration
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &imageCb);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  ros::Publisher object_pub = nh_.advertise<camera_to_cv::points_array>("object_chatter", 1000);
  ros::Publisher holes_pub = nh_.advertise<camera_to_cv::points_array>("holes_chatter", 1000);
  ros::Publisher table_properties_pub = nh_.advertise<camera_to_cv::table_properties>("table_properties_chatter", 1000);
  ros::Publisher table_found_pub = nh_.advertise<std_msgs::Bool>("table_found", 1000);
  ros::Rate loop_rate(30);


  while(ros::ok())
  {
    if(cv_ptr)
    {
      image = cv_ptr->image;
      ///////////////////////////////////////////
      ///////// Begin image processing //////////
      ///////////////////////////////////////////
      im_orig = image;
      split(image, channel);
      cvtColor(image, im_gray, COLOR_BGR2GRAY);

      Canny(im_gray, im_bw_canny, 1000, 5000, 5); // Canny detector is used for table finding
      im_gray = channel[0]; // Gray image out of blue channel (to improve contrast)
      GaussianBlur( im_bw_canny, im_bw_canny, Size( 3, 3 ), 1.5, 1.5 );
      imshow("canny table detection", im_bw_canny);
      
      //////////////////////////////////////////////////
      ///////// Getting working table limits ///////////
      //////////////////////////////////////////////////
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

          if (fabs(contourArea(contours_table[i])) < 200 || !isContourConvex(approx))
                continue;

          if (approx.size() >= 4 && approx.size() <= 6)
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
            if (mincos >= -0.1 && maxcos <= 0.3 && vtc == 4)
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
      //////////////////////////////////////////////////
      ////////// End of working table limits ///////////
      //////////////////////////////////////////////////

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
        image = image.clone();
      }

      int histSize = 256;
      float range[] = { 0, 256 } ;
      const float* histRange = { range };
      bool uniform = true; bool accumulate = false;
      Mat b_hist, g_hist, r_hist;
      vector<Mat> bgr_planes;
      split( image, bgr_planes );
      /// Compute the histograms:
      calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
      calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
      calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
      int hist_w = 256; int hist_h = 400;
      int bin_w = cvRound( (double) hist_w/histSize );

      Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

      /// Normalize the result to [ 0, histImage.rows ]
      normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
      normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
      normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
      
      /*double maxVal=0;
      Point maxind;
      minMaxLoc(r_hist, 0, &maxVal, 0, &maxind);
      cout << "maxval:" << maxVal << "maxind " << maxind.y;*/

      /// Draw for each channel
      for( int i = 1; i < histSize; i++ )
      {
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                          Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                          Scalar( 255, 0, 0), 2, 8, 0  );
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                          Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                          Scalar( 0, 255, 0), 2, 8, 0  );
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                          Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                          Scalar( 0, 0, 255), 2, 8, 0  );
      }

      /// Display
      namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
      imshow("calcHist Demo", histImage );

      cvtColor(image, HSV, COLOR_BGR2HSV);
      // Detect the object based on HSV Range Values
      inRange(HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
      // Show the frames
      imshow(window_detection_name, frame_threshold);
      
      /*im_objectdetect = Mat::zeros( im_gray_.size(), im_gray_.type() );
      for( int y = 0; y < im_gray_.rows; y++ ) {
            for( int x = 0; x < im_gray_.cols; x++ ) {
              im_objectdetect.at<unsigned char>(y,x) = 
                saturate_cast<uchar>(pow(im_gray_.at<unsigned char>(y,x) / 255.0, gamma_object) * 255.0 + contrast_object*im_gray_.at<unsigned char>(y,x) + brightness_object );
            }
        }*/
      im_objects_allchanels = Mat::zeros( image.size(), image.type() );
      for( int y = 0; y < image.rows; y++ ) 
        {
            for( int x = 0; x < image.cols; x++ ) {
                for( int c = 0; c < image.channels(); c++ ) {
                    im_objects_allchanels.at<Vec3b>(y,x)[c] =
                      saturate_cast<uchar>( pow(image.at<Vec3b>(y,x)[c] / 255.0, gamma_object) * 255.0 + contrast_object*image.at<Vec3b>(y,x)[c] + brightness_object );
                }
            }
        }
      split(im_objects_allchanels, im_objects_splitedchannels);
      imshow("ObjdetectC/B", im_objects_allchanels);
      im_objects_redthr = im_objects_splitedchannels[0];
      im_objects_greenthr = im_objects_splitedchannels[1];

      createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
      createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
      createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
      createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
      createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
      createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);

      //////////////////////////////////////////////////
      /////////// Find the object centers  /////////////
      //////////////////////////////////////////////////
 
      // based on 1. thresholding of image (the "whitest" objects) and 2. Circle shape    
      GaussianBlur( im_objects_redthr, im_objects_redthr, Size( ksize, ksize ), sigma, sigma );
      GaussianBlur( im_objects_greenthr, im_objects_greenthr, Size( ksize, ksize ), sigma, sigma );
      /*threshold(im_objects_redthr, im_bw_objects_fromred, threshold_invredobjects, 255.0, THRESH_BINARY_INV);
      threshold(im_objects_greenthr, im_bw_objects_fromgreen, threshold_greenobjects, 255.0, THRESH_BINARY);
      imshow( "Red thr", im_bw_objects_fromred);
      imshow( "Green thr", im_bw_objects_fromgreen);
      im_bw_objects = im_bw_objects_fromred & im_bw_objects_fromgreen;*/


      /*imshow("red channel before threshold", im_objects_redthr);
      threshold(im_objects_redthr, im_bw_objects, 70, 255.0, THRESH_BINARY);
      imshow( "obj detect", im_bw_objects);*/
      cvtColor(im_objects_allchanels, im_bw_objects, COLOR_BGR2GRAY);
      threshold(im_bw_objects, im_bw_objects, 170, 255.0, THRESH_BINARY);

      //threshold(im_objectdetect, im_bw_objects, object_threshold, 255.0, THRESH_BINARY);
      
      // closing operation
      Mat element = getStructuringElement( MORPH_ELLIPSE, 
                        Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                        Point( erosion_size, erosion_size ));
      
      dilate( im_bw_objects, im_bw_objects, element );
      erode( im_bw_objects, im_bw_objects, element );

      imshow("gray image", im_bw_objects);

      vector< vector <Point> > contours_objects;
      vector<Point> approx_objects;
      findContours(im_bw_objects, contours_objects, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image based on threshold
      vector<Point2f>center_objects;
      vector<float>radius_vect_objects;
      center_objects.reserve(contours_objects.size());
      radius_vect_objects.reserve(contours_objects.size());

      for (int i = 0; i < contours_objects.size(); i ++)
      {
        approxPolyDP(Mat(contours_objects[i]), approx_objects, arcLength(Mat(contours_objects[i]), true)*0.02, true);

        if (fabs(contourArea(contours_objects[i])) < 100 || !isContourConvex(approx_objects))
                continue;

          if (approx_objects.size() > 6) // Detects just circles
          {
            // Detect and label objects
            double area_objects = contourArea(contours_objects[i]);
            Rect r_objects = boundingRect(contours_objects[i]);
            int radius_objects = r_objects.width / 2;
            
            if (abs(1 - ((double)r_objects.width / r_objects.height)) <= 0.3 &&
                abs(1 - (area_objects / (CV_PI * pow(radius_objects, 2)))) <= 0.3 &&
                area_objects > min_contour_area && area_objects < max_contour_area)
                {
                  //cout << "Object " << i << " area: " << area_objects << "\n";
                  minEnclosingCircle( (Mat)contours_objects[i], center_objects[i], radius_vect_objects[i] );
                  if (center_objects[i].x > 1.0 && center_objects[i].y > 1.0 
                  && center_objects[i].x < 640.0 && center_objects[i].y < 480.0 )
                  {
                    center_objects.push_back(center_objects[i]); // Fill the vector
                    radius_vect_objects.push_back(radius_vect_objects[i]);
                    circle( image, center_objects[i], (int)radius_vect_objects[i], Scalar ( 0,255,0), 2, 8, 0 ); // draw green circle around the contour
                    //cout << "Center Object coordinates" << center_objects[j];            // Show circle coordinates 
                  }
                }
          }
      }



      //////////////////////////////////////////////////
      ///////////// End of object centers  /////////////
      //////////////////////////////////////////////////

      //////////////////////////////////////////////////
      /////////// Find the holes centers  //////////////
      //////////////////////////////////////////////////
      /*im_holesdetect = Mat::zeros( image.size(), image.type() );
      for( int y = 0; y < image.rows; y++ ) 
        {
            for( int x = 0; x < image.cols; x++ ) {
                for( int c = 0; c < image.channels(); c++ ) {
                    im_holesdetect.at<Vec3b>(y,x)[c] =
                      saturate_cast<uchar>( pow(image.at<Vec3b>(y,x)[c] / 255.0, gamma_hole) * 255.0 + contrast_hole*image.at<Vec3b>(y,x)[c] + brightness_hole );
                }
            }
        }
      split(im_holesdetect, im_holes_splitedchannels);
      imshow("imholeschanells", im_holesdetect);
      im_bw_holes_fromred = im_holes_splitedchannels[2];
      imshow("Imred", im_bw_holes_fromred);
      threshold(im_bw_holes_fromred, im_bw_holes_, 170, 255.0, THRESH_BINARY);*/
      //GaussianBlur( HSV, HSV, Size( 5, 5 ), 1.5, 1.5 );
      inRange(HSV, Scalar(0, 0, 79), Scalar(112, 67, 248), im_bw_holes_);
      // closing operation
      element = getStructuringElement( MORPH_RECT, 
                        Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                        Point( erosion_size, erosion_size ));
      
      erode( im_bw_holes_, im_bw_holes_, element );
      dilate( im_bw_holes_, im_bw_holes_, element );

      vector< vector <Point> > contours_holes;
      vector<Point> approx_holes;

      findContours(im_bw_holes_, contours_holes, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE); // Find the circles in the image based on Canny edge d.
      vector<Point2f>center_holes;
      vector<float>radius_vect_holes;
      center_holes.reserve(contours_holes.size());
      radius_vect_holes.reserve(contours_holes.size());
      vector <bool> objects_flag (contours_holes.size());
      //vector<vector<Point> >hull( contours_holes.size() );

        for (int i = 0; i < contours_holes.size(); i ++)                         // iterate through each contour.
        { 
          //convexHull( Mat(contours_holes[i]), hull[i], false ); 
          approxPolyDP(Mat(contours_holes[i]), approx_holes, arcLength(Mat(contours_holes[i]), true)*0.02, true);

          if (fabs(contourArea(contours_holes[i])) < 100 ) // || !isContourConvex(approx_holes)
                continue;

          if (approx_holes.size() > 6) // Detects just circles
          {
            // Detect and label circles
            double area = contourArea(contours_holes[i]);
            Rect r = boundingRect(contours_holes[i]);
            int radius = r.width / 2;

            if (abs(1 - ((double)r.width / r.height)) <= 0.5 &&
                abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.5 &&
                area > min_contour_area && area < max_contour_area)
              {
                minEnclosingCircle( (Mat)contours_holes[i], center_holes[i], radius_vect_holes[i] );

                for (int j = 0; j < center_objects.size(); j++)
                {
                  if (center_holes[i].x < (center_objects[j].x + center_proximity) && 
                  center_holes[i].x > (center_objects[j].x - center_proximity) && 
                  center_holes[i].y < (center_objects[j].y + center_proximity) && 
                  center_holes[i].y > (center_objects[j].y - center_proximity))
                  {
                    // flag i -> do not draw the circle
                    objects_flag[i] = 1;
                  }
                }
              }
          }
          if (objects_flag[i] != 1) {
              for (int j = 0; j < center_holes.size(); j++)
              {
                if (center_holes[i].x < (center_holes[j].x + center_proximity) && 
                center_holes[i].x > (center_holes[j].x - center_proximity) && 
                center_holes[i].y < (center_holes[j].y + center_proximity) && 
                center_holes[i].y > (center_holes[j].y - center_proximity))
                {
                  center_holes.erase(center_holes.begin() + j);
                }
              }
            // draw red circle if it is a hole
            if (radius_vect_holes[i] > 3 && radius_vect_holes[i] < 30 && center_holes[i].x > 1.0 )
            {
              circle( image, center_holes[i], (int)radius_vect_holes[i], Scalar ( 0,0,255), 2, 8, 0 );
              center_holes.push_back(center_holes[i]); // Fill the vector
              radius_vect_holes.push_back(radius_vect_holes[i]);
            }
          }
        }
   


        camera_to_cv::points_array points_msg;
        camera_to_cv::points_array holes_points_msg;

        for (int i = 0; i < center_objects.size(); i++)
        {
          if (working_table_limits_found == 1)
          {
            cout << "Object [" << i << "] coordinates -> x: " << center_objects[i].x << ", y: " << center_objects[i].y << "\n";
            geometry_msgs::Point point;
            point.x = center_objects[i].x;
            point.y = center_objects[i].y;
            point.z = 0;
            if (point.x > 0 && point.x < 640 && point.y > 0 && point.y < 480)
            {
              points_msg.points.push_back(point);
            }
          }
        }
        for (int i = 0; i < center_holes.size(); i++)
        {
          if (working_table_limits_found == 1)
          {
            cout << "Hole [" << i << "] coordinates -> x: " << center_holes[i].x << ", y: " << center_holes[i].y << "\n";
            geometry_msgs::Point point_hole;
            point_hole.x = center_holes[i].x;
            point_hole.y = center_holes[i].y;
            point_hole.z = 0;
            if (point_hole.x > 0 && point_hole.x < 640 && point_hole.y > 0 && point_hole.y < 480)
            {
              holes_points_msg.points.push_back(point_hole);
            }
          }
        }
        
      //////////////////////////////////////////////////
      ///////////// End of holes centers  //////////////
      //////////////////////////////////////////////////
 
      //////////////////////////////////////////////////
      ////////////// End image processing //////////////
      //////////////////////////////////////////////////
      
      waitKey(3);
      
      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
      if (working_table_limits_found == 1) {
        object_pub.publish(points_msg);
        holes_pub.publish(holes_points_msg);
      }

      ///////////////// Display images ///////////////////
      imshow("Object detect", im_bw_objects);
      imshow("Holes detect", im_bw_holes_);
      imshow(original_image_w, im_orig);
      //imshow("Grayscale out of blue c", im_gray);
      namedWindow(adjusted_image, CV_WINDOW_AUTOSIZE);
      createTrackbar("Brightness (reverse values)", adjusted_image, &brightness_level, max_brightness, ParametersChange);
      createTrackbar("Contrast", adjusted_image, &contrast_level, max_contrast, ParametersChange);
      createTrackbar("Gamma correction", adjusted_image, &gamma_level, 20000, ParametersChange);
      imshow(OPENCV_WINDOW, image);
      working_table_limits_found = 0;
      table_found_pub.publish(table_found_msg);
      table_properties_pub.publish(table_properties_msg);
    }
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


