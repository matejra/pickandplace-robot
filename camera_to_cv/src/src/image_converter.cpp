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

using namespace cv;
using namespace std;

// Constants for image processing
int sigma = 1.2;
int ksize = 5;
int erosion_size = 2; 
double min_contour_area = 20;
double max_contour_area = 400;
float center_proximity = 7.0;
double contour_area = 0;
double holes_threshold = 60;
double object_threshold = 180;
int max_rectangle_index = 0;
bool working_table_limits_found = 0;
int n_expected_objects = 100;
static const std::string OPENCV_WINDOW = "Color image";
Rect working_table;
Mat im_working_table_gray;

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void setLabel(Mat& im, const string label, vector<Point>& contour)
{
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	Size text = getTextSize(label, fontface, scale, thickness, &baseline);
	Rect r = boundingRect(contour);

	Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher chatter_pub = nh_.advertise<geometry_msgs::Point>("chatter", 1000);


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
    Mat image, im_gray, im_bw_canny, im_gray_edges;
    image = cv_ptr->image;
    cvtColor(image, im_gray, COLOR_BGR2GRAY);
    //char* imageName = argv[1];
    //image = imread( imageName, 1 );
    //cvtColor(image, im_gray, COLOR_BGR2GRAY);
    Mat bw;
	  Canny(im_gray, im_bw_canny, 100, 300, 5);
    GaussianBlur(im_gray, im_gray, Size(ksize, ksize), sigma, sigma);
    
    ///////// Getting working table limits
    // shape detection based on https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
    // should be done just once - to find the table
    vector< vector <Point> > contours_table;
    vector<Point> approx;
    double largest_rectangle_area = 0;
    if (working_table_limits_found == 0)
    {
      findContours(im_bw_canny, contours_table, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the rectangles in the image
      for (int i = 0; i < contours_table.size(); i ++)                         // iterate through each contour.
      { 
        approxPolyDP(Mat(contours_table[i]), approx, arcLength(Mat(contours_table[i]), true)*0.02, true);

        if (fabs(contourArea(contours_table[i])) < 100 || !isContourConvex(approx))
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

          // Use the degrees obtained above and the number of vertices
          // to determine the shape of the contour
          if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
          {
            setLabel(im_bw_canny, "RECT", contours_table[i]);
            // Find the greatest rectangle
            Rect r = boundingRect(contours_table[i]);
            double rectangle_area = r.height*r.width; 
            if (rectangle_area>largest_rectangle_area) //find the largest rectangle
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
        //imshow("Working table", im_working_table_gray);
        working_table_limits_found = 1;
        cout << "Working table found \n";
      }
    }

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
    ///////// End working table limits

    //////// Find the object centers based on 1. thresholding of image (the "whitest" objects) and 2. Circle shape
    threshold(im_gray_, im_bw_objects, object_threshold, 255.0, THRESH_BINARY);
    
    // Apply (uncomment) erosion if the objects are too close together
    /*Mat element = getStructuringElement( MORPH_ELLIPSE, 
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ));
    erode( im_bw_objects, im_bw_objects, element );*/
    
    vector< vector <Point> > contours_objects;
    vector<Point> approx_objects;
    findContours(im_bw_objects, contours_objects, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image based on threshold
    vector<Point2f>center_objects;
    vector<float>radius_vect_objects;
    center_objects.reserve(n_expected_objects);
    radius_vect_objects.reserve(n_expected_objects);

    int j = 0;
    for (int i = 0; i < contours_objects.size(); i ++)                         // iterate through each contour.
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
                minEnclosingCircle( (Mat)contours_objects[i], center_objects[j], radius_vect_objects[j] );
                center_objects.push_back(center_objects[j]); // Fill the vector
                radius_vect_objects.push_back(radius_vect_objects[j]);
                circle( image, center_objects[j], (int)radius_vect_objects[j], Scalar ( 0,255,0), 2, 8, 0 ); // draw green circle around the contour
                //cout << "Center Object coordinates" << center_objects[j];            // Show circle coordinates 
                j++;
              }
        }
    }
    //////// End of object centers
//
    //////// Find the holes centers
    Mat im_bw_holes_;
    // adaptiveThreshold(im_gray_, im_bw_holes_, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 25, 5);
    threshold(im_gray_, im_bw_holes_, 60.0, 255.0, THRESH_BINARY);
    // Opening - erosion followed by dilation for noise (glare) removal
    Mat element = getStructuringElement( MORPH_ELLIPSE, 
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ));
    
    erode( im_bw_holes_, im_bw_holes_, element );
    dilate( im_bw_holes_, im_bw_holes_, element );
<<<<<<< HEAD


    //Canny(im_gray_, im_bw_holes_, 10, 50, 3);
    vector< vector <Point> > contours_holes;
    vector<Point> approx_holes;



    findContours(im_bw_holes_, contours_holes, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE); // Find the circles in the image based on Canny edge d.
    vector<Point2f>center_holes;
    vector<float>radius_vect_holes;
    vector<Point2f>true_object_center (center_objects.size());
    center_holes.reserve(contours_holes.size());
    radius_vect_holes.reserve(contours_holes.size());
    vector <bool> objects_flag (contours_holes.size());

=======


    //Canny(im_gray_, im_bw_holes_, 10, 50, 3);
    vector< vector <Point> > contours_holes;
    vector<Point> approx_holes;



    findContours(im_bw_holes_, contours_holes, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE); // Find the circles in the image based on Canny edge d.
    vector<Point2f>center_holes;
    vector<float>radius_vect_holes;
    vector<Point2f>true_object_center (center_objects.size());
    center_holes.reserve(contours_holes.size());
    radius_vect_holes.reserve(contours_holes.size());
    vector <bool> objects_flag (contours_holes.size());

>>>>>>> 137869a2d2c6d56b2b25cbefa4a4e031b7538920
      for (int i = 0; i < contours_holes.size(); i ++)                         // iterate through each contour.
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

          if (abs(1 - ((double)r.width / r.height)) <= 0.5 &&
              abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.5)
            {
              //setLabel(im_bw_holes_, "CIR", contours_holes[i]);
              minEnclosingCircle( (Mat)contours_holes[i], center_holes[i], radius_vect_holes[i] );

              for (int j = 0; j < center_objects.size(); j++)
              {
                if (center_holes[i].x < (center_objects[j].x + center_proximity) && 
                center_holes[i].x > (center_objects[j].x - center_proximity) && 
                center_holes[i].y < (center_objects[j].y + center_proximity) && 
                center_holes[i].y > (center_objects[j].y - center_proximity))
                {
                  true_object_center[j] = center_objects[j];
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
          // draw red circle if it is a hole -- not working yet
          if (radius_vect_holes[i] > 0 && radius_vect_holes[i] < 30)
          {
            circle( image, center_holes[i], (int)radius_vect_holes[i], Scalar ( 0,0,255), 2, 8, 0 );
            center_holes.push_back(center_holes[i]); // Fill the vector
            radius_vect_holes.push_back(radius_vect_holes[i]);
          }
        }
      }

      for (int i = 0; i < true_object_center.size(); i++)
      {
        cout << "Object [" << i << "] coordinates -> x: " << true_object_center[i].x << ", y: " << true_object_center[i].y << "\n";
      }
      for (int i = 0; i < center_holes.size(); i++)
      {
        cout << "Hole [" << i << "] coordinates -> x: " << center_holes[i].x << ", y: " << center_holes[i].y << "\n";
      }
<<<<<<< HEAD
      geometry_msgs::Point holes_msg;
      holes_msg.x = true_object_center[0].x;
      holes_msg.y = true_object_center[0].y;
=======

>>>>>>> 137869a2d2c6d56b2b25cbefa4a4e031b7538920
      // cout << center_holes.size();
//
      //////// End of the holes centers

      //imshow("Color image", cv_ptr->image);
      imshow(OPENCV_WINDOW, image);

      //imshow(OPENCV_WINDOW, im_gray_);
      imshow("Holes thresholding", im_bw_holes_);
      ///////// End image processing /////////
    waitKey(3);
    
    chatter_pub.publish(holes_msg);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


