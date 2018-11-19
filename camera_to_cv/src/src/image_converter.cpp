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

using namespace cv;
using namespace std;

// Constants for image processing
int sigma = 1.2;
int ksize = 5;
int erosion_size = 4; 
double min_contour_area = 20;
double max_contour_area = 200;
double contour_area = 0;
double table_threshold = 100;
double object_threshold = 180;
int max_rectangle_index = 0;
bool working_table_limits_found = 0;
static const std::string OPENCV_WINDOW = "Grayscale";
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

    Mat im_gray, im_bw, im_gray_edges;
    cvtColor(cv_ptr->image, im_gray, COLOR_BGR2GRAY);
    Mat bw;
	  Canny(im_gray, im_bw, 150, 450, 5);
    GaussianBlur(im_gray, im_gray, Size(ksize, ksize), sigma, sigma);
    
    /// Getting working table limits

    // shape detection based on https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
    // should be done just once - to find the table
    vector< vector <Point> > contours_table;
    vector<Point> approx;
    double largest_rectangle_area = 0;
    if (working_table_limits_found == 0)
    {
      findContours(im_bw, contours_table, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the rectangles in the image
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
            setLabel(im_bw, "RECT", contours_table[i]);
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
    // End shape detection
    /// End working table limits

    // If the working table is found work with coordinates of working table, otherwise whole image
    Mat im_gray_objects, im_bw_objects;
    if (working_table_limits_found == 1)
    {
      im_gray_objects = im_gray(working_table).clone();
    } else {
      im_gray_objects = im_gray;
    }

    /// Getting the object centers
    threshold(im_gray_objects, im_bw_objects, object_threshold, 255.0, THRESH_BINARY);
    Mat element = getStructuringElement( MORPH_ELLIPSE, 
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ));
    erode( im_bw_objects, im_bw_objects, element );
    
    vector< vector <Point> > contours; // Vector for storing contour

    findContours(im_bw_objects, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
    for (int i = 0; i < contours.size(); i ++)                         // iterate through each contour.
    {
      Rect r_objects = boundingRect(contours[i]); 
      contour_area = r_objects.width * r_objects.height;  //  Find the area of contour
      // Check the contour area limits and relation between height and width
      if(contour_area > min_contour_area && contour_area < max_contour_area)
      {
        // draw unfilled rectangle around the contour
        //rectangle(im_gray_objects, r_objects, Scalar(0, 0, 255), 2, 8, 0);
      }
    }
    /// End the object centers

    /// Find the holes centers

    vector< vector <Point> > contours_circles;
    vector<Point> approx_circles;


    findContours(im_bw, contours_circles, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the circles in the image
    vector<Point2f>center( contours_circles.size() );
    vector<float>radius( contours_circles.size() );

      for (int i = 0; i < contours_circles.size(); i ++)                         // iterate through each contour.
      { 
        approxPolyDP(Mat(contours_circles[i]), approx_circles, arcLength(Mat(contours_circles[i]), true)*0.02, true);

        if (fabs(contourArea(contours_circles[i])) < 100 || !isContourConvex(approx_circles))
              continue;

        if (approx_circles.size() > 6)
        {
          // Detect and label circles
          double area = contourArea(contours_circles[i]);
          Rect r = boundingRect(contours_circles[i]);
          int radius = r.width / 2;

          if (abs(1 - ((double)r.width / r.height)) <= 0.3 &&
              abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.3)
            setLabel(im_bw, "CIR", contours_circles[i]);
            
            // Show circle coordinates 
        }
      }
    imshow("Canny edge detector", im_bw);
    waitKey(3);
    /// End of holes centers
   
    imshow("Color image", cv_ptr->image);

    imshow(OPENCV_WINDOW, im_gray_objects);
    imshow("Binary", im_bw_objects);
    ///////// End image processing /////////
    waitKey(3);
    

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


