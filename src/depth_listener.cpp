//----------------------------------------------------------------------------------
// Defines the functionality to listen for messages on the queue
//
// @author: Wild Boar
//----------------------------------------------------------------------------------

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//----------------------------------------------------------------------------------
// Function Prototypes
//----------------------------------------------------------------------------------
void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

//----------------------------------------------------------------------------------
// Main entry point
//----------------------------------------------------------------------------------

/**
 * Main callback method
 * @param argc The number of incomming arguments passed to the application
 * @param argv The values of the incomming arguments
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  ros::NodeHandle nh;

  cv::namedWindow("view");
  //cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, ImageCallback);

  ros::spin();

  cv::destroyWindow("view");

  return EXIT_SUCCESS;
}

//----------------------------------------------------------------------------------
// Image Callback Functionality
//----------------------------------------------------------------------------------

/**
 * Handles the image callback
 * @param msg The image message that was passed to the system
 */
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
