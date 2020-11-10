//----------------------------------------------------------------------------------
// Defines the functionality to listen for messages on the queue
//
// @author: Wild Boar
//----------------------------------------------------------------------------------

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------------
// Type Definitions
//----------------------------------------------------------------------------------
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

//----------------------------------------------------------------------------------
// Function Prototypes
//----------------------------------------------------------------------------------
void Callback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2);

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
  message_filters::Subscriber<sensor_msgs::Image> image1(nh, "camera_array/cam1/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image2(nh, "camera_array/cam2/image_raw", 1);

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image1, image2);
  sync.registerCallback(boost::bind(&Callback, _1, _2));

  cv::namedWindow("view");
  ros::spin();
  cv::destroyWindow("view");

  return EXIT_SUCCESS;
}

//----------------------------------------------------------------------------------
// Image Callback Functionality
//----------------------------------------------------------------------------------

/**
 * Handles the image callback
 * @param image1 The first image passed to the system
 * @param image2 The second image passed to the system
 */
void Callback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2)
{
  try
  {
      cv::Mat imageData1 = cv_bridge::toCvShare(image1, "bgr8")->image;
      cv::Mat imageData2 = cv_bridge::toCvShare(image2, "bgr8")->image;

      cv::Mat smallImage1; cv::resize(imageData1, smallImage1, cv::Size(), 0.25, 0.25);
      cv::Mat smallImage2; cv::resize(imageData2, smallImage2, cv::Size(), 0.25, 0.25);

      cv::imshow("Image 1", smallImage1); cv::imshow("Image 2", smallImage2);
      cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Unable to show images on the screen");
  }
}
