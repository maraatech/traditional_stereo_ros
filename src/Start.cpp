//----------------------------------------------------------------------------------
// Defines the functionality to listen for messages on the queue
//
// @author: Wild Boar
//----------------------------------------------------------------------------------

// Standard includes
#include <iostream>
using namespace std;

// Ros Includes
#include <ros/ros.h>

// Messaging includes
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Amantis Includes
#include "Constants.h"
#include "StereoFrame.h"
#include "StereoFrameUtils.h"
#include "DisplayUtils.h"
#include "GeneralUtils.h"
using namespace Amantis;

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
      // Retrieve the raw images
      cv::Mat imageData1 = cv_bridge::toCvShare(image1, "bgr8")->image;
      cv::Mat imageData2 = cv_bridge::toCvShare(image2, "bgr8")->image;

      // Build a stereo frame
      auto frame = StereoFrame(imageData1, imageData2);

      // Show the stereo frame on the screen
      DisplayUtils::ShowStereoFrame("Frame", &frame, 1000);

      // Create a small version of the frame
      auto smallFrame = StereoFrameUtils::Resize(frame, 1000);

      // Save the frame to disk
      string suffix = GeneralUtils::GetTimeString();
      auto baseFolder = GeneralUtils::CompletePath(IMAGE_FOLDER);
      StereoFrameUtils::Save(smallFrame, baseFolder, suffix, ".jpg");

      // Pause the screen
      cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Unable to show images on the screen");
  }
}
