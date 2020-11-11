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

#include "StereoPipeline.h"

//----------------------------------------------------------------------------------
// Type Definitions
//----------------------------------------------------------------------------------
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

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
  // Initialize ROS
  ros::init(argc, argv, "image_listener");

  // Setup the pipeline
  auto pipeline = Amantis::StereoPipeline();

  // Setup the messaging
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> image1(nh, "camera_array/cam1/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image2(nh, "camera_array/cam2/image_raw", 1);

  // Register the callback
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image1, image2);
  sync.registerCallback(boost::bind(&Amantis::StereoPipeline::Launch, pipeline, _1, _2));

  // Process Loop
  cv::namedWindow("view");
  ros::spin();
  cv::destroyWindow("view");

  // Return success
  return EXIT_SUCCESS;
}

