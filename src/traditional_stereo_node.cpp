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
#include <cares_msgs/StereoCameraInfo.h>

#include "StereoPipeline.h"
#include "parameters.h"
//----------------------------------------------------------------------------------
// Type Definitions
//----------------------------------------------------------------------------------
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, cares_msgs::StereoCameraInfo> SyncPolicy;

//----------------------------------------------------------------------------------
// Main entry point
//----------------------------------------------------------------------------------

/**
 * Main callback method
 * @param argc The number of incoming arguments passed to the application
 * @param argv The values of the incoming arguments
 */
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "image_listener");

  // Setup the pipeline
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string point_cloud, depth_image;
  if(!nh_private.getParam(CARES::TraditionalStereo::POINT_CLOUD_S, point_cloud)){
    ROS_ERROR((CARES::TraditionalStereo::LEFT_IMAGE_S+" not set.").c_str());
    return EXIT_FAILURE;
  }
  if(!nh_private.getParam(CARES::TraditionalStereo::DEPTH_IMAGE_S, depth_image)){
    ROS_ERROR((CARES::TraditionalStereo::RIGHT_IMAGE_S+" not set.").c_str());
    return EXIT_FAILURE;
  }
  double scale = 1.0;
  nh_private.param(CARES::TraditionalStereo::SCALE_D, scale, 1.0);
  if(scale <= 0){
    ROS_ERROR("Scale set to or below 0");
    exit(EXIT_FAILURE);
  }
  ROS_INFO("%s", point_cloud.c_str());
  ROS_INFO("%s", depth_image.c_str());
  StereoPipeline pipeline = Amantis::StereoPipeline(point_cloud, depth_image, scale);

  std::string left_image, right_image, stereo_info;
  if(!nh_private.getParam(CARES::TraditionalStereo::LEFT_IMAGE_S, left_image)){
    ROS_ERROR((CARES::TraditionalStereo::LEFT_IMAGE_S+" not set.").c_str());
    return EXIT_FAILURE;
  }
  if(!nh_private.getParam(CARES::TraditionalStereo::RIGHT_IMAGE_S, right_image)){
    ROS_ERROR((CARES::TraditionalStereo::RIGHT_IMAGE_S+" not set.").c_str());
    return EXIT_FAILURE;
  }
  if(!nh_private.getParam(CARES::TraditionalStereo::STEREO_INFO_S, stereo_info)){
    ROS_ERROR((CARES::TraditionalStereo::STEREO_INFO_S+" not set.").c_str());
    return EXIT_FAILURE;
  }

  // Setup the messaging
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, left_image, 1);
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub(nh, right_image, 1);
  message_filters::Subscriber<cares_msgs::StereoCameraInfo> stereo_info_sub(nh, stereo_info, 1);

  // Register the callback
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(1), left_image_sub, right_image_sub, stereo_info_sub);
  sync.registerCallback(boost::bind(&StereoPipeline::Launch, &pipeline, _1, _2, _3));

  // Process Loop
  cv::namedWindow("view");
  ros::spin();
  cv::destroyWindow("view");

  // Return success
  return EXIT_SUCCESS;
}

