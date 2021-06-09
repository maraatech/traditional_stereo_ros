//--------------------------------------------------
// A pipeline for holding the stereo processing
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

// Standard includes
#include <iostream>
using namespace std;

// Ros Includes
#include <ros/ros.h>

// Messaging includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cares_msgs/StereoCameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Amantis Includes
#include "TraditionalStereo/Constants.h"
#include "TraditionalStereo/DepthFrame.h"
#include "TraditionalStereo/Calibration.h"
#include "TraditionalStereo/Math3D.h"
#include "TraditionalStereo/SGBM.h"
#include "TraditionalStereo/LoadUtils.h"
#include "TraditionalStereo/StereoFrame.h"
#include "TraditionalStereo/OpticsUtils.h"
#include "TraditionalStereo/StereoFrameUtils.h"
#include "TraditionalStereo/DisplayUtils.h"
#include "TraditionalStereo/GeneralUtils.h"
#include "TraditionalStereo/CloudUtils.h"
#include "TraditionalStereo/PointSaver.h"
using namespace Amantis;

namespace Amantis
{
	class StereoPipeline  
	{
		private:
	    	double _scale;

			Calibration * _calibration;
			RectificationParameters * _rectificationParameters;

			ros::Publisher _imageColorPublisher;
			ros::Publisher _cameraInfoPublisher;
  			ros::Publisher _pointCloudPublisher;
      		ros::Publisher _depthPublisher;
  		public:
			StereoPipeline(double scale = 1.0);
			~StereoPipeline();

			void Launch(const sensor_msgs::ImageConstPtr& left_image_msg, const sensor_msgs::ImageConstPtr& right_image_msg, const cares_msgs::StereoCameraInfoConstPtr& stereo_info);
		private:

			void ProcessFrame(StereoFrame& frame, std_msgs::Header& header);
			StereoFrame RemoveDistortion(StereoFrame& frame);
			StereoFrame PerformRectification(StereoFrame& frame); 
			DepthFrame PerformStereoMatching(StereoFrame& frame); 
			DepthFrame PerformDepthExtraction(DepthFrame& frame); 

			void SetCalibration(const cares_msgs::StereoCameraInfo& stereo_info, double ratio);
      		Calibration* LoadCalibration(const cares_msgs::StereoCameraInfo& stereo_info, double ratio);

      		void GenerateAndPublishPointCloud(DepthFrame& frame, std_msgs::Header& header);
      		void PublishDepthFrame(DepthFrame& frame, std_msgs::Header& header);
	};
}