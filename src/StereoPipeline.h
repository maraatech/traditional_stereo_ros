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
#include <maara_msgs/StereoCameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Amantis Includes
#include "Constants.h"
#include "DepthFrame.h"
#include "Calibration.h"
#include "Math3D.h"
#include "SGBM.h"
#include "LoadUtils.h"
#include "StereoFrame.h"
#include "OpticsUtils.h"
#include "StereoFrameUtils.h"
#include "DisplayUtils.h"
#include "GeneralUtils.h"
#include "CloudUtils.h"
#include "PointSaver.h"
using namespace Amantis;

namespace Amantis 
{
	class StereoPipeline  
	{
		private:
			Calibration * _calibration;
			RectificationParameters * _rectificationParameters;
		public:
			StereoPipeline(ros::NodeHandle& _nh);
			~StereoPipeline();

			void Launch(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2);
		private:
		    ros::NodeHandle _nh;
			ros::Publisher _pc_pub;
			ros::Publisher _depth_pub;
			void ProcessFrame(StereoFrame& frame, ros::Time timestamp);
			void PublishDepthFrame(DepthFrame& frame, ros::Time timestamp);
			StereoFrame RemoveDistortion(StereoFrame& frame); 
			StereoFrame PerformRectification(StereoFrame& frame); 
			DepthFrame PerformStereoMatching(StereoFrame& frame); 
			DepthFrame PerformDepthExtraction(DepthFrame& frame); 
			void GenerateModel(DepthFrame& frame, ros::Time timestamp);
			void LoadCalibrationCallback(const maara_msgs::StereoCameraInfo ci);
			unsigned int cvtRGB(Vec3i _color);
	};
}