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
using namespace Amantis;

namespace Amantis 
{
	class StereoPipeline  
	{
		private:
			Calibration * _calibration;
			RectificationParameters * _rectificationParameters;
		public:
			StereoPipeline();
			~StereoPipeline();

			void Launch(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2);
		private:
			void ProcessFrame(StereoFrame& frame);

			StereoFrame RemoveDistortion(StereoFrame& frame); 
			StereoFrame PerformRectification(StereoFrame& frame); 
			DepthFrame PerformStereoMatching(StereoFrame& frame); 
			DepthFrame PerformDepthExtraction(DepthFrame& frame); 
	};
}