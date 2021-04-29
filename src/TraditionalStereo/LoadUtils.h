//--------------------------------------------------
// A set of utilities for loading things
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;
#include "Constants.h"
#include <cares_msgs/StereoCameraInfo.h>

#include "Calibration.h"

namespace Amantis
{
	class LoadUtils
	{
	public:
		static Calibration * LoadCalibration(const string& folder, double ratio);
		static void vector2Mat(std::vector<std::vector<double>>& list, Mat& out);
		static void vector2Mat(std::vector<double>& list, Mat& out);
		static Calibration * LoadCalibration(const cares_msgs::StereoCameraInfo ci, double ratio);
	};
}
