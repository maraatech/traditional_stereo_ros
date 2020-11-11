//--------------------------------------------------
// A set of utilities for manipulating stereo frames
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>

#include "StereoFrame.h"
#include "Calibration.h"
#include "RectificationParameters.h"

namespace Amantis 
{
	class StereoFrameUtils  
	{
		public:
			static StereoFrame Resize(StereoFrame& frame, int width);
			static void Save(StereoFrame& frame, const string& baseFolder, const string& suffix, const string& extension);
			static StereoFrame Undistort(Calibration* calibration, StereoFrame& frame);
			static StereoFrame RectifyFrames(Calibration * calibration, RectificationParameters* parameters, StereoFrame& frame); 
	};
}