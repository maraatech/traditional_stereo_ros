//--------------------------------------------------
// A simple wrapper around OpenCv's semi-global block matching algorithm.
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "Constants.h"
#include "StereoFrame.h"

namespace Amantis
{
	class SGBM
	{
	private:
		int _minDisparity;
		int _maxDisparity;
		Ptr<StereoSGBM> _matcher;
	public:
		SGBM(int  minDisparity, int maxDisparity);

		Mat Match(StereoFrame & frame);
	private:
		Mat ConvertResult(Mat& disparity);
	};
}
