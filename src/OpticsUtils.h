//--------------------------------------------------
// Defines a set of utilities associated with optics.
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "Calibration.h"
#include "RectificationParameters.h"

namespace Amantis
{
	class OpticsUtils
	{
	public:
		static RectificationParameters* FindRectification(Calibration* calibration);
		static Mat ExtractDepthMap(Mat& Q, Mat& disparityMap, double zmin, double zmax);
	};
}
