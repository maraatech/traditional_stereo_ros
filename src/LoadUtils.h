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

#include "Calibration.h"

namespace Amantis
{
	class LoadUtils
	{
	public:
		static Calibration * LoadCalibration(const string& folder);
	};
}
