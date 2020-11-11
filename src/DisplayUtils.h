//--------------------------------------------------
// A set of utilities for displaying various things within the project.
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>

#include "StereoFrame.h"

namespace Amantis
{
	class DisplayUtils
	{
	public:
		static void ShowStereoFrame(const string& caption, StereoFrame* frame, int width);
	};
}
