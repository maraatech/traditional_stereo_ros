//--------------------------------------------------
// A set of utilities wrt dealing with point clouds
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "Math3D.h"
#include "ColorPoint.h"

namespace Amantis
{
	class CloudUtils
	{
	public:
		static Mat ExtractDepthMap(Mat& Q, Mat& disparityMap, double zmin, double zmax);
		static void ExtractCloud(vector<ColorPoint>& points, Mat& Q, Mat& color, Mat& depth);
		static void TransformCloud(Mat& pose, vector<ColorPoint>& points);
	};
}
