//--------------------------------------------------
// Implementation code for CloudUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "CloudUtils.h"
using namespace Amantis;

//--------------------------------------------------
// ExtractDepthMap
//--------------------------------------------------

/**
 * Extract a depth map from a disparity map and Q Matrix
 * @param Q The given Q Matrix
 * @param disparityMap The disparity map that we are presenting
 * @param zmin The minimum Z value
 * @param zmax The maximum Z value
 */
Mat CloudUtils::ExtractDepthMap(Mat& Q, Mat& disparityMap, double zmin, double zmax)
{
	auto qdata = (double*)Q.data;

	auto f = qdata[11];
	auto baseline = (-1.0 / qdata[14]) * 1000; // convert to mm
	if (baseline < 0) baseline *= -1; // Make sure that the baseline is always positive

	Mat result = Mat_<double>::zeros(disparityMap.size());

	auto input = (double*)disparityMap.data;
	auto output = (double*)result.data;

	for (auto row = 0; row < disparityMap.rows; row++) 
	{
		for (auto column = 0; column < disparityMap.cols; column++) 
		{
			auto index = column + row * disparityMap.cols;

			auto disparityValue = input[index];
			auto Z = (f * baseline) / disparityValue;

			if (Z < zmin || Z > zmax) continue;

			output[index] = Z;
		}	
	}

	return result;
}

//--------------------------------------------------
// ExtractCloud
//--------------------------------------------------

/**
 * Add the logic to extract the given point cloud
 * @param points The points that we are extracting
 * @param Q The Q matrix containing the calibration parameters of the rectified system
 * @param color The color map containing the associated colors
 * @param depth The depth map containing the associated depth values
 */
void CloudUtils::ExtractCloud(vector<ColorPoint>& points, Mat& Q, Mat& color, Mat& depth) 
{
	// Extract elements from the Q matrix
	auto qdata = (double*)Q.data;
	auto cx = -qdata[3];
	auto cy = -qdata[7];
	auto f = -qdata[11];

	auto depthData = (double*)depth.data;

	for (auto row = 0; row < color.rows; row++) 
	{
		for (auto column = 0; column < color.cols; column++) 
		{
			auto index = column + row * color.cols;

			auto Z = depthData[index];
			if (Z <= 0) continue;

			auto R = color.data[index * 3 + 2];
			auto G = color.data[index * 3 + 1];
			auto B = color.data[index * 3 + 0];

			auto X = -(column - cx) * (Z / f);
			auto Y = -(row - cy) * (Z / f);

			points.push_back(ColorPoint(Vec3i(R, G, B), Point3d(X, Y, Z)));		
		}	
	}

}

//--------------------------------------------------
// TransformCloud
//--------------------------------------------------

/**
 * Add the logic to transform a cloud given a pose
 * @param pose The pose that we want to transform the cloud to
 * @param points The list of points that we are transforming 
 */
void CloudUtils::TransformCloud(Mat& pose, vector<ColorPoint>& points) 
{
	for (auto & point : points) point.Transform(pose);
}
