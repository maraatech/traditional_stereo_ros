//--------------------------------------------------
// Implementation code for Math3D
//
// @author: Wild Boar
//--------------------------------------------------

#include "Math3D.h"
using namespace Amantis;

//--------------------------------------------------
// Project
//--------------------------------------------------

/**
 * Project a point based on the associated camera matrix
 * @param cameraMatrix The camera matrix that we are basing our project upon
 * @param point The point that we are projecting
 */
Point2d Math3D::Project(const Mat& cameraMatrix, const Point3d& point) 
{
	auto cdata = (double*)cameraMatrix.data;

	auto fx = cdata[0]; auto fy = cdata[4];
	auto cx = cdata[2]; auto cy = cdata[5];

	auto u = (fx * point.x / point.z) + cx;
	auto v = (fy * point.y / point.z) + cy;

	return Point2d(u, v);
}

//--------------------------------------------------
// UnProject
//--------------------------------------------------

/**
 * Find the original 3D point an image point, given the camera matrix and the depth 
 * @param cameraMatrix The camera matrix defining the projecting transform
 * @param point The image point that we are projecting back
 * @param Z The depth expected in 3D space
 * @return The original 3D location of the point
 */
Point3d Math3D::UnProject(const Mat& cameraMatrix, const Point2d& point, double Z) 
{
	auto cdata = (double*)cameraMatrix.data;

	auto fx = cdata[0]; auto fy = cdata[4];
	auto cx = cdata[2]; auto cy = cdata[5];

	auto X = (point.x - cx) * (Z / fx);
	auto Y = (point.y - cy) * (Z / fy);

	return Point3d(X, Y, Z);
}

//--------------------------------------------------
// TransformPoint
//--------------------------------------------------

/**
 * Transform a 3D point with a given pose
 * @param pose The pose matrix that we are transforming with
 * @param point The point that we are transforming
 * @return Return a Point3d
 */
Point3d Math3D::TransformPoint(const Mat& pose, const Point3d & point)
{
	auto data = (double*)pose.data;

	auto X = data[0] * point.x + data[1] * point.y + data[2] * point.z + data[3];
	auto Y = data[4] * point.x + data[5] * point.y + data[6] * point.z + data[7];
	auto Z = data[8] * point.x + data[9] * point.y + data[10] * point.z + data[11];

	return Point3d(X, Y, Z);
}

//--------------------------------------------------
// ExtractDepth
//--------------------------------------------------

/**
 * Retrieve the depth of a 2D point from the depth map
 * @param depthMap The depth map that we are retrieving from
 * @param position The position that we are retrieving from
 * @return The depth value as a double
 */
double Math3D::ExtractDepth(Mat& depthMap, const Point2d & position)
{
	auto u = (int)round(position.x); auto v = (int)round(position.y);

	if (u < 0 || u >= depthMap.cols || v < 0 || v >= depthMap.rows) return 0;

	return ((double*)depthMap.data)[u + v * depthMap.cols];
}

//--------------------------------------------------
// ExtractColor
//--------------------------------------------------

/**
 * Retrieve the color of a 2D point from a color map
 * @param colorMap The color map that we are getitng the color from
 * @param position The position that we are getting the color from
 * @return The associated color value
 */
Vec3i Math3D::ExtractColor(Mat& colorMap, const Point2d& position) 
{
	auto u = (int)round(position.x); auto v = (int)round(position.y);

	if (u < 0 || u >= colorMap.cols || v < 0 || v >= colorMap.rows) return Vec3i();

	auto index = u + v * colorMap.cols;

	auto blue = colorMap.data[index * 3 + 0];
	auto green = colorMap.data[index * 3 + 1];
	auto red = colorMap.data[index * 3 + 2];

	return Vec3i(red, green, blue);
}

//--------------------------------------------------
// ExtractK
//--------------------------------------------------

/**
 * Retrieve the camera matrix from the Q matrix 
 */
Mat Math3D::ExtractK(Mat& Q) 
{
	Mat result = Mat_<double>::eye(3, 3);

	auto qdata = (double*)Q.data;
	auto kdata = (double*)result.data;

	kdata[2] = -qdata[3];
	kdata[5] = -qdata[7];
	kdata[0] = qdata[11];
	kdata[4] = qdata[11];

	return result;
}

//--------------------------------------------------
// GetViewLimits
//--------------------------------------------------

/**
 * Find the view limits for the given camera
 * @param cameraMatrix The projection model for the given camera
 * @param imageSize The size of the image that we are processing
 * @param zmin The minimum distance we care about from the camera
 * @param zmax The maximum distance we care about from the camera
 */
void Math3D::GetViewLimits(Mat& cameraMatrix, const Size& imageSize, double zmin, double zmax, vector<Point3d>& output)
{
	// Add the close points
	output.push_back(UnProject(cameraMatrix, Point2d(0, 0), zmin));
	output.push_back(UnProject(cameraMatrix, Point2d(imageSize.width, 0), zmin));
	output.push_back(UnProject(cameraMatrix, Point2d(imageSize.width, imageSize.height), zmin));
	output.push_back(UnProject(cameraMatrix, Point2d(0, imageSize.height), zmin));

	// Add the far points
	output.push_back(UnProject(cameraMatrix, Point2d(0, 0), zmax));
	output.push_back(UnProject(cameraMatrix, Point2d(imageSize.width, 0), zmax));
	output.push_back(UnProject(cameraMatrix, Point2d(imageSize.width, imageSize.height), zmax));
	output.push_back(UnProject(cameraMatrix, Point2d(0, imageSize.height), zmax));
}

//--------------------------------------------------
// TransformPointSet
//--------------------------------------------------

/**
 * Transform the given point set
 * @param input The input set of points that we are transforming
 * @param output The output set of points after the transformation
 */
void Math3D::TransformPointSet(const Mat& pose, vector<Point3d>& input, vector<Point3d>& output)
{
	for (const auto& point : input)
	{
		output.push_back(TransformPoint(pose, point));
	}
}

//--------------------------------------------------
// FindDisparity
//--------------------------------------------------

/**
 * NOTE: ASSUMPTION THAT THIS IS NOT A RIGHT-LEFT SYSTEM! 
 * Estimate the disparity given a particular Z value
 * @param Q The Q matrix from rectification (assuming Zero Disparity setting)
 * @parma Z The distance component that we want the disparity for
 * @param roundUp Indicates whether we should be rounding down or up
 * @return The disparity rounded to the closest 16
 */
double Math3D::FindDisparity(const Mat& Q, double Z, bool roundUp) 
{
	auto qdata = (double*)Q.data;

	if (Z == 0 || qdata[15] != 0) return INFINITY;

	auto f = qdata[11];
	auto baseline = (-1.0 / qdata[14]) * 1000; // convert to mm
	if (baseline < 0) baseline *= -1; // Make sure that the baseline is always positive

	auto rawDisparity = (f * baseline) / Z;

	auto last16 = (int)floor(rawDisparity / 16.0);

	return roundUp ? (last16 + 1) * 16 : (last16) * 16;
}