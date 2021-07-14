//--------------------------------------------------
// A set of utilities for performing 3D maths
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

namespace Amantis
{
	class Math3D
	{
	public:
		static Point2d Project(const Mat& cameraMatrix, const Point3d& point);
		static Point3d UnProject(const Mat& cameraMatrix, const Point2d& point, double Z);
		static Point3d TransformPoint(const Mat& pose, const Point3d & point);
		static double ExtractDepth(Mat& depthMap, const Point2d & position);
		static Vec3i ExtractColor(Mat& colorMap, const Point2d& position);
		static Mat ExtractK(Mat& Q);
		static void GetViewLimits(Mat& cameraMatrix, const Size& imageSize, double zmin, double zmax, vector<Point3d>& output);
		static void TransformPointSet(const Mat& pose, vector<Point3d>& input, vector<Point3d>& output);
		static double FindDisparity(const Mat& Q, double Z, bool roundUp);
	};
}
