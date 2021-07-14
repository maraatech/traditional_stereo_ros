//--------------------------------------------------
// Defines a 3D point with a color attribute
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "Math3D.h"

namespace Amantis
{
	class ColorPoint
	{
	private:
		Vec3i _color;
		Point3d _location;
	public:
		ColorPoint(const Vec3i& color, const Point3d& location) : _color(color), _location(location) {}

		inline Vec3i& GetColor() { return _color; }
		inline Point3d& GetLocation() { return _location; }

		inline void Transform(Mat& pose) { _location = Math3D::TransformPoint(pose, _location); }
	};
}
