//--------------------------------------------------------
// Defines a depth frame within the system
//
// @author: Wild Boar
//--------------------------------------------------------

#pragma once

#include <cassert>
#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

class DepthFrame
{
private:
	Mat _color;
	Mat _depth;
public:
	DepthFrame(Mat& color, Mat& depth) : _color(color), _depth(depth) {}
	DepthFrame(DepthFrame && frame) { _color = frame._color; _depth = frame._depth; } // Move Constructor

	inline Mat& Color() { return _color; }
	inline Mat& Depth() { return _depth; }
	inline int Width() { return _color.cols; }
	inline int Height() { return _color.rows; }
	inline Size2i Size() { return _color.size(); }
	inline bool IsEmpty() { return _color.empty() || _depth.empty(); }
};