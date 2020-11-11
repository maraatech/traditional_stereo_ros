//--------------------------------------------------
// Defines a stereo frame (a stereo pair of images)
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
	class StereoFrame
	{
	private:
		Mat _image1;
		Mat _image2;
	public:
		StereoFrame(Mat& image1, Mat& image2) : _image1(image1), _image2(image2) { ValidateSize(); }
		StereoFrame(StereoFrame && frame) { _image1 = frame._image1; _image2 = frame._image2;  } // Move Constructor

		inline Mat& Image1() { return _image1; }
		inline Mat& Image2() { return _image2; }
		inline int Width() { return _image1.cols; }
		inline int Height() { return _image1.rows; }
		inline bool Exists() { return !_image1.empty() && !_image2.empty(); }

	private:
		inline void ValidateSize() { if (_image1.rows != _image2.rows || _image1.cols != _image2.cols) throw string("Stereo Frames are not the same size!"); }
	};
}
