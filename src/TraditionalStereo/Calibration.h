//--------------------------------------------------
// The stereo calibration values
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
	class Calibration
	{
	private:
		Mat _camera1;
		Mat _camera2;
		Mat _distortion1;
		Mat _distortion2;
		Mat _rotation;
		Mat _translation;
		Size _imageSize;
	public:
		Calibration(Mat& camera1, Mat& camera2, Mat& distortion1, Mat& distortion2, Mat& rotation, Mat& translation, Size& imageSize) :
			_camera1(camera1), _camera2(camera2), _distortion1(distortion1), _distortion2(distortion2), _rotation(rotation), _translation(translation), _imageSize(imageSize) {}

		inline Mat& GetCamera1() { return _camera1; }
		inline Mat& GetCamera2() { return _camera2; }
		inline Mat& GetDistortion1() { return _distortion1; }
		inline Mat& GetDistortion2() { return _distortion2; }
		inline Mat& GetRotation() { return _rotation; }
		inline Mat& GetTranslation() { return _translation; }
		inline Size& GetImageSize() { return _imageSize; }

		inline bool LoadSuccess() { return !_camera1.empty() && !_camera2.empty() && !_distortion1.empty() && !_distortion2.empty() && !_rotation.empty() && !_translation.empty(); }
		inline String toString(){ stringstream ss; ss<<_camera1<<_camera2<<_distortion1<<_distortion2<<_translation<<_rotation;return ss.str();} 	
	};
}
