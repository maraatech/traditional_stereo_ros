//--------------------------------------------------
// The parameters associated with rectification.
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
	class RectificationParameters
	{
	private:
		Mat _R1;
		Mat _R2;
		Mat _P1;
		Mat _P2;
		Mat _Q;
	public:
		RectificationParameters(Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q) : _R1(R1), _R2(R2), _P1(P1), _P2(P2), _Q(Q) {}

		inline Mat& GetR1() { return _R1; }
		inline Mat& GetR2() { return _R2; }
		inline Mat& GetP1() { return _P1; }
		inline Mat& GetP2() { return _P2; }
		inline Mat& GetQ() { return _Q; }
	};
}
