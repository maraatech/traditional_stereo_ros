//--------------------------------------------------
// Save points to a PLY file
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <fstream>
#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "ColorPoint.h"

namespace Amantis
{
	class PointSaver
	{
	private:
		vector<ColorPoint> _points;
	public:
		PointSaver();

		void AddPointSet(vector<ColorPoint> & points);
		void Save(const string & path);	
	private:
		static void RenderPLYHeader(ofstream& writer, int pointCount);
		static void RenderPLYBody(ofstream& writer, vector<ColorPoint>& points);
	};
}
