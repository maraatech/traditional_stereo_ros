//--------------------------------------------------
// Some general utilities for computing
//
// @author: Wild Boar
//--------------------------------------------------

#pragma once

#include <iomanip>
#include <sstream>
#include <chrono>
#include <iostream>
using namespace std;

namespace Amantis 
{
	class GeneralUtils  
	{
		public:
			static string GetTimeString();
			static string CompletePath(const string& path);
	};
}
