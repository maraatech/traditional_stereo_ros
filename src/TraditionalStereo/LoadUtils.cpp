//--------------------------------------------------
// Implementation code for LoadUtils
//
// @author: Wild Boar
//--------------------------------------------------

#include "LoadUtils.h"
using namespace Amantis;

//--------------------------------------------------
// LoadCalibration
//--------------------------------------------------

/**
 * Load calibration from disk
 * @param path The path to the calibration file
 * @return resulting intrinsics
 */
Calibration* LoadUtils::LoadCalibration(const string& path,double ratio) 
{
	std::cout<<"load path is "<<path<<std::endl;
	std::cout<<"CALIBRATION PATH "<<CALIBRATION_PATH<<" "<<int(path.find(CALIBRATION_PATH))<<std::endl;
	int mode = path.find(CALIBRATION_PATH)<0?(FileStorage::READ | FileStorage::FORMAT_JSON):(FileStorage::READ | FileStorage::FORMAT_XML);
	auto reader = FileStorage(path, mode);
	Mat K1,K2,D1,D2,R,T; 
	if( int(path.find(CALIBRATION_PATH))<0){
		std::cout<<"reading json params"<<std::endl;
		std::vector<std::vector<double>> v_k1,v_k2,v_r,v_t,v_d1,v_d2;
		reader["cameras"]["cam1"]["K"] >> v_k1;
		vector2Mat(v_k1,K1);
		K1 = K1*ratio;
		((double *)K1.data)[8] = 1.0;
		std::cout<<K1<<std::endl;

		
		reader["cameras"]["cam2"]["K"] >> v_k2;
		vector2Mat(v_k2,K2);
		K2 = K2*ratio;
		((double *)K2.data)[8] = 1.0;
		
		reader["cameras"]["cam1"]["dist"] >> v_d1;
        	vector2Mat(v_d1,D1);

		reader["cameras"]["cam2"]["dist"] >> v_d2;
		vector2Mat(v_d2,D2);
	
		reader["extrinsics"]["cam2"]["R"] >> v_r;
        	vector2Mat(v_r,R);

		reader["extrinsics"]["cam2"]["T"] >> v_t;
        	vector2Mat(v_t,T);
			T= T*1000.0;

		vector<int> size; reader["cameras"]["cam1"]["image_size"] >> size;

        Size imageSize(size[0]*ratio,size[1]*ratio);
		std::cout<<imageSize<<std::endl;
		std::cout<<K1<<K2<<D1<<D2<<T<<R<<std::endl;
		return new Calibration(K1, K2, D1, D2, R, T, imageSize);
	}else{
		std::cout<<"reading params"<<std::endl;
		reader["Camera1"] >> K1;
		reader["Camera2"] >> K2;
		reader["Distortion1"] >> D1;
        	reader["Distortion2"] >> D2;
		reader["Rotation"] >> R;
        	reader["Translation"] >> T;
        	Size imageSize; reader["ImageSize"] >> imageSize;
		return new Calibration(K1, K2, D1, D2, R, T, imageSize);
	}
}

void LoadUtils::vector2Mat(std::vector<std::vector<double>>& list, Mat& out){
	for(int i=0; i<list.size(); ++i)
        	out.push_back(Mat(list[i]).t());
	return;
}

void LoadUtils::vector2Mat(std::vector<double>& list, Mat& out){
	out.push_back(Mat(list).t());
	return;
}

Calibration * LoadUtils::LoadCalibration(const cares_msgs::StereoCameraInfo ci,double ratio){
	Size imageSize(ci.left_info.width*ratio,ci.left_info.width*ratio);
	cv::Mat K1_tmp(3, 3, CV_64FC1, (void *) ci.left_info.K.data());
	cv::Mat K1 = K1_tmp.clone(); 
	K1 = K1*ratio;
	((double *)K1.data)[8] = 1.0;
	cv::Mat K2_tmp(3, 3, CV_64FC1, (void *) ci.right_info.K.data());
	cv::Mat K2 = K2_tmp.clone();
	K2 = K2*ratio;
	((double *)K2.data)[8] = 1.0;
	cv::Mat D1_tmp(1, 5, CV_64FC1, (void *) ci.left_info.D.data());
	cv::Mat D1 = D1_tmp.clone();
	cv::Mat D2_tmp(1, 5, CV_64FC1, (void *) ci.right_info.D.data());
	cv::Mat D2 = D2_tmp.clone();
	cv::Mat T_tmp(3, 1, CV_64FC1, (void *) ci.T_left_right.data());
	cv::Mat T = T_tmp.clone();
	T= T*1000.0;
	cv::Mat R_tmp(3, 3, CV_64FC1, (void *) ci.R_left_right.data());
	cv::Mat R = R_tmp.clone();
	// R= R*-1.0;
	// ((double *)R.data)[0] = ((double *)R.data)[0] * -1.0;
	// ((double *)R.data)[4] = ((double *)R.data)[4] * -1.0;
	// ((double *)R.data)[8] = ((double *)R.data)[8] * -1.0;
	std::cout<<K1<<K2<<D1<<D2<<T<<R<<std::endl;
	return new Calibration(K1, K2, D1, D2, R, T, imageSize);
}