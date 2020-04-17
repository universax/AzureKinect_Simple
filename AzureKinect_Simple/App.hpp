#pragma once
#include "AzureKinect.hpp"
//CV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include "Sensor.h"
#include "PCL_Functions.h"

class App
{
public:
	App(bool running = false);
	~App() {}

	void init();
	void run();

private:
	// Kinect
	AzureKinect kinect;
	cv::Mat update_depth();
	cv::Mat update_color();
	cv::Mat update_body();
	cv::Mat update_pointcloud();

	// Event
	bool _running;
	void handleKey(char key);

	// PCL
	Sensor& _sensor = Sensor::GetInstance();
	pcl_func::PCL_Functions pcl;

	pcl::PointCloud<PointType>::Ptr inputPoits;
	pcl::PointCloud<PointType>::Ptr convert_to_pointcloud(cv::Mat& in_pointcloud_mat, cv::Mat& in_color_mat);
};

