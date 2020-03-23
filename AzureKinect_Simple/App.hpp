#pragma once
#include "AzureKinect.hpp"
//CV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

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

};

