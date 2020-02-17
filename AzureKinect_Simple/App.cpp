#include "App.hpp"
//CV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

App::App(bool running)
	:_running(running)
{
}

void App::init() {
	
	kinect.init();

	_running = true;
}

void App::run() {
	while (_running)
	{
		// Kinect
		kinect.update();

		// OCV
		// Color
		int color_rows = kinect.get_color_height();
		int color_cols = kinect.get_color_width();
		uint8_t* color_buf = kinect.get_color_image_buf();
		cv::Mat color_mat(color_rows, color_cols, CV_8UC4, (void*)color_buf, cv::Mat::AUTO_STEP);
		cv::resize(color_mat, color_mat, cv::Size(color_cols / 2, color_rows / 2));
		cv::imshow("color", color_mat);
		
		// Depth
		int depth_rows = kinect.get_depth_height();
		int depth_cols = kinect.get_depth_width();
		uint8_t* depth_buf = kinect.get_depth_image_buf();
		cv::Mat depth_mat(depth_rows, depth_cols, CV_16U, (void*)depth_buf, cv::Mat::AUTO_STEP);
		depth_mat *= 10;
		cv::imshow("depth", depth_mat);

		handleKey((char)cv::waitKey(1));
	}
}

void App::handleKey(char key)
{
    switch (key) {
    case 27:
        _running = false;
        break;
    default:
        break;
    }
}
