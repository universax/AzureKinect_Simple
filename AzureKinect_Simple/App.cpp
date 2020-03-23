#include "App.hpp"

App::App(bool running)
	:_running(running)
{
}

void App::init() {
	std::string filename = "C:/Users/unive/Develop/kinect_rec_20200304023933.mkv";
	//std::string filename = "";
	_running = kinect.init(filename);
}

void App::run() {

	cv::VideoCapture vid(1);
	if (!vid.isOpened())
	{
		printf("Fuck \n");
	}

	while (_running)
	{
		// Kinect
		if (kinect.update()) {
			// OCV
			update_body();
			update_depth();
			update_color();
			update_pointcloud();

			kinect.clear();
		}

		handleKey((char)cv::waitKey(1));
	}
}

cv::Mat App::update_depth()
{
	
	// Depth
	int depth_rows = kinect.get_depth_height();
	int depth_cols = kinect.get_depth_width();
	cv::Mat result_mat = cv::Mat::zeros(depth_rows, depth_cols, CV_16U);

	if (depth_rows * depth_cols > 0)
	{
		uint8_t* depth_buf = kinect.get_depth_image_buf();
		if (!depth_buf)
		{
			return result_mat;
		}
		cv::Mat depth_mat(depth_rows, depth_cols, CV_16U, (void*)depth_buf, cv::Mat::AUTO_STEP);
		result_mat = depth_mat;

		// Show
		cv::resize(depth_mat, depth_mat, cv::Size(depth_cols / 2, depth_rows / 2));
		depth_mat *= 20;
		cv::imshow("depth", depth_mat);
	}

	return result_mat;
}

cv::Mat App::update_color()
{
	// Color (if color mode is BGRA, this process is possible) 
	int color_rows = kinect.get_color_height();
	int color_cols = kinect.get_color_width(); 
	cv::Mat result_mat = cv::Mat::zeros(color_rows, color_cols, CV_8UC1);

	uint8_t* color_buf = kinect.get_color_image_buf();

	//cv::Mat color_mat(color_rows, color_cols, CV_8UC1, (void*)color_buf, cv::Mat::AUTO_STEP);
	//result_mat = color_mat;
	//if (!color_mat.empty())
	//{
	//	//cv::resize(color_mat, color_mat, cv::Size(color_cols / 2, color_rows / 2));
	//	cv::imshow("color", color_mat);
	//}
	
	
	

	return result_mat;
}

cv::Mat App::update_body()
{
	// Body
	int body_image_rows = kinect.get_body_image_height();
	int body_image_cols = kinect.get_body_image_width();
	cv::Mat result_mat = cv::Mat::zeros(body_image_rows, body_image_cols, CV_8U);

	if (body_image_rows * body_image_cols > 0)
	{
		uint8_t* body_image_buf = kinect.get_body_image_buf();
		cv::Mat body_image_mat(body_image_rows, body_image_cols, CV_8U, (void*)body_image_buf, cv::Mat::AUTO_STEP);
		result_mat = body_image_mat;

		cv::imshow("body", body_image_mat);
	}

	return result_mat;
}

cv::Mat App::update_pointcloud()
{
	// Pointcloud
	int pointcloud_rows = kinect.get_color_height();
	int pointcloud_cols = kinect.get_color_width();
	cv::Mat result_mat = cv::Mat::zeros(pointcloud_rows, pointcloud_cols, CV_16SC3);

	if (pointcloud_rows * pointcloud_cols > 0)
	{
		uint8_t* pointcloud_buf = kinect.get_pointcloud_buf();
		cv::Mat pointcloud_mat(pointcloud_rows, pointcloud_cols, CV_16SC3, (void*)pointcloud_buf, cv::Mat::AUTO_STEP);
		result_mat = pointcloud_mat;

		cv::resize(pointcloud_mat, pointcloud_mat, cv::Size(pointcloud_cols / 2, pointcloud_rows / 2));
		cv::imshow("pointcloud", pointcloud_mat);
	}

	return result_mat;
}

void App::handleKey(char key)
{
    switch (key) {
    case 27:
        _running = false;
        break;
	case 'r':
		kinect.start_recording();
		break;
	case 's':
		kinect.stop_recording();
		break;
    default:
        break;
    }
}
