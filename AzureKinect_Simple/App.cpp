#include "App.hpp"
//CV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

App::App(bool running)
	:_running(running), framecount(0)
{
}

void App::init() {
	//std::string filename = "E:/KinectRecord/output-3.mkv";
	std::string filename = "";
	_running = kinect.init(filename);
}

void App::run() {
	while (_running)
	{
		// Kinect
		if (kinect.update()) {
			// OCV
			update_body();
			update_depth();
			update_pointcloud();

			kinect.clear();
		}

		handleKey((char)cv::waitKey(1));
	}
}

void App::update_depth()
{
	// Depth
	int depth_rows = kinect.get_depth_height();
	int depth_cols = kinect.get_depth_width();
	if (depth_rows * depth_cols > 0)
	{
		uint8_t* depth_buf = kinect.get_depth_image_buf();
		if (!depth_buf)
		{
			return;
		}
		cv::Mat depth_mat(depth_rows, depth_cols, CV_16U, (void*)depth_buf, cv::Mat::AUTO_STEP);

		//// Filter with pointcloud
		//uint8_t* pointcloud_buf = kinect.get_pointcloud_buf();
		//if (!pointcloud_buf)
		//{
		//	return;
		//}
		//cv::Mat pointcloud_mat(depth_rows, depth_cols, CV_16SC3, (void*)pointcloud_buf, cv::Mat::AUTO_STEP);
		//for (int y = 0; y < depth_rows; y++)
		//{
		//	for (int x = 0; x < depth_cols; x++)
		//	{
		//		// Points
		//		cv::Vec3s* p_ptr = pointcloud_mat.ptr<cv::Vec3s>(y);
		//		auto p_color = p_ptr[x];
		//		
		//		int p_x = p_color[0];
		//		int p_y = p_color[1];
		//		int p_z = p_color[2];

		//		if (p_y > 440 || p_z > 2000)
		//		{
		//			uint16_t* d_ptr = depth_mat.ptr<uint16_t>(y);
		//			d_ptr[x] = 0;
		//		}
		//	}
		//}

		//// Write to file
		//std::string rec_filename = "E:/KinectRecord/depth_03/depth_" + std::to_string(framecount) + ".png";
		//cv::imwrite(rec_filename, depth_mat, std::vector<int>(0));
		//framecount++;
		//printf("frame: %d\n", framecount);

		// Show
		cv::resize(depth_mat, depth_mat, cv::Size(depth_cols / 2, depth_rows / 2));
		depth_mat *= 20;
		cv::imshow("depth", depth_mat);

	}
}

void App::update_color()
{
	// Color (if color mode is BGRA, this process is possible) 
	//int color_rows = kinect.get_color_height();
	//int color_cols = kinect.get_color_width(); 
	//uint8_t* color_buf = kinect.get_color_image_buf();
	//cv::Mat color_mat(color_rows, color_cols, CV_8UC4, (void*)color_buf, cv::Mat::AUTO_STEP);
	//if (!color_mat.empty())
	//{
	//	cv::resize(color_mat, color_mat, cv::Size(color_cols / 2, color_rows / 2));
	//	cv::imshow("color", color_mat);
	//}
}

void App::update_body()
{
	// Body
	int body_image_rows = kinect.get_body_image_height();
	int body_image_cols = kinect.get_body_image_width();
	if (body_image_rows * body_image_cols > 0)
	{
		uint8_t* body_image_buf = kinect.get_body_image_buf();
		cv::Mat body_image_mat(body_image_rows, body_image_cols, CV_8U, (void*)body_image_buf, cv::Mat::AUTO_STEP);
		//depth_mat *= 20;
		cv::imshow("body", body_image_mat);
	}
}

void App::update_pointcloud()
{
	// Pointcloud
	int pointcloud_rows = kinect.get_color_height();
	int pointcloud_cols = kinect.get_color_width();
	if (pointcloud_rows * pointcloud_cols > 0)
	{
		uint8_t* pointcloud_buf = kinect.get_pointcloud_buf();
		cv::Mat pointcloud_mat(pointcloud_rows, pointcloud_cols, CV_16SC3, (void*)pointcloud_buf, cv::Mat::AUTO_STEP);

		cv::resize(pointcloud_mat, pointcloud_mat, cv::Size(pointcloud_cols / 2, pointcloud_rows / 2));
		cv::imshow("pointcloud", pointcloud_mat);
	}
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
