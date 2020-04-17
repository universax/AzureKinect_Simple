#include "App.hpp"

App::App(bool running)
	:_running(running)
{
}

void App::init() {
	//std::string filename = "C:/Users/unive/Develop/kinect_rec_20200304023933.mkv";
	std::string filename = "";
	_running = kinect.init(filename);
}

void App::run() {

	//cv::VideoCapture vid(1);
	//if (!vid.isOpened())
	//{
	//	printf("Fuck \n");
	//}

	//Sensor setup
	_sensor.loadSensorRangeData();
	_sensor.loadSensorPostureData();

	while (_running)
	{
		// Kinect
		if (kinect.update()) {
			// OCV
			//update_body();
			cv::Mat depth_mat = update_depth();
			cv::Mat color_mat = update_color();
			cv::Mat pointcloud_mat = update_pointcloud();


			pcl::PointCloud<PointType>::Ptr kinect_pointcloud = convert_to_pointcloud(pointcloud_mat, color_mat);
			pcl.update(_sensor, kinect_pointcloud);

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
	cv::Mat result_mat = cv::Mat::zeros(color_rows, color_cols, CV_8UC4);

	uint8_t* color_buf = kinect.get_color_image_buf();

	cv::Mat color_mat(color_rows, color_cols, CV_8UC4, (void*)color_buf, cv::Mat::AUTO_STEP);
	result_mat = color_mat;
	if (!color_mat.empty())
	{
		cv::resize(color_mat, color_mat, cv::Size(color_cols / 2, color_rows / 2));
		cv::imshow("color", color_mat);
	}
	
	
	

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
		//pointcloud_mat *= 10.0;
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

pcl::PointCloud<PointType>::Ptr App::convert_to_pointcloud(cv::Mat& in_pointcloud_mat, cv::Mat& in_color_mat)
{
	//cv::resize(in_pointcloud_mat, in_pointcloud_mat, cv::Size(in_pointcloud_mat.size().width / 4, in_pointcloud_mat.size().height / 4));
	//cv::resize(in_color_mat, in_color_mat, cv::Size(in_color_mat.size().width / 4, in_color_mat.size().height / 4));

	pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());


	int pw = in_pointcloud_mat.size().width;
	int ph = in_pointcloud_mat.size().height;

	int cw = in_color_mat.size().width;
	int ch = in_color_mat.size().height;


	// size check
	if (pw != cw || ph != ch) {
		printf("convert_to_pointcloud: pointcloud mat and color mat is not same size");
		return ptr;
	}


	int step = 2;
	for (int y = 0; y < ph; y += step)
	{
		for (int x = 0; x < pw; x += step)
		{
			// Position
			cv::Vec3s* p_ptr = in_pointcloud_mat.ptr<cv::Vec3s>(y);
			auto p_color = p_ptr[x];

			float p_x = p_color[0];
			float p_y = p_color[1];
			float p_z = p_color[2];

			PointType point;
			if (p_z <= 0)
			{
				continue;
			}

			point.x = p_x * 0.001;
			point.y = p_z * 0.001;
			point.z = -p_y * 0.001;

			// Color
			//cv::Vec4s* c_ptr = in_color_mat.ptr<cv::Vec4s>(y);
			//auto c_color = c_ptr[x];

			//int c_r = c_color[0];
			//int c_g = c_color[1];
			//int c_b = c_color[2];
			//int c_a = c_color[3];

			//point.r = c_r;
			//point.g = c_g;
			//point.b = c_b;
			//point.a = c_a;
			

			// Add point
			ptr->points.push_back(point);
		}
	}
	return ptr;
}
