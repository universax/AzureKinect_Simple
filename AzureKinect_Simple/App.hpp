#pragma once
#include "AzureKinect.hpp"
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
	void update_depth();
	void update_color();
	void update_body();
	void update_pointcloud();

	// tmp
	int framecount;

	// Event
	bool _running;
	void handleKey(char key);
};

