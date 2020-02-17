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

	// Event
	bool _running;

	void handleKey(char key);
};

