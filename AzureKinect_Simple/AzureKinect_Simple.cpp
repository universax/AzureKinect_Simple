// AzureKinect_Simple.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <sstream>

#include "App.hpp"

int main(int argc, char** argv)
{
	try {
		App app;
		app.init();
		app.run();
	}
	catch (std::exception & ex)
	{
		std::cout << ex.what() << std::endl;
		return -1;
	}
	return 0;
}