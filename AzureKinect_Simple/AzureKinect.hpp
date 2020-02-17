#pragma once

#include <k4a/k4a.hpp>

class AzureKinect
{
private:
	// Kinect
    k4a_device_t _device;
    k4a_capture_t _capture;
    k4a_calibration_t  _calibration;
    k4a_transformation_t _transformation;
    const k4a_device_configuration_t _device_configuration;
    uint32_t _device_index;

    // Color
    k4a_image_t _color_image;

    // Depth
    k4a_image_t _depth_image;

public:
    AzureKinect(const uint32_t index = K4A_DEVICE_DEFAULT);
    ~AzureKinect();

    void init();
    void update();
    void close();

    // Depth
    uint8_t* get_depth_image_buf();
    int get_depth_width();
    int get_depth_height();

    // Color
    uint8_t* get_color_image_buf();
    int get_color_width();
    int get_color_height();

private:
    void init_sensor();
    void update_capture();
    void update_depth();
    void update_color();
};

