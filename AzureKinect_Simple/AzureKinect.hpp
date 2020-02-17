#pragma once

#include <k4a/k4a.hpp>
#include <k4arecord/record.h>

class AzureKinect
{
private:
	// Kinect
    k4a_device_t _device;
    k4a_capture_t _capture;
    k4a_calibration_t  _calibration;
    k4a_transformation_t _transformation;
    k4a_device_configuration_t _device_configuration;
    uint32_t _device_index;
    k4a_record_t _recording;

    // Color
    k4a_image_t _color_image;
    uint8_t* _color_buf;
    int _color_width;
    int _color_height;

    // Depth
    k4a_image_t _depth_image;
    uint8_t* _depth_buf;
    int _depth_width;
    int _depth_height;

    // Recording
    bool _is_recording;
    

public:
    AzureKinect();
    ~AzureKinect();

    void init();
    void update();
    void clear();
    void close();
    void start_recording();
    void stop_recording();

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
    bool update_capture();
    void update_depth();
    void update_color();

    bool setup_recording();
    void update_recording();

    std::string get_timestamp();
};

