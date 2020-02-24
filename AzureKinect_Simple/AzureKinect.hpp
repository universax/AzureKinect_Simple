#pragma once

#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <k4arecord/playback.hpp>
#include <k4arecord/record.h>

class AzureKinect
{
private:
    // Run Flag
    bool _kinect_enable;

    // Mode
    bool _playback_mode;

	// Kinect
    k4a_device_t _device;
    k4a_capture_t _capture;
    k4a_calibration_t  _calibration;
    k4a_transformation_t _transformation;
    k4a_device_configuration_t _device_configuration;
    uint32_t _device_index;

    // Playback
    k4a_playback_t _playback;

    // Recording
    k4a_record_t _recording;

    // Body
    k4abt_tracker_t _tracker;
    k4abt_frame_t _body_frame;
    k4a_capture_t _body_input_capture;
    k4a_image_t _body_index_map;
    uint8_t* _body_image_buf;
    int _body_image_width;
    int _body_image_height;

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

    bool init(std::string filename = "");
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

    // Body Image Map
    uint8_t* get_body_image_buf();
    int get_body_image_width();
    int get_body_image_height();

private:
    bool init_sensor();
    bool init_recorded_file(std::string filename);
    bool update_capture();
    void update_depth();
    void update_color();
    void update_body();

    bool setup_recording();
    void update_recording();

    std::string get_timestamp();
};

