#include "AzureKinect.hpp"

AzureKinect::AzureKinect()
	: _device_index(0), _is_recording(false)
{
}

AzureKinect::~AzureKinect()
{
	close();
}

void AzureKinect::init()
{
	init_sensor();
}

void AzureKinect::update()
{
    if (update_capture()) {
        update_depth();
        //update_color();
        update_recording();
    }
}

void AzureKinect::clear()
{
    // Clear
    if (_color_image != NULL)
    {
        k4a_image_release(_color_image);
    }
    if (_depth_image != NULL)
    {
        k4a_image_release(_depth_image);
    }
    if (_capture != NULL)
    {
        k4a_capture_release(_capture);
    }
}

void AzureKinect::close()
{
    k4a_device_stop_cameras(_device);
    k4a_device_close(_device);
}

void AzureKinect::start_recording()
{
    _is_recording = setup_recording();
}

void AzureKinect::stop_recording()
{
    _is_recording = false;

    printf("Saving recording...\n");
    if (K4A_FAILED(k4a_record_flush(_recording)))
    {
        throw k4a::error("Failed to flush recording");
    }
    k4a_record_close(_recording);
    printf("Recording Done...\n");
}

uint8_t* AzureKinect::get_depth_image_buf()
{
    return _depth_buf;
}

int AzureKinect::get_depth_width()
{
    return _depth_width;
}

int AzureKinect::get_depth_height()
{
    return _depth_height;
}

uint8_t* AzureKinect::get_color_image_buf()
{
    return _color_buf;
}

int AzureKinect::get_color_width()
{
    return _color_width;
}

int AzureKinect::get_color_height()
{
    return _color_height;
}


void AzureKinect::init_sensor()
{
    // Get Sensor Count
    const uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        throw k4a::error("Failed to found device!");
    }

    // Open
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &_device)))
    {
        throw k4a::error("Failed to open k4a device!");
    }

    // Device Serial Number
    // Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(_device, NULL, &serial_size);
    // Allocate memory for the serial, then acquire it
    char* serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(_device, serial, &serial_size);
    printf("Opened device: %s\n", serial);
    free(serial);

    // Configure
    _device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    _device_configuration.camera_fps = K4A_FRAMES_PER_SECOND_30;
    _device_configuration.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    _device_configuration.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    _device_configuration.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    _device_configuration.synchronized_images_only = true;
    _device_configuration.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    

    // Device Setting
    //k4a_device_set_color_control(_device, K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY, K4A_COLOR_CONTROL_MODE_MANUAL, 1);

    // Start Camera
    if (K4A_FAILED(k4a_device_start_cameras(_device, &_device_configuration)))
    {
        k4a_device_stop_cameras(_device);
        k4a_device_close(_device);
        throw k4a::error("Failed to start camera!");
    }

    //// Get Calibration
    //if (K4A_FAILED(k4a_device_get_calibration(_device, _device_configuration.depth_mode, _device_configuration.color_resolution, &_calibration)))
    //{
    //    k4a_device_stop_cameras(_device);
    //    k4a_device_close(_device);
    //    throw k4a::error("Failed to get capture!");
    //}


    //// Get Transform
    //_transformation = k4a_transformation_create(&_calibration);

}

bool AzureKinect::update_capture()
{
    bool is_frame_new = false;
    // Get Capture Frame
    switch (k4a_device_get_capture(_device, &_capture, K4A_WAIT_INFINITE))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        //printf("Success to read a capture\n");
        // Recording
        is_frame_new = true;
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        k4a_device_stop_cameras(_device);
        k4a_device_close(_device);
        throw k4a::error("Timed out waiting for a capture!");
        break;
    case K4A_WAIT_RESULT_FAILED:
        k4a_device_stop_cameras(_device);
        k4a_device_close(_device);
        throw k4a::error("Failed to get capture!");
        break;
    }

    return is_frame_new;
}

void AzureKinect::update_depth()
{
    // Access the depth16 image
    _depth_image = k4a_capture_get_depth_image(_capture);
    if (_depth_image != NULL) {
        // buf
        _depth_buf = k4a_image_get_buffer(_depth_image);
        // Size
        _depth_width = k4a_image_get_width_pixels(_depth_image);
        _depth_height = k4a_image_get_height_pixels(_depth_image);
    }
    else {
        printf("depth image is NULL\n");
    }
}

void AzureKinect::update_color()
{
    // Access the color image
    _color_image = k4a_capture_get_color_image(_capture);
    if (_color_image != NULL) {
        // buf
        _color_buf = k4a_image_get_buffer(_color_image);
        // size
        _color_width = k4a_image_get_width_pixels(_color_image);
        _color_height = k4a_image_get_height_pixels(_color_image);
    }
    else {
        printf("color image is NULL\n");
    }
}

bool AzureKinect::setup_recording()
{
    std::string recording_filename = "C:/Users/unive/Develop/kinect_rec_" + get_timestamp() + ".mkv";
    printf("Recoding file is %s\n", recording_filename.c_str());
    if (K4A_FAILED(k4a_record_create(recording_filename.c_str(), _device, _device_configuration, &_recording)))
    {
        throw k4a::error("Unable to create recording file: " + recording_filename);
        return false;
    }
    printf("Success create recording\n");


    if (K4A_FAILED(k4a_record_write_header(_recording)))
    {
        throw k4a::error("Unable to write header: " + recording_filename);
        return false;
    }
    printf("Success write header\n");

    return true;
}

void AzureKinect::update_recording()
{
    if (_is_recording)
    {
        if (K4A_FAILED(k4a_record_write_capture(_recording, _capture)))
        {
            throw k4a::error("Failed to write capture");
            stop_recording();
        }
    }
}

std::string AzureKinect::get_timestamp()
{
    time_t t = time(0);
    struct tm now;
    errno_t error;
    error = localtime_s(&now, &t);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d%H%M%S", &now);

    return buffer;
}
