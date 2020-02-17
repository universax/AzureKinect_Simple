#include "AzureKinect.hpp"

AzureKinect::AzureKinect(const uint32_t index)
	: _device_index(index)
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
    update_capture();
    update_depth();
    update_color();
}

void AzureKinect::close()
{
    k4a_image_release(_depth_image);
    //k4a_image_release(_color_image);
    k4a_device_stop_cameras(_device);
    k4a_device_close(_device);
}

uint8_t* AzureKinect::get_depth_image_buf()
{
    if (_depth_image != NULL)
    {
        // for check
        k4a_image_format_t format = k4a_image_get_format(_depth_image);

        // buf
        uint8_t* buf = k4a_image_get_buffer(_depth_image);
        return buf;
    }
    
    return nullptr;
}

int AzureKinect::get_depth_width()
{
    if (_depth_image != NULL)
    {
        return k4a_image_get_width_pixels(_depth_image);
    }
    return 0;
}

int AzureKinect::get_depth_height()
{
    if (_depth_image != NULL)
    {
        return k4a_image_get_height_pixels(_depth_image);
    }
    return 0;
}

uint8_t* AzureKinect::get_color_image_buf()
{
    if (_color_image != NULL)
    {
        // for check
        k4a_image_format_t format = k4a_image_get_format(_color_image);

        // buf
        uint8_t* buf = k4a_image_get_buffer(_color_image);
        return buf;
    }
    return nullptr;
}

int AzureKinect::get_color_width()
{
    if (_color_image != NULL)
    {
        return k4a_image_get_width_pixels(_color_image);
    }
    return 0;
}

int AzureKinect::get_color_height()
{
    if (_color_image != NULL)
    {
        return k4a_image_get_height_pixels(_color_image);
    }
    return 0;
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
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

    // Start Camera
    if (K4A_FAILED(k4a_device_start_cameras(_device, &config)))
    {
        k4a_device_stop_cameras(_device);
        k4a_device_close(_device);
        throw k4a::error("Failed to start camera!");
    }

    // Get Calibration
    if (K4A_FAILED(k4a_device_get_calibration(_device, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &_calibration)))
    {
        k4a_device_stop_cameras(_device);
        k4a_device_close(_device);
        throw k4a::error("Failed to get capture!");
    }


    // Get Transform
    _transformation = k4a_transformation_create(&_calibration);

}

void AzureKinect::update_capture()
{
    // Get Capture Frame
    switch (k4a_device_get_capture(_device, &_capture, 10000))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        //printf("Success to read a capture\n");
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
    }
}

void AzureKinect::update_depth()
{
    // Access the depth16 image
    _depth_image = k4a_capture_get_depth_image(_capture);
    if (_depth_image != NULL) {
        //printf(" | Depth16 res:%4dx%4d stride:%5d\n",
        //    k4a_image_get_height_pixels(_depth_image),
        //    k4a_image_get_width_pixels(_depth_image),
        //    k4a_image_get_stride_bytes(_depth_image)
        //);
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
        //printf(" | Color res:%4dx%4d stride:%5d\n",
        //    k4a_image_get_height_pixels(_color_image),
        //    k4a_image_get_width_pixels(_color_image),
        //    k4a_image_get_stride_bytes(_color_image)
        //);
    }
    else {
        printf("color image is NULL\n");
    }
}
