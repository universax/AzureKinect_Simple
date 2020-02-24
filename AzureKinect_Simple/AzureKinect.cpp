#include "AzureKinect.hpp"

AzureKinect::AzureKinect()
	:_kinect_enable(false), _device_index(0), _is_recording(false)
{
}

AzureKinect::~AzureKinect()
{
	close();
}

bool AzureKinect::init(std::string filename)
{
    _playback_mode = (filename != "");
    if (_playback_mode)
    {
        _kinect_enable = init_recorded_file(filename);
    }
    else {
        _kinect_enable = init_sensor();
    }
    return _kinect_enable;
}

bool AzureKinect::update()
{
    if (update_capture()) {
        update_body();
        update_color();
        update_recording();
        return update_depth();
    }
    return false;
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
    if (_body_frame != NULL) {
        k4abt_frame_release(_body_frame);
    }
    if (_capture != NULL)
    {
        k4a_capture_release(_capture);
    }
    if (_body_input_capture != NULL)
    {
        k4a_capture_release(_body_input_capture);
    }
    if (_body_index_map != NULL)
    {
        k4a_image_release(_body_index_map);
    }
    if (_pointcloud_image != NULL)
    {
        k4a_image_release(_pointcloud_image);
    }
    
}

void AzureKinect::close()
{
    if (_playback)
    {
        k4a_playback_close(_playback);
    }
    else {
        k4a_device_stop_cameras(_device);
        k4a_device_close(_device);
    }
    
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

uint8_t* AzureKinect::get_body_image_buf()
{
    return _body_image_buf;
}

int AzureKinect::get_body_image_width()
{
    return _body_image_width;
}

int AzureKinect::get_body_image_height()
{
    return _body_image_height;
}

uint8_t* AzureKinect::get_pointcloud_buf()
{
    return _pointcloud_buf;
}


bool AzureKinect::init_sensor()
{
    // Get Sensor Count
    const uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        throw k4a::error("Failed to found device!");
        return false;
    }

    // Open
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &_device)))
    {
        throw k4a::error("Failed to open k4a device!");
        return false;
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
    _device_configuration.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    _device_configuration.synchronized_images_only = true;
    _device_configuration.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    

    // Device Setting
    //k4a_device_set_color_control(_device, K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY, K4A_COLOR_CONTROL_MODE_MANUAL, 1);

    // Start Camera
    if (K4A_FAILED(k4a_device_start_cameras(_device, &_device_configuration)))
    {
        close();
        throw k4a::error("Failed to start camera!");
        return false;
    }

    // Get Calibration
    if (K4A_FAILED(k4a_device_get_calibration(_device, _device_configuration.depth_mode, _device_configuration.color_resolution, &_calibration)))
    {
        close();
        throw k4a::error("Failed to get capture!");
        return false;
    }

    // Get Transform
    _transformation = k4a_transformation_create(&_calibration);

    // Get Tracker
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    if (K4A_FAILED(k4abt_tracker_create(&_calibration, tracker_config, &_tracker)))
    {
        close();
        throw k4a::error("Failed to get tracker!");
        return false;
    }

    printf("Success init device\n");
    return true;
}

bool AzureKinect::init_recorded_file(std::string filename)
{
    if (k4a_playback_open(filename.c_str(), &_playback) == K4A_RESULT_SUCCEEDED)
    {
        // Get Calibration
        if (K4A_FAILED(k4a_playback_get_calibration(_playback, &_calibration)))
        {
            close();
            throw k4a::error("Failed to get capture!");
            return false;
        }

        // Get Transform
        _transformation = k4a_transformation_create(&_calibration);

        // Get Tracker
        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        if (K4A_FAILED(k4abt_tracker_create(&_calibration, tracker_config, &_tracker)))
        {
            close();
            throw k4a::error("Failed to get tracker!");
            return false;
        }

        printf("Success init playback\n");

        return true;
    }
    else {
        throw k4a::error("Failed load playback!");
        return false;
    }

    return false;
}

bool AzureKinect::update_capture()
{
    bool is_frame_new = false;
    if (_playback_mode)
    {
        switch (k4a_playback_get_next_capture(_playback, &_capture))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            is_frame_new = true;
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            //throw k4a::error("Timed out waiting for a capture!");
            printf("Timed out waiting for a capture!");
            break;
        case K4A_WAIT_RESULT_FAILED:
            //throw k4a::error("Failed to get capture!");
            printf("Failed to get capture!");
            break;
        default:
            break;
        }
    }
    else {
        // Get Capture Frame
        switch (k4a_device_get_capture(_device, &_capture, K4A_WAIT_INFINITE))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            //printf("Success to read a capture\n");
            // Recording
            is_frame_new = true;
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            close();
            throw k4a::error("Timed out waiting for a capture!");
            break;
        case K4A_WAIT_RESULT_FAILED:
            close();
            throw k4a::error("Failed to get capture!");
            break;
        }
    }
    

    return is_frame_new;
}

bool AzureKinect::update_depth()
{
    bool result = false;

    // Access the depth16 image
    k4a_image_t depth_raw_image;
    depth_raw_image = k4a_capture_get_depth_image(_capture);
    if (depth_raw_image != NULL) {
        // Transform to color camera
        //k4a_image_t transformed = NULL;
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, 1920, 1080, 1920 * sizeof(uint16_t), &_depth_image);
        if (k4a_transformation_depth_image_to_color_camera(_transformation, depth_raw_image, _depth_image) == K4A_RESULT_SUCCEEDED)
        {
            if (_depth_image != NULL)
            {
                // buf
                _depth_buf = k4a_image_get_buffer(_depth_image);
                // Size
                _depth_width = k4a_image_get_width_pixels(_depth_image);
                _depth_height = k4a_image_get_height_pixels(_depth_image);

                // point cloud
                k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, 1920, 1080, 1920 * 6, &_pointcloud_image);
                if (k4a_transformation_depth_image_to_point_cloud(_transformation, _depth_image, K4A_CALIBRATION_TYPE_COLOR, _pointcloud_image) == K4A_RESULT_SUCCEEDED)
                {
                    // buf
                    _pointcloud_buf = k4a_image_get_buffer(_pointcloud_image);
                    result = true;
                }
            }
            else {
                printf("depth image is NULL\n");
            }
        }
        //k4a_image_release(transformed);
        k4a_image_release(depth_raw_image);
    }
    else {
        printf("raw depth image is NULL\n");
    }

    return result;
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

void AzureKinect::update_body()
{
    // Tracking
    //k4abt_frame_t _body_frame = NULL;
    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(_tracker, _capture, K4A_WAIT_INFINITE);
    if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
    {
        throw k4a::error("Failed to capture tracker!");
        //printf("");
    }
    else if (queue_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(_tracker, &_body_frame, K4A_WAIT_INFINITE);
        if (pop_frame_result == K4A_WAIT_RESULT_FAILED)
        {
            throw k4a::error("Failed to pop tracker!");
        }
        else if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            size_t num_bodies = k4abt_frame_get_num_bodies(_body_frame);

            // Get capture
            _body_input_capture = k4abt_frame_get_capture(_body_frame);

            // Map
            _body_index_map = k4abt_frame_get_body_index_map(_body_frame);

            // buf
            _body_image_buf = k4a_image_get_buffer(_body_index_map);

            // Size
            _body_image_width = k4a_image_get_width_pixels(_body_index_map);
            _body_image_height = k4a_image_get_height_pixels(_body_index_map);
        }
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
