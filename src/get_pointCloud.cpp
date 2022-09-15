#include <k4a/k4a.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    uint32_t count = k4a_device_get_installed_count();
    if(count == 0){
	cout << "No k4a device attached!" << endl;
	return 1;
    }
    k4a_device_t device = NULL;
    k4a_device_open(K4A_DEVICE_DEFAULT, &device);

    // configuration of the camera
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps		= K4A_FRAMES_PER_SECOND_30;
    config.color_format		= K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution	= K4A_COLOR_RESOLUTION_720P;
    config.depth_mode		= K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    //start camera
    if(K4A_FAILED(k4a_device_start_cameras(device, &config))){
	cout << "Failed to start cameras" << endl;
	k4a_device_close(device);
	return 1;
    }

    // create a capture
    k4a_capture_t capture;
    namedWindow("depth_color", WINDOW_NORMAL);
    namedWindow("depth_capture", WINDOW_NORMAL);
    namedWindow("color", WINDOW_NORMAL);


    // get frames
    while(true){
	switch(k4a_device_get_capture(device, &capture, 100)){
	    case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	    case K4A_WAIT_RESULT_TIMEOUT:
		continue;
	    case K4A_WAIT_RESULT_FAILED:
		break;
	};
	
	// get transformation handle
	cout << "start get the transformation handle" << endl;
	k4a_transformation_t transformation_handle;
	k4a_calibration_t calibration;
	k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration);
	transformation_handle = k4a_transformation_create(&calibration);
	cout << "success get the transformation handle" << endl;
	
	// get depth image color
	cout << "start get depth_image_color" << endl;
	k4a_image_t depth_image_capture = k4a_capture_get_depth_image(capture);
	k4a_image_t color_image_capture = k4a_capture_get_color_image(capture);
	k4a_image_t depth_image_color;
	int width_pixels = k4a_image_get_width_pixels(color_image_capture);
	int height_pixels = k4a_image_get_height_pixels(color_image_capture);
	int stride_bytes = 0;
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, width_pixels, height_pixels, stride_bytes, &depth_image_color);
	k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image_capture, depth_image_color);
	cout << "success get depth_image_color" << endl;

	// get the calibration type
	k4a_calibration_type_t camera = K4A_CALIBRATION_TYPE_COLOR;

	// get the output: xyz_image
	
	cout << "start get the xyz_image" << endl;
	k4a_image_t xyz_image;
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, width_pixels, height_pixels, 6*width_pixels, &xyz_image);
	k4a_transformation_depth_image_to_point_cloud(transformation_handle, depth_image_color, camera, xyz_image);
	cout << "success get the xyz_image" << endl;
	
	// show depth_image_color and color_image_capture
	if(color_image_capture != NULL){
	    uint8_t* buffer_color = k4a_image_get_buffer(color_image_capture);
	    cv::Mat colorMat(height_pixels, width_pixels, CV_8UC4, (void*)buffer_color, cv::Mat::AUTO_STEP);
	    imshow("color", colorMat);
	}
	
	if(depth_image_color != NULL){
	    uint8_t* buffer_depth_color = k4a_image_get_buffer(depth_image_color);
	    cv::Mat depthColorMat(height_pixels, width_pixels, CV_16U, (void*)buffer_depth_color, cv::Mat::AUTO_STEP);
	    imshow("depth_color", depthColorMat);
	}

/***	if(depth_image_capture != NULL){
	    uint8_t* buffer_depth_capture = k4a_image_get_buffer(depth_image_capture);
	    cv::Mat depthCaptureMat(height_pixels, width_pixels, CV_16U, (void*)buffer_depth_capture, cv::Mat::AUTO_STEP);
	    imshow("depth_capture", depthCaptureMat);
	}
	***/
	
	k4a_transformation_destroy(transformation_handle);

	k4a_image_release(depth_image_capture);
	k4a_image_release(color_image_capture);
	k4a_image_release(depth_image_color);
	k4a_image_release(xyz_image);
	k4a_capture_release(capture);

	if(waitKey(10) == 27)
	    break;
    }

    destroyWindow("color");
    destroyWindow("depth_color");
    destroyWindow("depth_capture");
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    return 0;
    
}
