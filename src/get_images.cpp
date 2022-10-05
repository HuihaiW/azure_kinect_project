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

    //Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);
    cout << "serial size is: " << serial_size << endl;

    //Allocate memory for the serial, then acquire it
    char *serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    cout << "serial size is: " << serial_size << endl;
    cout << "Open Device: " << serial << endl;
    free(serial);

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps		= K4A_FRAMES_PER_SECOND_30;
    config.color_format		= K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution	= K4A_COLOR_RESOLUTION_720P;
    config.depth_mode		= K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    k4a_capture_t capture;
    if(K4A_FAILED(k4a_device_start_cameras(device, &config))){
	cout << "Failed to start cameras!" << endl;
	k4a_device_close(device);
	return 1;
    }
    namedWindow("depth", WINDOW_NORMAL);
    namedWindow("color", WINDOW_NORMAL);
    namedWindow("ir", WINDOW_NORMAL);

    while(true){
	switch(k4a_device_get_capture(device, &capture, 100)){
	    case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	    case K4A_WAIT_RESULT_TIMEOUT:
		cout << "Timed out waiting for a capture" << endl;
		continue;
		break;
	    case K4A_WAIT_RESULT_FAILED:
		cout << "Failed to read a capture" << endl;
		break; 
	}
	k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
	k4a_image_t color_image = k4a_capture_get_color_image(capture);
	k4a_image_t ir_image = k4a_capture_get_ir_image(capture);
	
	if(depth_image != NULL){
	    k4a_image_format_t format = k4a_image_get_format(depth_image);
	    uint8_t* buffer = k4a_image_get_buffer(depth_image);
	    int rows = k4a_image_get_height_pixels(depth_image);
	    int cols = k4a_image_get_width_pixels(depth_image);

	    cout << "depth row: " << rows << "; " << " depth cols: " << cols << endl;
	    cv::Mat depthMat(rows, cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);

	    imshow("depth", depthMat);
	}

	if(ir_image != NULL){
	    uint8_t* buffer_ir = k4a_image_get_buffer(ir_image);
	    int rows = k4a_image_get_height_pixels(ir_image);
	    int cols = k4a_image_get_width_pixels(ir_image);

	    cout << "ir row: " << rows << "; " << " ir cols: " << cols << endl;
	    cv::Mat irMat(rows, cols, CV_16U, (void*)buffer_ir, cv::Mat::AUTO_STEP);

	    imshow("ir", irMat);
	    
	}

	cv::Mat color_image_save;

	if(color_image != NULL){
	    uint8_t* buffer_color = k4a_image_get_buffer(color_image);

	    int rows = k4a_image_get_height_pixels(color_image);
	    int cols = k4a_image_get_width_pixels(color_image);
	    
	    cout << "color row: " << rows << "; " << " color cols: " << cols << endl;
	    
	    cv::Mat colorMat(rows, cols, CV_8UC4, (void*)buffer_color, cv::Mat::AUTO_STEP);
	    color_image_save = colorMat;

	    imshow("color", colorMat);
	}
	if (waitKey(10) == 27){
	    bool check = imwrite("chess_board.jpg", color_image_save);
	    break;
	}



	k4a_image_release(depth_image);
	k4a_image_release(color_image);
	k4a_image_release(ir_image);
        k4a_capture_release(capture);
    }
    destroyWindow("depth");
    destroyWindow("color");
    destroyWindow("ir");
    k4a_capture_release(capture);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    return 0;
}

