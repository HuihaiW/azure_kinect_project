#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    const uint32_t device_count = k4a::device::get_installed_count();
    if(device_count == 0){
	cout << "No k4a device attached!" << endl;
	return -1;
    }
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT); 
    cout << "Done: Open device." << endl;

    // configuration of the camera
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps		= K4A_FRAMES_PER_SECOND_30;
    config.color_format		= K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution	= K4A_COLOR_RESOLUTION_720P;
    config.depth_mode		= K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    //start camera
    device.start_cameras(&config);
    cout << "Done: start camera." << endl;

    // create a capture
    k4a::capture capture;
    namedWindow("rgb", WINDOW_NORMAL);


    double dur;
    clock_t start, end;
    int iAuto = 0;
    int iAutoError = 0;
    // get frames
    while(true){

	if(device.get_capture(&capture)){
	    std::cout << iAuto << ". Capture several frames to give auto-exposure" << endl;
	    if (iAuto != 30){
		iAuto++;
		continue;
	    }else{
		cout << "Done: auto-exposure" << endl;
		break;
	    }
	}else{
	    cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << endl;
	    if (iAutoError != 30){
		iAutoError++;
		continue;
	    }else{
		cout << "Error: failed to give auto-exposure. " << endl;
		return -1;
	    }
	}
    }
    cout << "------------------------------------" << endl;
    cout << "--------ave Started Kinect DK.------" << endl;
    cout << "------------------------------------" << endl;

    k4a::image rgbImage;
    k4a::image depthImage;
    k4a::image irImage;
    k4a::image transformed_depthImage;
    k4a::image xyzImage;

    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_irImage;
    cv::Mat cv_irImage_8U;

    while (true){
	if(device.get_capture(&capture)){
	    //color image
	    rgbImage = capture.get_color_image();
	    //cout << "[rgb] " << "\n" << "format: " << rgbImage.get_format() << "\n" << "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n" << "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n" << "height*width" << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels() << endl;
	    
	    //depth image
	    depthImage = capture.get_depth_image();
	    //cout << "[depth] " << "\n" << "format: " << depthImage.get_format() << "\n" << "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n" << "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n" << "height*width" << rgbImage.get_height_pixels() << ", " << depthImage.get_width_pixels() << endl;

	    //get the camera calibration for the entire K4A device, which is used for all transformation functions.
	    k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);
	    k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);

	    start = clock();
	    transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
	    end = clock();
	    dur = double(end-start);
	    cout << "time used: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    xyzImage = k4aTransformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH);

	    int height = rgbImage.get_height_pixels();
	    int width = rgbImage.get_width_pixels();

	    cout << "height X width: " << height << "X" << width << endl;

	    cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
						(void *)rgbImage.get_buffer());
	    cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
	    cv::imshow("rgb", cv_rgbImage_no_alpha);
	    cout << "display image" << endl;


	    /*rgbImage.release();
	    depthImage.release();
	    capture.release();
	    xyzImage.release();
	    */

	    
	}
    }
    rgbImage.reset();
    depthImage.reset();
    capture.reset();
    device.close();
    return 0;
    
}
