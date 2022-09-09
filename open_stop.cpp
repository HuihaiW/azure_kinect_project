#include <k4a/k4a.h>
#include <iostream>

using namespace std;

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
    config.color_format		= K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution	= K4A_COLOR_RESOLUTION_2160P;
    config.depth_mode		= K4A_DEPTH_MODE_NFOV_UNBINNED;

    k4a_capture_t capture;
    if(K4A_FAILED(k4a_device_start_cameras(device, &config))){
	cout << "Failed to start cameras!" << endl;
	k4a_device_close(device);
	return 1;
    }

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
	k4a_image_t image = k4a_capture_get_depth_image(capture);
	if(image != NULL){
	    cout << "Depth16 res: " << k4a_image_get_height_pixels(image)<< "x" << k4a_image_get_width_pixels(image)<< ", stride: " << k4a_image_get_stride_bytes(image) << endl;
	    k4a_image_release(image);
	}
	cout << image << endl;
    }
    k4a_capture_release(capture);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    return 0;
}

