#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

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
    namedWindow("generated_low", WINDOW_NORMAL);
    namedWindow("generated_high", WINDOW_NORMAL);
    namedWindow("resized_high", WINDOW_NORMAL);


    double dur;
    clock_t start, end, end_xyz, end_whole;
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
    cv::Mat cv_location;
    cv::Mat cv_rgb;

    start = clock();
    k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);

    k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
    k4a_calibration_camera_t color_matrix = k4aCalibration.color_camera_calibration;
    k4a_calibration_intrinsics_t intrinsic = color_matrix.intrinsics;
    k4a_calibration_intrinsic_parameters_t parameters = intrinsic.parameters;
    float color_intrinsic_matrix[] = {0};
    int matrix_index = 0;
    while(matrix_index < 15){
	color_intrinsic_matrix[matrix_index] = parameters.v[matrix_index];
	//cout << "the parameter " << matrix_index << " of the matrix is: " << color_intrinsic_matrix[matrix_index] << endl;
	matrix_index ++;
    }


    end = clock();
    dur = double(end-start);
    cout << "time used for getting transformation: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;

    int index = 0;

    Matrix3d Homography, intrinsic_rotate, temp_matrix, rotation_matrix;
    
    //Homography << 0.000139809, -0.000187151, 0.815971, -1.88624e-06, -3.60233e-05, 0.578092, -3.35392e-08, -2.5564e-07, 0.00120086;
    //Homography << 0.000540608, -0.000543355, 0.736318, 1.08167e-05, -0.000132894, 0.676634, 2.55567e-08, -8.55258e-07, 0.00114261;
    //Homography <<  0.000530757, -0.000537706, 0.706097, 4.45295e-05, -0.000209662, 0.708113,  8.27766e-08, -8.7597e-07, 0.00119099;

    //Homography <<  0.000577036,-0.000556275,0.701894, 7.53814e-05,-0.000200309,0.71228, 1.11737e-07,-8.80059e-07,0.00108064;
    Homography <<  0.000724782, -0.000644843, 0.705882, 7.69309e-05,-0.000192769, 0.708329, 1.23824e-07,-1.02981e-06, 0.00112925;

    intrinsic_rotate << 606.782, 0.0,  643.805,
		     	0.0, 606.896,  366.084,
			0.0, 0.0,  1.0;
    Mat cv_K(4, 4, CV_64FC1);
    for(int i=0; i<3; i++){
	for(int j=0; j<4; j++){
	    if(j==2){
		cv_K.at<double>(i,j) = 0;
	    }
	    else if(j==3){
		cv_K.at<double>(i,j) = intrinsic_rotate(i,j-1);
	    }
	    else{
	    cv_K.at<double>(i,j) = intrinsic_rotate(i,j);
	    }
	}
    }

    for(int i=0; i<4; i++){
	cv_K.at<double>(3, i) = 0;
    }
    cv_K.at<double>(3, 2) = 1;

    temp_matrix = intrinsic_rotate.inverse() * Homography;

    Matrix<double, 3, 1> R1, R2, R3, translation;
    R1 << temp_matrix(0, 0), temp_matrix(1, 0), temp_matrix(2,0);
    R2 << temp_matrix(0, 1), temp_matrix(1, 1), temp_matrix(2,1);
    R3 = R1.normalized().cross(R2.normalized());
    rotation_matrix << R1.normalized(), R2.normalized(), R3;
    translation << temp_matrix(0,2)/R1.norm(),  temp_matrix(1,2)/R1.norm(), temp_matrix(2,2)/R1.norm();
    cout << "R1: " << R1.transpose() << endl;
    cout << "R2: " << R2.transpose() << endl;
    cout << "R3: " << R3.transpose() << endl;
    translation = rotation_matrix.inverse()*translation;
    cout << "translation: " << translation.transpose() << endl;
    cout << "inversed rotation matrix: " << rotation_matrix.inverse() << endl;
    Mat cv_Rotation(3, 4, CV_64FC1);
    for(int i=0; i<3; i++){
	for(int j=0; j<4; j++){
	    if(j==3){
		cv_Rotation.at<double>(i,j)= -1 * translation(i,0);
	    }
	    else{
		double rotation_value = 0;
		rotation_value = rotation_matrix.inverse()(i, j);
	        cv_Rotation.at<double>(i,j)=rotation_value;
	    }
	}
    }

    for (int i=0; i<cv_Rotation.rows; i++){
	cout << "-------------"<<endl;
	for(int j=0; j<cv_Rotation.cols; j++){
	    cout << cv_Rotation.at<double>(i,j)<<endl;
	}
    }

    while (true){

	
	cout << "xxxxxxxxxxxxx" << endl;
	cout << "Loop: " << index++ << endl;
	
	start = clock();
	if(device.get_capture(&capture)){
	    //color image
	    rgbImage = capture.get_color_image();
	    //cout << "[rgb] " << "\n" << "format: " << rgbImage.get_format() << "\n" << "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n" << "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n" << "height*width" << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels() << endl;
	    
	    //depth image

	    int height = rgbImage.get_height_pixels();
	    int width = rgbImage.get_width_pixels();
	    depthImage = capture.get_depth_image();

	    end = clock();
	    dur = double(end-start);
	    cout << "time used for get new color and depth images: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;
	    //cout << "[depth] " << "\n" << "format: " << depthImage.get_format() << "\n" << "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n" << "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n" << "height*width" << rgbImage.get_height_pixels() << ", " << depthImage.get_width_pixels() << endl;

	    //get the camera calibration for the entire K4A device, which is used for all transformation functions.

	    start = clock();
	    transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
	    cv_depth = cv::Mat(height, width, CV_16U, (void*)transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));
	    end = clock();
	    dur = double(end-start);
	    //cout << "time used for depth to color transformation: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    start = clock();
	    xyzImage = k4aTransformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR);
	    end = clock();

	    dur = double(end - start);
	    //cout << "time used for xyz generation: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    start = clock();


	    cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
						(void *)rgbImage.get_buffer());
	    cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
	    cv::imshow("rgb", cv_rgbImage_no_alpha);
	    cout << "display image" << endl;
	    
	    end = clock();
	    dur = double(end - start);
	    //cout << "time used for new images and display: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    cout << "creating points" << endl;
	    cout << "height X width: " << height << "X" << width << endl;
	    //Get point clouds

	    const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(xyzImage.get_buffer());
	    const uint8_t* color_buffer = rgbImage.get_buffer();
	    const size_t pointcount = xyzImage.get_height_pixels() * xyzImage.get_width_pixels(); 

	    const int new_height = 921600;
	    const int xyz_height=xyzImage.get_height_pixels();

	    cv_location = cv::Mat(xyzImage.get_height_pixels(),xyzImage.get_width_pixels(),  CV_16SC3, (void *)xyzImage.get_buffer());
	    cv_location = cv_location.reshape(1, xyzImage.get_height_pixels()*xyzImage.get_width_pixels() );
	    cv_rgb = cv_rgbImage_no_alpha.reshape(1, cv_rgbImage_no_alpha.rows*cv_rgbImage_no_alpha.cols);

	    Mat col_ones = Mat::ones(pointcount, 1, CV_16SC1);
	    hconcat(cv_location, col_ones, cv_location);

	    cv_location = cv_location.t();
	    cv_rgb = cv_rgb.t();

	    cv_location.convertTo(cv_location,CV_64FC1);

	    cv_location = (cv_Rotation * cv_location)/translation(2, 0);
	    Mat row_ones = Mat::ones(1, pointcount, CV_64FC1);
	    cv_location.push_back(row_ones);
	    cv_location = cv_K * cv_location;
	    

    	    Mat generated_image_low(180, 360, CV_8UC3, Scalar(0,0,0));
    	    Mat generated_image_high(360, 720, CV_8UC3, Scalar(0,0,0));
    	    Mat resized_image_high(360, 720, CV_8UC3, Scalar(0,0,0));
	    Mat mask(360, 720, CV_8UC1, Scalar(0));
	    Mat height_high(360, 720, CV_64FC1, Scalar(0));
	    Mat height_low(180, 360, CV_64FC1, Scalar(0));

	    
	    
	    for(size_t i=0; i<pointcount; i++){

		double x = cv_location.at<double>(0, i);
		double y = cv_location.at<double>(1, i);
		double z = -1.0 * cv_location.at<double>(3, i);

		int r = cv_rgb.at<uchar>(0, i);
		int g = cv_rgb.at<uchar>(1, i);
		int b = cv_rgb.at<uchar>(2, i);
		
		if(x>1440) continue;
		if(x<0) continue;
		if(y>720)continue;
		if(y<0)continue;

		double scale_image = 4.0;

		int i_x = static_cast<int>(x/scale_image);
		int i_y = static_cast<int>(y/scale_image);

		if(z > height_low.at<double>(i_y, i_x)){
		    generated_image_low.at<Vec3b>(i_y, i_x)[0] = r;
		    generated_image_low.at<Vec3b>(i_y, i_x)[1] = g;
		    generated_image_low.at<Vec3b>(i_y, i_x)[2] = b;
		    height_low.at<double>(i_y, i_x) = z;
		}
		
                scale_image = 2.0;
		i_x = static_cast<int>(x/scale_image);
		i_y = static_cast<int>(y/scale_image);
		if(z > height_high.at<double>(i_y, i_x)){
		    generated_image_high.at<Vec3b>(i_y, i_x)[0] = r;
		    generated_image_high.at<Vec3b>(i_y, i_x)[1] = g;
		    generated_image_high.at<Vec3b>(i_y, i_x)[2] = b;
		    height_high.at<double>(i_y, i_x) = z;
		    mask.at<uchar>(i_y,i_x)=255;
		}


	    }

  	    resize(generated_image_low, resized_image_high, Size(720,360), INTER_AREA);
	    resized_image_high.copyTo(generated_image_high, 255-mask);
	    //blur(generated_image_high,generated_image_high, Size(5,5));
	    

	    imshow("generated_low", generated_image_low);
	    imshow("resized_high", resized_image_high);
	    imshow("generated_hight", generated_image_high);


	    capture.reset();
	    
	    if(cv::waitKey(10) == 'q'){

		cout << "--------------------------------------------" << endl;
		cout << "--------------------closed------------------" << endl;
		cout << "--------------------------------------------" << endl;
		break;

	    }
	    pointcloud.clear();
	}
    }
    cv::destroyAllWindows();
    
    rgbImage.reset();
    depthImage.reset();
    capture.reset();
    device.close();
    return 1;
    
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 3840, 2160);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3]/255.0, p[4]/255.0, p[5]/255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
