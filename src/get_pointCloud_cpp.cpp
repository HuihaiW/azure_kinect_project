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
	cout << "the parameter " << matrix_index << " of the matrix is: " << color_intrinsic_matrix[matrix_index] << endl;
	matrix_index ++;
    }


    end = clock();
    dur = double(end-start);
    cout << "time used for getting transformation: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;

    int index = 0;

    Matrix3d Homography, intrinsic_rotate, temp_matrix, rotation_matrix;
    //Homography << 0.00694, -0.00184, 0.67004, 
	          -0.00069, 0.00317, 0.74227,
		  0.0, 0.0, 0.00160;

    //Homography << 0.0143067, -0.00385386, 0.856358, -6.73185e-05, 0.00746237, 0.516135, 2.43995e-07, -5.37434e-06, 0.000985956;
    //Homography << -0.511637, 0.629602, -0.466258, 0.0223014, 0.0954103, -0.328452, 0.000104113, 0.000889112, -0.000662905;
    //Homography << -0.504521, 0.628555, 0.471883, 0.108265, 0.00757, 0.335677, 0.000156151, 0.000889, 0.0006899; 
    Homography << -0.508861, 0.62868, -0.465, 0.0234815, 0.0963, -0.327516, 0.000107326, 0.00088763, -0.000660675;
    intrinsic_rotate << 606.782, 0.0, 643.805,
		     	0.0, 606.896, 366.084,
			0.0, 0.0, 1.0;
    temp_matrix = intrinsic_rotate.inverse() * Homography*intrinsic_rotate;
    Matrix<double, 3, 1> R1, R2, R3;
    R1 << temp_matrix(0, 0), temp_matrix(1, 0), temp_matrix(2,0);
    R2 << temp_matrix(0, 1), temp_matrix(1, 1), temp_matrix(2,1);
    R3 = R1.cross(R2);
    rotation_matrix << R1, R2, R3;

    
    Matrix3d new_coord; 
    new_coord << 1.0, 0.0, 0.0, 0.0, 0.5, 0.87, 0.0, -0.87, 0.5;
    rotation_matrix = new_coord.inverse();
    
    


    cout << "The temp_matrix is: " << temp_matrix << endl;
    cout << "R1 is: " << R1 << endl;
    cout << "R2 is: " << R2 << endl;
    cout << "R3 is: " << R3 << endl;
    cout << "rotation coord is: " << rotation_matrix.inverse() << endl;
    

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
	    cout << "time used for depth to color transformation: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    start = clock();
	    xyzImage = k4aTransformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR);
	    end = clock();

	    dur = double(end - start);
	    cout << "time used for xyz generation: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    start = clock();


	    cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
						(void *)rgbImage.get_buffer());
	    cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
	    cv::imshow("rgb", cv_rgbImage_no_alpha);
	    cout << "display image" << endl;
	    
	    end = clock();
	    dur = double(end - start);
	    cout << "time used for new images and display: " << (dur * 1000 / CLOCKS_PER_SEC) << endl;

	    cout << "creating points" << endl;
	    cout << "height X width: " << height << "X" << width << endl;
	    //Get point clouds

	    const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(xyzImage.get_buffer());
	    const uint8_t* color_buffer = rgbImage.get_buffer();
	    const size_t pointcount = xyzImage.get_height_pixels() * xyzImage.get_width_pixels(); 

	    for(size_t i = 0; i < pointcount; i++){
		float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);
		if (z <= 0.0f || z ==0){
		    continue;
		}
		else{
		    Vector6d point;
		    constexpr float kMillimeterToMeter = 1.0/1000.0f;
		    float x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
		    float y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
		    z = kMillimeterToMeter * z;

		    int r = color_buffer[4 * i + 2];
		    int g = color_buffer[4 * i + 1];
		    int b = color_buffer[4 * i + 0];
		    Matrix<float, 3, 1> o_p, new_p;
		    o_p << x, y, z;
		    //cout << "new point is: " << rotation_matrix* o_p.cast<double> << endl;
		    new_p = rotation_matrix.cast<float>() * o_p;
		    //new_p = temp_matrix.cast<float>() * o_p;
		    //new_p = Homography.inverse().cast<float>() * o_p;
		    ///*
		    //*/
		    ///*
		    float scale = 1.0;
		    point[0] = scale * new_p(0, 0);
		    point[1] = scale * new_p(1, 0);
		    point[2] = 0 * new_p(2, 0);
		    //*/
		    point[3] = r;
		    point[4] = g;
		    point[5] = b;

		    pointcloud.push_back(point);
		    
		    point[0] = x;
		    point[1] = y;
		    point[2] = z;
		   
		    //pointcloud.push_back(point);
		    
		    //cout << "rgb: " << r << ", " << g << ", " << b << "," << endl;
		    //cout << "xyz: " << x << ", " << y << ", " << z << "," << endl;
		}
	    }
	    

	    showPointCloud(pointcloud);

	    /*

	    for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
		    int r = cv_rgbImage_no_alpha.at<Vec3d>(i, j)[1];
		    int g = cv_rgbImage_no_alpha.at<Vec3d>(i, j)[2];
		    int b = cv_rgbImage_no_alpha.at<Vec3d>(i, j)[3];
		    int x = cv_depth.at<Vec3d>(i, j)[0];
		    int y = cv_depth.at<Vec3d>(i, j)[1];
		    int z = cv_depth.at<Vec3d>(i, j)[2];
		    cout << "rgb: " << r << ", " << g << ", " << b << "," << endl;
		    cout << "xyz: " << x << ", " << y << ", " << z << "," << endl;
		}
	    }
	    */

	    /*rgbImage.release();
	    depthImage.release();
	    capture.release();
	    xyzImage.release();
	    */
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

