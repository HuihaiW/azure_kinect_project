cmake_minimum_required(VERSION 3.9.0)

project(Hello_Azure_Kinect)
find_package(k4a REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(images ./src/get_images.cpp)
add_executable(pointCloud ./src/get_pointCloud.cpp)
add_executable(pointCloudCpp ./src/get_pointCloud_cpp.cpp)


target_link_libraries(images k4a::k4a ${OpenCV_LIBS})
target_link_libraries(pointCloud k4a::k4a ${OpenCV_LIBS})
target_link_libraries(pointCloudCpp k4a::k4a ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

set(CMAKE_CXX_FLAGES "-std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
