cmake_minimum_required(VERSION 3.9.0)

project(Hello_Azure_Kinect)
find_package(k4a REQUIRED)

add_executable(hello test.cpp)
add_executable(first open_stop.cpp)
add_executable(open_device find_open.cpp)


target_link_libraries(hello k4a::k4a)
target_link_libraries(first k4a::k4a)
target_link_libraries(open_device k4a::k4a)

set(CMAKE_CXX_FLAGES "-std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
