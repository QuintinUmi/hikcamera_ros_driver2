#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "image_transport/image_transport.h"

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "CameraManager.h"

using namespace hikcamera_ros_driver2;

void sigintHandler(int sig) {
    ros::shutdown();  
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "multi_cam_ros_pub");
    ros::NodeHandle nh;
    CameraManager cameras(&nh);

    bool cRet = true;
    cRet = cameras.addAllCameras();
    if (!cRet) {
        ROS_ERROR("In addAllCameras Error Occurred\n");
    }
    cRet = cameras.initAllCameras();
    if (!cRet) {
        ROS_ERROR("In addAllCameras Error occurred\n");
    }
    cRet = cameras.startAllCamerasGrabbing();
    if (!cRet) {
        ROS_ERROR("In startAllCamerasGrabbing Error occurred\n");
    }

    while(ros::ok() && ros::master::check()) {
        ros::Rate(50).sleep();
    }


    return 0;
}
