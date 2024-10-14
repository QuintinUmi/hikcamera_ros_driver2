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

int CAM_NUM = 0;

void ImageReceivedCallBack(std::shared_ptr<cv::Mat> rcv_image, int mv_format, uint64_t timestamp, int camera_index) {
    ROS_INFO("Camera [%d]: Grabbed Image at timestamp: [%ld]", camera_index, timestamp);
    cv::imshow(std::string(std::string("image_callback_") + std::to_string(camera_index)).c_str(), 
                Hikcamera::imageCvFormatting(*rcv_image, static_cast<HIK_PIXEL_FORMAT>(mv_format)));
    cv::waitKey(1);
}

void sigintHandler(int sig) {
    ros::shutdown();  // 显式调用ros::shutdown()
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "multi_cam_rcv_cb");
    ros::NodeHandle nh;
    CameraManager cameras(&nh);

    bool cRet = true;
    cRet = cameras.addAllCameras();
    if (!cRet) {
        ROS_ERROR("In addAllCameras Error Occurred\n");
    }
    cRet = cameras.initAllCameras(Hikcamera::WORK_THREAD_MODE_IMAGE_CALLBACK);
    if (!cRet) {
        ROS_ERROR("In addAllCameras Error occurred\n");
    }

    auto camera_list = cameras.getCameraList();
    CAM_NUM = camera_list.size();

    cRet = true;
    for (auto& cam : camera_list) {
        if(cam.second->setParamFromRosServer() != MV_OK) cRet = false;
    }
    for (auto& cam : camera_list) {
        cv::namedWindow(std::string(std::string("image_callback_") + std::to_string(cam.first)).c_str(), cv::WINDOW_NORMAL);
        if(cam.second->SetImageReceivedCb(ImageReceivedCallBack, &cam) != MV_OK) cRet = false;
    }
    for (auto& cam : camera_list) {
        if(cam.second->initWorkThread(Hikcamera::WORK_THREAD_MODE_IMAGE_CALLBACK) != MV_OK) cRet = false;
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
