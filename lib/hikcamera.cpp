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

#include "hikcamera.h"




void HikCamera::printParam(){
    printf("width: %d\n", this->hikcamera_param.width);
    printf("height: %d\n", this->hikcamera_param.height);
    printf("Offset_x: %d\n", this->hikcamera_param.Offset_x);
    printf("Offset_y: %d\n", this->hikcamera_param.Offset_y);
    printf("FrameRateEnable: %d\n", this->hikcamera_param.FrameRateEnable);
    printf("FrameRate: %d\n", this->hikcamera_param.FrameRate);

    printf("TriggerMode: %d\n", this->hikcamera_param.TriggerMode);
    printf("LineSelector: %d\n", this->hikcamera_param.LineSelector);
    printf("StrobeEnable: %d\n", this->hikcamera_param.StrobeEnable);
    printf("StrobeLineDelay: %d\n", this->hikcamera_param.StrobeLineDelay);
    printf("StrobeLinePreDelay: %d\n", this->hikcamera_param.StrobeLinePreDelay);

    printf("ExposureAuto: %d\n", this->hikcamera_param.ExposureAuto);
    printf("ExposureTimeUpper: %d\n", this->hikcamera_param.ExposureTimeUpper);
    printf("ExposureTimeLower: %d\n", this->hikcamera_param.ExposureTimeLower);
    printf("ExposureTime: %d\n", this->hikcamera_param.ExposureTime);

    printf("GainAuto: %d\n", this->hikcamera_param.GainAuto);
    printf("Gain: %f\n", this->hikcamera_param.Gain); 

    printf("BayerCvtQuality: %d\n", this->hikcamera_param.bayerCvtQuality);

    printf("undistortion: %d\n", this->undistortion); 
}


