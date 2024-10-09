#ifndef HIKCAMERA_ROS_H_
#define HIKCAMERA_ROS_H_

#include <condition_variable>
#include <chrono>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"       

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "MvCameraControl.h"


namespace hikcamera_ros_driver2
{
    class Hikcamera
    {
        public:

            typedef struct HIKCAMERA_PARAM {

                int width;
                int height;
                int Offset_x;
                int Offset_y;
                bool FrameRateEnable;
                int FrameRate;

                int TriggerMode;
                int LineSelector;
                int LineMode;
                int LineSource;

                bool StrobeEnable;
                int StrobeLineDelay;
                int StrobeLinePreDelay;

                int ExposureAuto;
                int ExposureTimeUpper;
                int ExposureTimeLower;
                int ExposureTime;

                float Gain;
                int GainAuto;

                int bayerCvtQuality;

            } HIKCAMERA_PARAM;

        public:

            



        protected:

            HIKCAMERA_PARAM hikcamera_param;

            cv::String cameraIntrinsicsPath;
            bool undistortion;
            int interpolation;

            cv::Size imageSize;
            cv::Mat cameraMatrix;
            cv::Mat disCoffes;
            double alpha;
            cv::Mat newCameraMatrix;
            cv::Size newImageSize;
            cv::Mat map1;
            cv::Mat map2;

            bool isResize;
            cv::Size imageReize;

        private:

            ros::NodeHandle rosHandle;

            void *camHandle;
            int camIndex;

            int nRet;

    };

}


#endif 
