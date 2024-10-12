#ifndef _HIKCAMERA_ROS_H_
#define _HIKCAMERA_ROS_H_

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


namespace hikcamera_ros_driver2
{
    class CameraManager
    {
        public:
            CameraManager();
            ~CameraManager();

            

        public:





        protected:

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

            ros::NodeHandle nh;

            

            void *camHandle;
            int camIndex;

            int nRet;

    };
}




#endif 
