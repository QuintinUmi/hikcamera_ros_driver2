#ifndef _HIKCAMERA_H_
#define _HIKCAMERA_H_

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

#include "HIKCAM_PARAM_CFG.h"

namespace hikcamera_ros_driver2{

    class Hikcamera {
    public:
        Hikcamera(MV_CC_DEVICE_INFO* pDeviceInfo, ros::NodeHandle* nh, bool load_ros_param);
        ~Hikcamera();

        void load_ros_param();

        bool init();
        bool startCapture();
        bool stopCapture();
        std::vector<uint8_t> getFrame();
        // bool setExposure(double exposureLevel) override;

    private:

        bool _PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

    private:

        MV_CC_DEVICE_INFO* _deviceInfo;
        ros::NodeHandle* _nh;
        uint8_t _camera_index;

        void* handle;

        struct {

                /*Image Format Control*/
                int Width;
                int Height;
                int OffsetX;
                int OffsetY;
                int PixelFormat;
                int ImageCompressionMode;

                /*Acquisition Control*/
                int AcquisitionLineRate;
                bool AcquisitionLineRateEnable;

                int TriggerMode;
                int TriggerSource;
                int TriggerActivation;

                float ExposureTime;
                int ExposureAuto;
                int AutoExposureTimeLowerLimit;
                int AutoExposureTimeupperLimit;

                /*Analog Control*/
                float Gain;
                int GainAuto;
                float AutoGainLowerLimit;
                float AutoGainupperLimit;

                int Brightness;

                /*Digital IO Control*/
                std::vector<int> LineSelector;
                std::vector<int> LineMode;
                std::vector<int> LineDebouncerTime;
                std::vector<bool> StrobeEnable;
                std::vector<int> LineSource;
                std::vector<int> StrobeLineDuration;
                std::vector<int> StrobeLineDelay;
                std::vector<int> StrobeLinePreDelay;

                /*Transport Layer Control*/
                bool GevIEEE1588;

            } _HIKCAMERA_PARAM;
        
    };

}


#endif