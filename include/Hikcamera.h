#ifndef _HIKCAMERA_H_
#define _HIKCAMERA_H_

#include <iostream>
#include <thread>
#include <functional>
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
#include "thread_safe_queue.h"

#define IS_SHOW_FRAME_INFO false
#define DEBUG_GET_FRAME_ONLY false

namespace hikcamera_ros_driver2{

    class Hikcamera {
        public: 
            typedef enum _WORK_THREAD_MODE_ {
                WORK_THREAD_MODE_ROS_PUBLISH,
                WORK_THREAD_MODE_IMAGE_CALLBACK,
                WORK_THREAD_MODE_NOT_SELECTED
            }WORK_THREAD_MODE;

            typedef void (*ImageReceivedCb)(std::shared_ptr<cv::Mat> rcv_image, uint64_t timestamp, int camera_index);

        public:
            Hikcamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index);
            Hikcamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index, ros::NodeHandle* nh);
            ~Hikcamera();

            int init(bool set_param_from_ros = true);
            int initDevice();
            int deinitDevice();
            int initWorkThread(WORK_THREAD_MODE work_thread_mode = WORK_THREAD_MODE_ROS_PUBLISH);
            int releaseAllWorkThread();

            int32_t SetImageReceivedCb(ImageReceivedCb cb, void *data);

            void loadParamFromRosServer();
            void loadRosConfig();
            int setParamFromRosServer();
            
            int setImageFormat(int Width, int Height, int OffsetX, int OffsetY, 
                                HIK_PIXEL_FORMAT PixelFormat, HIK_IMAGE_COMPRESSION_MODE ImageCompressionMode);

            int setAcquisitionRate(HIK_ACQUISITION_LINE_RATE_ENABLE AcquisitionLineRateEnable,
                                    int AcquisitionLineRate);
            int setTrigger(HIK_TRIGGER_MODE TriggerMode, HIK_TRIGGER_SOURCE TriggerSource, HIK_TRIGGER_ACTIVATION TriggerActivation);
            int setExplosure(HIK_EXPOSURE_AUTO ExposureAuto, float ExposureTime, int AutoExposureTimeLowerLimit, int AutoExposureTimeUpperLimit);
            
            int setGain(HIK_GAIN_AUTO GainAuto, float Gain, float AutoGainLowerLimit, float AutoGainUpperLimit);
            int setBrightness(int Brightness);
            
            int setLineIO(HIK_LINE_SELECTOR LineSelector, HIK_LINE_MODE LineMode, 
                            int LineDebouncerTime,
                            HIK_STROBE_ENABLE StrobeEnable, HIK_LINE_SOURCE LineSource, 
                            int StrobeLineDuration, int StrobeLineDelay, int StrobeLinePreDelay);
            int setLineIOBatch(std::vector<HIK_LINE_SELECTOR> LineSelector, std::vector<HIK_LINE_MODE> LineMode,
                                std::vector<int> LineDebouncerTime,
                                std::vector<HIK_STROBE_ENABLE> StrobeEnable, std::vector<HIK_LINE_SOURCE> LineSource,
                                std::vector<int> StrobeLineDuration, std::vector<int> StrobeLineDelay, std::vector<int> StrobeLinePreDelay);

            int setTransportLayerControl(HIK_GEV_IEEE_1588 GevIEEE1588);
            

            int startGrabbing();
            int stopGrabbing();
            
            static cv::Mat imageMvToCv(const MV_FRAME_OUT& stFrameOut, HIK_PIXEL_FORMAT hik_format);
            static sensor_msgs::ImagePtr imageCvToRos(const cv::Mat& cv_image, HIK_PIXEL_FORMAT hik_format);

            static uint64_t nDevTimeStampCov(unsigned int nDevTimeStampHigh, unsigned int nDevTimeStampLow);
            static ros::Time timestampToRos(const uint64_t timestamp);
            static std::string timestampToDateTime(uint64_t timestamp_ns);
            static std::string timestampToSeconds(uint64_t timestamp_ns);

            bool printDeviceInfo();

        public:

            struct ImageQueuePacket{
                cv::Mat image;
                MV_FRAME_OUT_INFO_EX stFrameInfo;
                uint64_t seq;

                ImageQueuePacket(cv::Mat image, MV_FRAME_OUT_INFO_EX stFrameInfo, int seq) : image(image), stFrameInfo(stFrameInfo), seq(seq) {}
            };
            

        private:

            void GetFrameWorkThread();
            void RosPublishWorkThread();
            void ImageReceivedCallBackWorkThread();

        private:

            MV_CC_DEVICE_INFO* _pDeviceInfo;
            ros::NodeHandle* _nh;
            uint8_t _camera_index;

            void* _handle;
            int _nPacketSize;

            WORK_THREAD_MODE _work_thread_mode;

            std::shared_ptr<std::thread> _get_frame_wt;
            volatile bool _exit_get_frame_wt;
            volatile bool _start_get_frame_wt;

            std::shared_ptr<std::thread> _ros_publish_wt;
            volatile bool _exit_ros_publish_wt;
            volatile bool _start_ros_publish_wt;

            std::shared_ptr<std::thread> _image_received_callback_wt;
            volatile bool _exit_image_received_callback_wt;
            volatile bool _start_image_received_callback_wt;

            ImageReceivedCb _image_received_cb;
            void* _client_data;

            ThreadSafeQueue<ImageQueuePacket> _ImageQueue;

            bool _set_center;
            struct {

                /*Image Format Control*/
                int Width;
                int Height;
                int OffsetX;
                int OffsetY;
                HIK_PIXEL_FORMAT PixelFormat;
                HIK_IMAGE_COMPRESSION_MODE ImageCompressionMode;

                /*Acquisition Control*/
                int AcquisitionLineRate;
                HIK_ACQUISITION_LINE_RATE_ENABLE AcquisitionLineRateEnable;

                HIK_TRIGGER_MODE TriggerMode;
                HIK_TRIGGER_SOURCE TriggerSource;
                HIK_TRIGGER_ACTIVATION TriggerActivation;

                float ExposureTime;
                HIK_EXPOSURE_AUTO ExposureAuto;
                int AutoExposureTimeLowerLimit;
                int AutoExposureTimeUpperLimit;

                /*Analog Control*/
                float Gain;
                HIK_GAIN_AUTO GainAuto;
                float AutoGainLowerLimit;
                float AutoGainUpperLimit;

                int Brightness;

                /*Digital IO Control*/
                std::vector<HIK_LINE_SELECTOR> LineSelector;
                std::vector<HIK_LINE_MODE> LineMode;
                std::vector<int> LineDebouncerTime;
                std::vector<HIK_STROBE_ENABLE> StrobeEnable;
                std::vector<HIK_LINE_SOURCE> LineSource;
                std::vector<int> StrobeLineDuration;
                std::vector<int> StrobeLineDelay;
                std::vector<int> StrobeLinePreDelay;

                /*Action Control*/
                int ActionDeviceKey;
                int ActionGroupMask;
                int ActionGroupKey;

                /*Transport Layer Control*/
                HIK_GEV_IEEE_1588 GevIEEE1588;

            } _HIKCAMERA_PARAM;

            struct {
                std::string image_publish_topic;
                bool publish_compressed;
            } _ROS_PARAM;
            
    };


    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstHIKDevInfo)
    {
        if (NULL == pstHIKDevInfo)
        {
            printf("The Pointer of pstHIKDevInfo is NULL!\n");
            return false;
        }
        if (pstHIKDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((pstHIKDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstHIKDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstHIKDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstHIKDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // print current ip and user defined name
            printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
            printf("UserDefinedName: %s\n\n" , pstHIKDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }

        return true;
    }
}


#endif