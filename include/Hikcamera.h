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

        int init_device();

        void loadRosServerParam();
        int setParamFromRosServer();
        
        int setImageFormat(int Width, int Height, int OffsetX, int OffsetY, 
                            MV_PIXEL_FORMAT PixelFormat, MV_IMAGE_COMPRESSION_MODE ImageCompressionMode);

        int setAcquisitionRate(MV_ACQUISITION_LINE_RATE_ENABLE AcquisitionLineRateEnable,
                                int AcquisitionLineRate);
        int setTrigger(MV_TRIGGER_MODE TriggerMode, MV_TRIGGER_SOURCE TriggerSource, MV_TRIGGER_ACTIVATION TriggerActivation);
        int setExplosure(MV_EXPOSURE_AUTO ExposureAuto, float ExposureTime, int AutoExposureTimeLowerLimit, int AutoExposureTimeUpperLimit);
        
        int setGain(MV_GAIN_AUTO GainAuto, float Gain, float AutoGainLowerLimit, float AutoGainUpperLimit);
        int setBrightness(int Brightness);
        
        int setLineIO(MV_LINE_SELECTOR LineSelector, MV_LINE_MODE LineMode, 
                        int LineDebouncerTime,
                        MV_STROBE_ENABLE StrobeEnable, MV_LINE_SOURCE LineSource, 
                        int StrobeLineDuration, int StrobeLineDelay, int StrobeLinePreDelay);
        int setLineIOBatch(std::vector<MV_LINE_SELECTOR> LineSelector, std::vector<MV_LINE_MODE> LineMode,
                            std::vector<int> LineDebouncerTime,
                            std::vector<MV_STROBE_ENABLE> StrobeEnable, std::vector<MV_LINE_SOURCE> LineSource,
                            std::vector<int> StrobeLineDuration, std::vector<int> StrobeLineDelay, std::vector<int> StrobeLinePreDelay);

        int setTransportLayerControl(MV_GEV_IEEE_1588 GevIEEE1588);
        

        int startGrabbing();
        bool stopCapture();
        std::vector<uint8_t> getFrame();
        // bool setExposure(double exposureLevel) override;

    private:

        bool _printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

        

    private:

        MV_CC_DEVICE_INFO* _pDeviceInfo;
        ros::NodeHandle* _nh;
        uint8_t _camera_index;

        void* _handle;
        int _nPacketSize;

        struct {

                /*Image Format Control*/
                int Width;
                int Height;
                int OffsetX;
                int OffsetY;
                MV_PIXEL_FORMAT PixelFormat;
                MV_IMAGE_COMPRESSION_MODE ImageCompressionMode;

                /*Acquisition Control*/
                int AcquisitionLineRate;
                MV_ACQUISITION_LINE_RATE_ENABLE AcquisitionLineRateEnable;

                MV_TRIGGER_MODE TriggerMode;
                MV_TRIGGER_SOURCE TriggerSource;
                MV_TRIGGER_ACTIVATION TriggerActivation;

                float ExposureTime;
                MV_EXPOSURE_AUTO ExposureAuto;
                int AutoExposureTimeLowerLimit;
                int AutoExposureTimeUpperLimit;

                /*Analog Control*/
                float Gain;
                MV_GAIN_AUTO GainAuto;
                float AutoGainLowerLimit;
                float AutoGainUpperLimit;

                int Brightness;

                /*Digital IO Control*/
                std::vector<MV_LINE_SELECTOR> LineSelector;
                std::vector<MV_LINE_MODE> LineMode;
                std::vector<int> LineDebouncerTime;
                std::vector<MV_STROBE_ENABLE> StrobeEnable;
                std::vector<MV_LINE_SOURCE> LineSource;
                std::vector<int> StrobeLineDuration;
                std::vector<int> StrobeLineDelay;
                std::vector<int> StrobeLinePreDelay;

                /*Transport Layer Control*/
                MV_GEV_IEEE_1588 GevIEEE1588;

            } _HIKCAMERA_PARAM;
        
    };

}


#endif