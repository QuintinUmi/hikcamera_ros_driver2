#include "Hikcamera.h"
#include <iostream> 

using namespace hikcamera_ros_driver2;

Hikcamera::Hikcamera(MV_CC_DEVICE_INFO* deviceInfo, ros::NodeHandle* nh, bool load_ros_param) : _deviceInfo(deviceInfo), _nh(nh) {

    if (load_ros_param) {

    }
}

void Hikcamera::load_ros_param() {

    /*Image Format Control*/
    _nh->param("Width", _HIKCAMERA_PARAM.Width, 2448);
    _nh->param("Height", _HIKCAMERA_PARAM.Height, 2048);
    _nh->param("OffsetX", _HIKCAMERA_PARAM.OffsetX, 0);
    _nh->param("OffsetY", _HIKCAMERA_PARAM.OffsetY, 0);
    _nh->param("PixelFormat", _HIKCAMERA_PARAM.PixelFormat, int(MV_PIXEL_FORMAT_BayerBG8));
    _nh->param("ImageCompressionMode", _HIKCAMERA_PARAM.ImageCompressionMode, int(MV_IMAGE_COMPRESSION_MODE_HB));

    /*Acquisition Control*/
    _nh->param("AcquisitionLineRate", _HIKCAMERA_PARAM.AcquisitionLineRate, 20);
    _nh->param("AcquisitionLineRateEnable", _HIKCAMERA_PARAM.AcquisitionLineRateEnable, bool(MV_ACQUISITION_LINE_RATE_ENABLE_ON));
    _nh->param("TriggerMode", _HIKCAMERA_PARAM.TriggerMode, int(MV_TRIGGER_MODE_ON));
    _nh->param("TriggerSource", _HIKCAMERA_PARAM.TriggerSource, int(MV_TRIGGER_SOURCE_Action1_SPECIAL));
    _nh->param("TriggerActivation", _HIKCAMERA_PARAM.TriggerActivation, int(MV_TRIGGER_ACTIVATION_RisingEdge));

    _nh->param("ExposureTime", _HIKCAMERA_PARAM.ExposureTime, 15000.f);
    _nh->param("ExposureAuto", _HIKCAMERA_PARAM.ExposureAuto, int(MV_EXPOSURE_AUTO_CONTINUOUS));
    _nh->param("AutoExposureTimeLowerLimit", _HIKCAMERA_PARAM.AutoExposureTimeLowerLimit, 5000);
    _nh->param("AutoExposureTimeupperLimit", _HIKCAMERA_PARAM.AutoExposureTimeupperLimit, 45000);

    /*Analog Control*/
    _nh->param("Gain", _HIKCAMERA_PARAM.Gain, 25.f);
    _nh->param("GainAuto", _HIKCAMERA_PARAM.GainAuto, int(MV_GAIN_AUTO_CONTINUOUS));
    _nh->param("AutoGainLowerLimit", _HIKCAMERA_PARAM.AutoGainLowerLimit, 25.f);
    _nh->param("AutoGainupperLimit", _HIKCAMERA_PARAM.AutoGainupperLimit, 0.f);

    _nh->param("Brightness", _HIKCAMERA_PARAM.Brightness, 100);

    /*Digital IO Control*/
    _nh->param("LineSelector", _HIKCAMERA_PARAM.LineSelector, std::vector<int>{MV_LINE_SELECTOR_Line1});
    _nh->param("LineMode", _HIKCAMERA_PARAM.LineMode, std::vector<int>{MV_LINE_MODE_Strobe});
    _nh->param("LineDebouncerTime", _HIKCAMERA_PARAM.LineDebouncerTime, std::vector<int>{50});
    _nh->param("StrobeEnable", _HIKCAMERA_PARAM.StrobeEnable, std::vector<bool>{MV_STROBE_ENABLE_OFF});
    _nh->param("LineSource", _HIKCAMERA_PARAM.LineSource, std::vector<int>{MV_LINE_SOURCE_ExposureStartActive});
    _nh->param("StrobeLineDuration", _HIKCAMERA_PARAM.StrobeLineDuration, std::vector<int>{0});
    _nh->param("StrobeLineDelay", _HIKCAMERA_PARAM.StrobeLineDelay, std::vector<int>{0});
    _nh->param("StrobeLinePreDelay", _HIKCAMERA_PARAM.StrobeLinePreDelay, std::vector<int>{0});

    /*Transport Layer Control*/
    _nh->param("GevIEEE1588", _HIKCAMERA_PARAM.GevIEEE1588, false);

}

Hikcamera::~Hikcamera() {
    // Destructor logic, maybe clean up resources
}

bool Hikcamera::init() {

    int nRet;
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet)
    {
        printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
    
    
}



// bool Hikcamera::startCapture() {
//     isCapturing = true;
//     // Start capture logic
//     std::cout << "Capture started." << std::endl;
//     return true;
// }

// bool Hikcamera::stopCapture() {
//     isCapturing = false;
//     // Stop capture logic
//     std::cout << "Capture stopped." << std::endl;
//     return true;
// }

// std::vector<uint8_t> Hikcamera::getFrame() {
//     std::vector<uint8_t> frame;
//     if (isCapturing) {
//         // Simulate a frame capture
//         frame.resize(width * height * 3); // Assuming a simple RGB format
//         std::fill(frame.begin(), frame.end(), 255); // Dummy data for demonstration
//     }
//     return frame;
// }

// bool Hikcamera::setExposure(double exposureLevel) {
//     exposure = exposureLevel;
//     // Set exposure logic
//     std::cout << "Exposure set to: " << exposure << std::endl;
//     return true;
// }

bool Hikcamera::_PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:��ӡ��ǰ���ip���û��Զ������� | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}