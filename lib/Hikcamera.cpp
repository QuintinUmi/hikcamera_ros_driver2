#include "Hikcamera.h"
#include <iostream> 

using namespace hikcamera_ros_driver2;

Hikcamera::Hikcamera(MV_CC_DEVICE_INFO* pDeviceInfo, ros::NodeHandle* nh, bool is_set_param_from_ros) : _pDeviceInfo(pDeviceInfo), _nh(nh) 
{
    if (init_device() != MV_OK) {
        ROS_WARN("Incomplete initialization!");
    }

    if (is_set_param_from_ros) {
        if(setParamFromRosServer() != MV_OK) {
            ROS_WARN("Incomplete setParamFromRosServer!");
        }
    }
}

Hikcamera::~Hikcamera() 
{
    // Destructor logic, maybe clean up resources
}


int Hikcamera::init_device()
{
    int nRet = MV_OK;
    nRet = MV_CC_CreateHandle(&_handle, _pDeviceInfo);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Create Handle fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    nRet = MV_CC_OpenDevice(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Open Device fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    if (_pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        _nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
        if (_nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValue(_handle,"GevSCPSPacketSize",_nPacketSize);
            if(nRet != MV_OK)
            {
                ROS_WARN("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
            }
        }
        else
        {
            ROS_WARN("Warning: Get Packet Size fail nRet [0x%x]!", _nPacketSize);
        }
    }

    return nRet;
}

void Hikcamera::loadRosServerParam() 
{

    /*Image Format Control*/
    _nh->param("Width", _HIKCAMERA_PARAM.Width, 2448);
    _nh->param("Height", _HIKCAMERA_PARAM.Height, 2048);
    _nh->param("OffsetX", _HIKCAMERA_PARAM.OffsetX, 0);
    _nh->param("OffsetY", _HIKCAMERA_PARAM.OffsetY, 0);
    _nh->param("PixelFormat", _HIKCAMERA_PARAM.PixelFormat, MV_PIXEL_FORMAT_BayerBG8);
    _nh->param("ImageCompressionMode", _HIKCAMERA_PARAM.ImageCompressionMode, MV_IMAGE_COMPRESSION_MODE_HB);

    /*Acquisition Control*/
    _nh->param("AcquisitionLineRate", _HIKCAMERA_PARAM.AcquisitionLineRate, 20);
    _nh->param("AcquisitionLineRateEnable", _HIKCAMERA_PARAM.AcquisitionLineRateEnable, MV_ACQUISITION_LINE_RATE_ENABLE_ON);

    _nh->param("TriggerMode", _HIKCAMERA_PARAM.TriggerMode, MV_TRIGGER_MODE_ON);
    _nh->param("TriggerSource", _HIKCAMERA_PARAM.TriggerSource, MV_TRIGGER_SOURCE_Action1_SPECIAL);
    _nh->param("TriggerActivation", _HIKCAMERA_PARAM.TriggerActivation, MV_TRIGGER_ACTIVATION_RisingEdge);

    _nh->param("ExposureTime", _HIKCAMERA_PARAM.ExposureTime, 15000.f);
    _nh->param("ExposureAuto", _HIKCAMERA_PARAM.ExposureAuto, MV_EXPOSURE_AUTO_CONTINUOUS);
    _nh->param("AutoExposureTimeLowerLimit", _HIKCAMERA_PARAM.AutoExposureTimeLowerLimit, 5000);
    _nh->param("AutoExposureTimeUpperLimit", _HIKCAMERA_PARAM.AutoExposureTimeUpperLimit, 45000);

    /*Analog Control*/
    _nh->param("Gain", _HIKCAMERA_PARAM.Gain, 25.f);
    _nh->param("GainAuto", _HIKCAMERA_PARAM.GainAuto, MV_GAIN_AUTO_CONTINUOUS);
    _nh->param("AutoGainLowerLimit", _HIKCAMERA_PARAM.AutoGainLowerLimit, 25.f);
    _nh->param("AutoGainUpperLimit", _HIKCAMERA_PARAM.AutoGainUpperLimit, 0.f);

    _nh->param("Brightness", _HIKCAMERA_PARAM.Brightness, 100);

    /*Digital IO Control*/
    _nh->param("LineSelector", _HIKCAMERA_PARAM.LineSelector, std::vector<MV_LINE_SELECTOR>{MV_LINE_SELECTOR_Line1});
    _nh->param("LineMode", _HIKCAMERA_PARAM.LineMode, std::vector<MV_LINE_MODE>{MV_LINE_MODE_Strobe});
    _nh->param("LineDebouncerTime", _HIKCAMERA_PARAM.LineDebouncerTime, std::vector<int>{50});
    _nh->param("StrobeEnable", _HIKCAMERA_PARAM.StrobeEnable, std::vector<MV_STROBE_ENABLE>{MV_STROBE_ENABLE_OFF});
    _nh->param("LineSource", _HIKCAMERA_PARAM.LineSource, std::vector<MV_LINE_SOURCE>{MV_LINE_SOURCE_ExposureStartActive});
    _nh->param("StrobeLineDuration", _HIKCAMERA_PARAM.StrobeLineDuration, std::vector<int>{0});
    _nh->param("StrobeLineDelay", _HIKCAMERA_PARAM.StrobeLineDelay, std::vector<int>{0});
    _nh->param("StrobeLinePreDelay", _HIKCAMERA_PARAM.StrobeLinePreDelay, std::vector<int>{0});

    /*Transport Layer Control*/
    _nh->param("GevIEEE1588", _HIKCAMERA_PARAM.GevIEEE1588, MV_GEV_IEEE_1588_ON);

}

int Hikcamera::setParamFromRosServer() {

    loadRosServerParam();

    int nRet = MV_OK;
    nRet |= setImageFormat(_HIKCAMERA_PARAM.Width, _HIKCAMERA_PARAM.Height, _HIKCAMERA_PARAM.OffsetX, _HIKCAMERA_PARAM.OffsetY,
                            _HIKCAMERA_PARAM.PixelFormat, _HIKCAMERA_PARAM.ImageCompressionMode);
    nRet |= setAcquisitionRate(_HIKCAMERA_PARAM.AcquisitionLineRateEnable, _HIKCAMERA_PARAM.AcquisitionLineRate);
    nRet |= setTrigger(_HIKCAMERA_PARAM.TriggerMode, _HIKCAMERA_PARAM.TriggerSource, _HIKCAMERA_PARAM.TriggerActivation);
    nRet |= setExplosure(_HIKCAMERA_PARAM.ExposureAuto, _HIKCAMERA_PARAM.ExposureTime, 
                        _HIKCAMERA_PARAM.AutoExposureTimeLowerLimit, _HIKCAMERA_PARAM.AutoExposureTimeUpperLimit);

    nRet |= setGain(_HIKCAMERA_PARAM.GainAuto, _HIKCAMERA_PARAM.Gain, _HIKCAMERA_PARAM.AutoGainLowerLimit, _HIKCAMERA_PARAM.AutoGainUpperLimit);
    nRet |= setBrightness(_HIKCAMERA_PARAM.Brightness);
    
    nRet |= setLineIOBatch(_HIKCAMERA_PARAM.LineSelector, _HIKCAMERA_PARAM.LineMode,
                        _HIKCAMERA_PARAM.LineDebouncerTime,
                        _HIKCAMERA_PARAM.StrobeEnable, _HIKCAMERA_PARAM.LineSource,
                        _HIKCAMERA_PARAM.StrobeLineDuration, _HIKCAMERA_PARAM.StrobeLineDelay, _HIKCAMERA_PARAM.StrobeLinePreDelay);

    nRet |= setTransportLayerControl(_HIKCAMERA_PARAM.GevIEEE1588);

    return nRet;
}

int Hikcamera::setImageFormat(int Width, int Height, int OffsetX, int OffsetY, 
                            MV_PIXEL_FORMAT PixelFormat, MV_IMAGE_COMPRESSION_MODE ImageCompressionMode) 
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetIntValue(_handle, "Width", Width);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set Width fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "Height", Height);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set Height fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "OffsetX", OffsetX);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set OffsetX fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "OffsetY", OffsetY);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set OffsetY fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetEnumValue(_handle, "PixelFormat", PixelFormat);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set PixelFormat Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetEnumValue(_handle, "ImageCompressionMode", ImageCompressionMode);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set ImageCompressionMode Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    return nRet_t;
}

int Hikcamera::setAcquisitionRate(MV_ACQUISITION_LINE_RATE_ENABLE AcquisitionLineRateEnable, int AcquisitionLineRate)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetBoolValue(_handle, "AcquisitionLineRateEnable", AcquisitionLineRateEnable);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set AcquisitionLineRateEnable Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    if (AcquisitionLineRateEnable == MV_ACQUISITION_LINE_RATE_ENABLE_ON) {
        nRet = MV_CC_SetIntValue(_handle, "AcquisitionLineRate", AcquisitionLineRate);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set AcquisitionLineRate Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setTrigger(MV_TRIGGER_MODE TriggerMode, MV_TRIGGER_SOURCE TriggerSource, MV_TRIGGER_ACTIVATION TriggerActivation)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", TriggerMode);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set TriggerMode Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    if (TriggerSource == MV_TRIGGER_SOURCE_Action1_SPECIAL) {

        nRet = MV_CC_SetEnumValueByString(_handle, "TriggerSource", "Action1");
        if (MV_OK != nRet)
        {
            ROS_WARN("Set Trigger Source fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "ActionDeviceKey", ActionDeviceKey);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set Action Device Key fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "ActionGroupKey", ActionGroupKey);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set Action Group Key fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "ActionGroupMask", ActionGroupMask);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set Action Group Mask fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

    } else {

        nRet = MV_CC_SetEnumValue(_handle, "TriggerSource", TriggerSource);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set TriggerSource Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }
        
        nRet = MV_CC_SetEnumValue(_handle, "TriggerActivation", TriggerActivation);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set TriggerActivation Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setExplosure(MV_EXPOSURE_AUTO ExposureAuto, float ExposureTime, 
                            int AutoExposureTimeLowerLimit, int AutoExposureTimeUpperLimit)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "ExposureAuto", ExposureAuto);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set ExposureAuto Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetFloatValue(_handle, "ExposureTime", ExposureTime);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set ExposureTime Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "AutoExposureTimeLowerLimit", AutoExposureTimeLowerLimit);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set AutoExposureTimeLowerLimit Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "AutoExposureTimeUpperLimit", AutoExposureTimeUpperLimit);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set AutoExposureTimeUpperLimit Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    return nRet_t;
}

int Hikcamera::setGain(MV_GAIN_AUTO GainAuto, float Gain, float AutoGainLowerLimit, float AutoGainUpperLimit)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "GainAuto", GainAuto);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set GainAuto Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetFloatValue(_handle, "Gain", Gain);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set Gain Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetFloatValue(_handle, "AutoGainLowerLimit", AutoGainLowerLimit);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set AutoGainLowerLimit Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetFloatValue(_handle, "AutoGainUpperLimit", AutoGainUpperLimit);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set AutoGainUpperLimit Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    return nRet_t;
}

int Hikcamera::setBrightness(int Brightness)
{
    int nRet = MV_OK;

    nRet = MV_CC_SetIntValue(_handle, "Brightness", Brightness);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set Brightness Mode fail! nRet [0x%x]\n", nRet);
    }

    return nRet;
}

int Hikcamera::setLineIO(MV_LINE_SELECTOR LineSelector, MV_LINE_MODE LineMode, 
                        int LineDebouncerTime,
                        MV_STROBE_ENABLE StrobeEnable, MV_LINE_SOURCE LineSource, 
                        int StrobeLineDuration, int StrobeLineDelay, int StrobeLinePreDelay)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "LineSelector", LineSelector);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set LineSelector Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetEnumValue(_handle, "LineMode", LineMode);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set LineMode Mode fail! nRet [0x%x]\n", nRet);
        nRet_t |= nRet;
    }

    if (LineMode == MV_LINE_MODE_Input) {

        nRet = MV_CC_SetIntValue(_handle, "LineDebouncerTime", LineDebouncerTime);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set LineDebouncerTime Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

    } else if (LineMode == MV_LINE_MODE_Strobe) {

        nRet = MV_CC_SetEnumValue(_handle, "LineSource", LineSource);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set LineSource Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "StrobeLineDuration", StrobeLineDuration);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set StrobeLineDuration Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "StrobeLineDelay", StrobeLineDelay);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set StrobeLineDelay Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "StrobeLinePreDelay", StrobeLinePreDelay);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set StrobeLinePreDelay Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetEnumValue(_handle, "StrobeEnable", StrobeEnable);
        if (MV_OK != nRet)
        {
            ROS_WARN("Set StrobeEnable Mode fail! nRet [0x%x]\n", nRet);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setLineIOBatch(std::vector<MV_LINE_SELECTOR> LineSelector, std::vector<MV_LINE_MODE> LineMode,
                                std::vector<int> LineDebouncerTime,
                                std::vector<MV_STROBE_ENABLE> StrobeEnable, std::vector<MV_LINE_SOURCE> LineSource,
                                std::vector<int> StrobeLineDuration, std::vector<int> StrobeLineDelay, std::vector<int> StrobeLinePreDelay)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    for (int i = 0; i < LineSelector.size(); i++) {
        nRet = setLineIO(LineSelector[i], LineMode[i], 
                            LineDebouncerTime[i],
                            StrobeEnable[i], LineSource[i], 
                            StrobeLineDuration[i], StrobeLineDelay[i], StrobeLinePreDelay[i]);

        if (nRet != MV_OK) {
            ROS_WARN("Incomplete Line [%d] Settings!\n", LineSelector[i]);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setTransportLayerControl(MV_GEV_IEEE_1588 GevIEEE1588)
{
    int nRet = MV_OK;

    MV_CC_SetBoolValue(_handle, "GevIEEE1588", GevIEEE1588);
    if (MV_OK != nRet)
    {
        ROS_WARN("Set GevIEEE1588 Mode fail! nRet [0x%x]\n", nRet);
    }

    return nRet;
}


int Hikcamera::startGrabbing() 
{
    int nRet = MV_OK;

    nRet = MV_CC_StartGrabbing(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Start Grabbing fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
}



// bool Hikcamera::init() {

//     int nRet;
//     nRet = MV_CC_Initialize();
//     if (MV_OK != nRet)
//     {
//         printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
//         return nRet;
//     }
    
    
// }



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

bool Hikcamera::_printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
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