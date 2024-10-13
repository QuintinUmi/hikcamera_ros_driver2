#include "Hikcamera.h"
#include <iostream> 

using namespace hikcamera_ros_driver2;

Hikcamera::Hikcamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index) : _pDeviceInfo(pDeviceInfo), _camera_index(camera_index)
{

}

Hikcamera::Hikcamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index, ros::NodeHandle* nh) : 
                    _pDeviceInfo(pDeviceInfo), _camera_index(camera_index), _nh(nh), _work_thread_mode(WORK_THREAD_MODE_NOT_SELECTED)
{
    
}

Hikcamera::~Hikcamera() 
{
    deinitDevice();
}


int Hikcamera::init(bool set_param_from_ros) {

    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = initDevice();
    if (nRet != MV_OK) {
        ROS_WARN("Device [%d]: Incomplete initialization!", _camera_index);
        nRet_t |= nRet;
    }

    loadRosConfig();
    
    if (set_param_from_ros) {
        nRet = setParamFromRosServer();
        if (nRet != MV_OK) {
            ROS_WARN("Device [%d]: Incomplete setParamFromRosServer!", _camera_index);
            nRet_t |= nRet;
        }
        nRet = initWorkThread(WORK_THREAD_MODE_ROS_PUBLISH);
        if (nRet != MV_OK) {
            ROS_WARN("Device [%d]: Incomplete initWorkThread!", _camera_index);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}


int Hikcamera::initDevice()
{
    int nRet = MV_OK;
    nRet = MV_CC_CreateHandle(&_handle, _pDeviceInfo);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Device [%d]: Create Handle fail! nRet [0x%x]\n", _camera_index, nRet);
        return nRet;
    }

    nRet = MV_CC_OpenDevice(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Device [%d]: Open Device fail! nRet [0x%x]\n", _camera_index, nRet);
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
                ROS_WARN("Device [%d]: Warning: Set Packet Size fail nRet [0x%x]!", _camera_index, nRet);
            }
        }
        else
        {
            ROS_WARN("Device [%d]: Warning: Get Packet Size fail nRet [0x%x]!", _camera_index, _nPacketSize);
        }
    }

    return nRet;
}

int Hikcamera::deinitDevice()
{
    int nRet = MV_OK;
    nRet = MV_CC_CloseDevice(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Device [%d]: Close Device fail! nRet [0x%x]\n", _camera_index, nRet);
        return nRet;
    }

    nRet = MV_CC_DestroyHandle(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Device [%d]: Destroy Handle fail! nRet [0x%x]\n", _camera_index, nRet);
        return nRet;
    }

    return nRet;
}

int Hikcamera::initWorkThread(WORK_THREAD_MODE work_thread_mode) {

    releaseAllWorkThread();

    _work_thread_mode = work_thread_mode;

    if (_work_thread_mode == WORK_THREAD_MODE_NOT_SELECTED) {

        return -1;
    }

    _get_frame_wt = std::make_shared<std::thread>(std::bind(&Hikcamera::GetFrameWorkThread, this));
    if (_work_thread_mode == WORK_THREAD_MODE_ROS_PUBLISH) {
        _ros_publish_wt = std::make_shared<std::thread>(std::bind(&Hikcamera::RosPublishWorkThread, this));
    } else if (_work_thread_mode == WORK_THREAD_MODE_IMAGE_CALLBACK) {
        _image_received_callback_wt = std::make_shared<std::thread>(std::bind(&Hikcamera::ImageReceivedCallBackWorkThread, this));
    }

    return 0;
}

int Hikcamera::releaseAllWorkThread() {
    
    _start_get_frame_wt = false;
    _start_ros_publish_wt = false;
    _start_image_received_callback_wt = false;
    _exit_get_frame_wt = true;
    _exit_ros_publish_wt = true;
    _exit_image_received_callback_wt = true;
    
    if (_get_frame_wt) {
        _get_frame_wt->join();
        _get_frame_wt = nullptr;
    }
    if (_ros_publish_wt) {
        _ros_publish_wt->join();
        _ros_publish_wt = nullptr;
    }
    if (_image_received_callback_wt) {
        _image_received_callback_wt->join();
        _image_received_callback_wt = nullptr;
    }

    return 0;
}

int32_t Hikcamera::SetImageReceivedCb(ImageReceivedCb cb, void *data) {

    if ((cb != nullptr) || (data != nullptr)) {
        _image_received_cb = cb;
        _client_data = data;
        return 0;
    } else {
        return -1;
    }
}

void Hikcamera::loadParamFromRosServer() 
{

    /*Image Format Control*/
    int PixelFormat, ImageCompressionMode;

    _nh->param("Width", _HIKCAMERA_PARAM.Width, 2448);
    _nh->param("Height", _HIKCAMERA_PARAM.Height, 2048);
    _nh->param("OffsetX", _HIKCAMERA_PARAM.OffsetX, 0);
    _nh->param("OffsetY", _HIKCAMERA_PARAM.OffsetY, 0);
    _nh->param("PixelFormat", PixelFormat, (int)HIK_PIXEL_FORMAT_BayerRG8);
    _nh->param("ImageCompressionMode", ImageCompressionMode, (int)HIK_IMAGE_COMPRESSION_MODE_HB);

    _HIKCAMERA_PARAM.PixelFormat = static_cast<HIK_PIXEL_FORMAT>(PixelFormat);
    _HIKCAMERA_PARAM.ImageCompressionMode = static_cast<HIK_IMAGE_COMPRESSION_MODE>(ImageCompressionMode);

    /*Acquisition Control*/
    int AcquisitionLineRateEnable, TriggerMode, TriggerSource, TriggerActivation, ExposureAuto;

    _nh->param("AcquisitionLineRate", _HIKCAMERA_PARAM.AcquisitionLineRate, 20);
    _nh->param("AcquisitionLineRateEnable", AcquisitionLineRateEnable, (int)HIK_ACQUISITION_LINE_RATE_ENABLE_ON);

    _nh->param("TriggerMode", TriggerMode, (int)HIK_TRIGGER_MODE_ON);
    _nh->param("TriggerSource", TriggerSource, (int)HIK_TRIGGER_SOURCE_Action1_SPECIAL);
    _nh->param("TriggerActivation", TriggerActivation, (int)HIK_TRIGGER_ACTIVATION_RisingEdge);

    _nh->param("ExposureTime", _HIKCAMERA_PARAM.ExposureTime, 15000.f);
    _nh->param("ExposureAuto", ExposureAuto, (int)HIK_EXPOSURE_AUTO_CONTINUOUS);
    _nh->param("AutoExposureTimeLowerLimit", _HIKCAMERA_PARAM.AutoExposureTimeLowerLimit, 5000);
    _nh->param("AutoExposureTimeUpperLimit", _HIKCAMERA_PARAM.AutoExposureTimeUpperLimit, 45000);

    _HIKCAMERA_PARAM.AcquisitionLineRateEnable = static_cast<HIK_ACQUISITION_LINE_RATE_ENABLE>(AcquisitionLineRateEnable);
    _HIKCAMERA_PARAM.TriggerMode = static_cast<HIK_TRIGGER_MODE>(TriggerMode);
    _HIKCAMERA_PARAM.TriggerSource = static_cast<HIK_TRIGGER_SOURCE>(TriggerSource);
    _HIKCAMERA_PARAM.TriggerActivation = static_cast<HIK_TRIGGER_ACTIVATION>(TriggerActivation);
    _HIKCAMERA_PARAM.ExposureAuto = static_cast<HIK_EXPOSURE_AUTO>(ExposureAuto);

    /*Analog Control*/
    int GainAuto;

    _nh->param("Gain", _HIKCAMERA_PARAM.Gain, 25.f);
    _nh->param("GainAuto", GainAuto, (int)HIK_GAIN_AUTO_CONTINUOUS);
    _nh->param("AutoGainLowerLimit", _HIKCAMERA_PARAM.AutoGainLowerLimit, 0.f);
    _nh->param("AutoGainUpperLimit", _HIKCAMERA_PARAM.AutoGainUpperLimit, 23.9812f);

    _nh->param("Brightness", _HIKCAMERA_PARAM.Brightness, 100);

    _HIKCAMERA_PARAM.GainAuto = (HIK_GAIN_AUTO)GainAuto;

    /*Digital IO Control*/
    std::vector<int> LineSelector, LineMode, StrobeEnable, LineSource;

    _nh->param("LineSelector", LineSelector, std::vector<int>{(int)HIK_LINE_SELECTOR_Line1});
    _nh->param("LineMode", LineMode, std::vector<int>{(int)HIK_LINE_MODE_Strobe});
    _nh->param("LineDebouncerTime", _HIKCAMERA_PARAM.LineDebouncerTime, std::vector<int>{50});
    _nh->param("StrobeEnable", StrobeEnable, std::vector<int>{(int)HIK_STROBE_ENABLE_OFF});
    _nh->param("LineSource", LineSource, std::vector<int>{(int)HIK_LINE_SOURCE_ExposureStartActive});
    _nh->param("StrobeLineDuration", _HIKCAMERA_PARAM.StrobeLineDuration, std::vector<int>{0});
    _nh->param("StrobeLineDelay", _HIKCAMERA_PARAM.StrobeLineDelay, std::vector<int>{0});
    _nh->param("StrobeLinePreDelay", _HIKCAMERA_PARAM.StrobeLinePreDelay, std::vector<int>{0});

    for (int num : LineSelector) 
        _HIKCAMERA_PARAM.LineSelector.push_back(static_cast<HIK_LINE_SELECTOR>(num));
    for (int num : LineMode) 
        _HIKCAMERA_PARAM.LineMode.push_back(static_cast<HIK_LINE_MODE>(num));
    for (int num : StrobeEnable) 
        _HIKCAMERA_PARAM.StrobeEnable.push_back(static_cast<HIK_STROBE_ENABLE>(num));
    for (int num : LineSource) 
        _HIKCAMERA_PARAM.LineSource.push_back(static_cast<HIK_LINE_SOURCE>(num));

    /*Action Control*/
    _nh->param("ActionDeviceKey", _HIKCAMERA_PARAM.ActionDeviceKey, 0x00000001);
    _nh->param("ActionGroupMask", _HIKCAMERA_PARAM.ActionGroupMask, 0x00000001);
    _nh->param("ActionGroupKey", _HIKCAMERA_PARAM.ActionGroupKey, 0x00000001);

    /*Transport Layer Control*/
    int GevIEEE1588;

    _nh->param("GevIEEE1588", GevIEEE1588, (int)HIK_GEV_IEEE_1588_ON);

    _HIKCAMERA_PARAM.GevIEEE1588 = static_cast<HIK_GEV_IEEE_1588>(GevIEEE1588);
}

void Hikcamera::loadRosConfig() {

    _nh->param("image_publish_topic", _ROS_PARAM.image_publish_topic, std::string("/hikcamera/image_"));
    _nh->param("publish_compressed", _ROS_PARAM.publish_compressed, true);

}

int Hikcamera::setParamFromRosServer() {

    loadParamFromRosServer();

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
                            HIK_PIXEL_FORMAT PixelFormat, HIK_IMAGE_COMPRESSION_MODE ImageCompressionMode) 
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetIntValue(_handle, "Width", Width);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set Width fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "Height", Height);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set Height fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "OffsetX", OffsetX);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set OffsetX fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "OffsetY", OffsetY);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set OffsetY fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetEnumValue(_handle, "PixelFormat", PixelFormat);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set PixelFormat fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetEnumValue(_handle, "ImageCompressionMode", ImageCompressionMode);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set ImageCompressionMode fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    return nRet_t;
}

int Hikcamera::setAcquisitionRate(HIK_ACQUISITION_LINE_RATE_ENABLE AcquisitionLineRateEnable, int AcquisitionLineRate)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetBoolValue(_handle, "AcquisitionFrameRateEnable", AcquisitionLineRateEnable);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set AcquisitionFrameRateEnable fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    if (AcquisitionLineRateEnable == HIK_ACQUISITION_LINE_RATE_ENABLE_ON) {
        nRet = MV_CC_SetFloatValue(_handle, "AcquisitionFrameRate", AcquisitionLineRate);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set AcquisitionFrameRate fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setTrigger(HIK_TRIGGER_MODE TriggerMode, HIK_TRIGGER_SOURCE TriggerSource, HIK_TRIGGER_ACTIVATION TriggerActivation)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", TriggerMode);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set TriggerMode fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    if (TriggerSource == HIK_TRIGGER_SOURCE_Action1_SPECIAL) {

        nRet = MV_CC_SetEnumValueByString(_handle, "TriggerSource", "Action1");
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set Trigger Source fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "ActionDeviceKey", _HIKCAMERA_PARAM.ActionDeviceKey);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set Action Device Key fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "ActionGroupKey", _HIKCAMERA_PARAM.ActionGroupKey);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set Action Group Key fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "ActionGroupMask", _HIKCAMERA_PARAM.ActionGroupMask);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set Action Group Mask fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

    } else {

        nRet = MV_CC_SetEnumValue(_handle, "TriggerSource", TriggerSource);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set TriggerSource fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
        
        nRet = MV_CC_SetEnumValue(_handle, "TriggerActivation", TriggerActivation);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set TriggerActivation fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setExplosure(HIK_EXPOSURE_AUTO ExposureAuto, float ExposureTime, 
                            int AutoExposureTimeLowerLimit, int AutoExposureTimeUpperLimit)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "ExposureAuto", ExposureAuto);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set ExposureAuto fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    if (ExposureAuto != HIK_EXPOSURE_AUTO_CONTINUOUS) {
        nRet = MV_CC_SetFloatValue(_handle, "ExposureTime", ExposureTime);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set ExposureTime fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
    }

    nRet = MV_CC_SetIntValue(_handle, "AutoExposureTimeLowerLimit", AutoExposureTimeLowerLimit);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set AutoExposureTimeLowerLimit fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetIntValue(_handle, "AutoExposureTimeUpperLimit", AutoExposureTimeUpperLimit);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set AutoExposureTimeUpperLimit fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    return nRet_t;
}

int Hikcamera::setGain(HIK_GAIN_AUTO GainAuto, float Gain, float AutoGainLowerLimit, float AutoGainUpperLimit)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "GainAuto", GainAuto);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set GainAuto fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    if (GainAuto != HIK_GAIN_AUTO_CONTINUOUS) {
        nRet = MV_CC_SetFloatValue(_handle, "Gain", Gain);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set Gain fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
    }

    nRet = MV_CC_SetFloatValue(_handle, "AutoGainLowerLimit", AutoGainLowerLimit);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set AutoGainLowerLimit fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    nRet = MV_CC_SetFloatValue(_handle, "AutoGainUpperLimit", AutoGainUpperLimit);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set AutoGainUpperLimit fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    return nRet_t;
}

int Hikcamera::setBrightness(int Brightness)
{
    int nRet = MV_OK;

    nRet = MV_CC_SetIntValue(_handle, "Brightness", Brightness);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set Brightness fail! nRet [0x%x]\n", _camera_index, nRet);
    }

    return nRet;
}

int Hikcamera::setLineIO(HIK_LINE_SELECTOR LineSelector, HIK_LINE_MODE LineMode, 
                        int LineDebouncerTime,
                        HIK_STROBE_ENABLE StrobeEnable, HIK_LINE_SOURCE LineSource, 
                        int StrobeLineDuration, int StrobeLineDelay, int StrobeLinePreDelay)
{
    int nRet = MV_OK;
    int nRet_t = MV_OK;

    nRet = MV_CC_SetEnumValue(_handle, "LineSelector", LineSelector);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set LineSelector fail! nRet [0x%x]\n", _camera_index, nRet);
        nRet_t |= nRet;
    }

    if (LineSelector != HIK_LINE_SELECTOR_Line0) {
        nRet = MV_CC_SetEnumValue(_handle, "LineMode", LineMode);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set LineMode fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
    }

    if (LineMode == HIK_LINE_MODE_Input) {

        nRet = MV_CC_SetIntValue(_handle, "LineDebouncerTime", LineDebouncerTime);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set LineDebouncerTime fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

    } else if (LineMode == HIK_LINE_MODE_Strobe) {

        nRet = MV_CC_SetEnumValue(_handle, "LineSource", LineSource);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set LineSource fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "StrobeLineDuration", StrobeLineDuration);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set StrobeLineDuration fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "StrobeLineDelay", StrobeLineDelay);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set StrobeLineDelay fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetIntValue(_handle, "StrobeLinePreDelay", StrobeLinePreDelay);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set StrobeLinePreDelay fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }

        nRet = MV_CC_SetBoolValue(_handle, "StrobeEnable", StrobeEnable);
        if (MV_OK != nRet) {
            ROS_WARN("Device [%d]: Set StrobeEnable fail! nRet [0x%x]\n", _camera_index, nRet);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setLineIOBatch(std::vector<HIK_LINE_SELECTOR> LineSelector, std::vector<HIK_LINE_MODE> LineMode,
                                std::vector<int> LineDebouncerTime,
                                std::vector<HIK_STROBE_ENABLE> StrobeEnable, std::vector<HIK_LINE_SOURCE> LineSource,
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
            ROS_WARN("Device [%d]: Incomplete Line [%d] Settings!\n", _camera_index, LineSelector[i]);
            nRet_t |= nRet;
        }
    }

    return nRet_t;
}

int Hikcamera::setTransportLayerControl(HIK_GEV_IEEE_1588 GevIEEE1588)
{
    int nRet = MV_OK;

    MV_CC_SetBoolValue(_handle, "GevIEEE1588", GevIEEE1588);
    if (MV_OK != nRet) {
        ROS_WARN("Device [%d]: Set GevIEEE1588 fail! nRet [0x%x]\n", _camera_index, nRet);
    }

    return nRet;
}


int Hikcamera::startGrabbing() 
{
    int nRet = MV_OK;

    if (_work_thread_mode == WORK_THREAD_MODE_NOT_SELECTED) {
        ROS_ERROR("Device [%d]: _work_thread_mode == WORK_THREAD_MODE_NOT_SELECTED\n", _camera_index);
    }

    nRet = MV_CC_StartGrabbing(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Device [%d]: Start Grabbing fail! nRet [0x%x]\n", _camera_index, nRet);
        return nRet;
    }

    _start_get_frame_wt = true;
    _exit_get_frame_wt = false;
    if (_work_thread_mode == WORK_THREAD_MODE_ROS_PUBLISH) {
        _start_ros_publish_wt = true;
        _exit_ros_publish_wt = false;
    } else if (_work_thread_mode == WORK_THREAD_MODE_IMAGE_CALLBACK) {
        _start_image_received_callback_wt = true;
        _exit_image_received_callback_wt = false;
    } else {
        ROS_ERROR("Device [%d]: Work Thread is NOT started\n", _camera_index);
        return -1;
    }

    return nRet;
}

int Hikcamera::stopGrabbing()
{
    releaseAllWorkThread();

    int nRet = MV_OK;

    nRet = MV_CC_StopGrabbing(_handle);
    if (MV_OK != nRet)
    {
        ROS_ERROR("Device [%d]: Start Grabbing fail! nRet [0x%x]\n", _camera_index, nRet);
        return nRet;
    }

    return nRet;
}


void Hikcamera::GetFrameWorkThread() {
    
    int nRet = MV_OK;

    unsigned char * pDstBuf = NULL;
    MV_FRAME_OUT stFrameDecBuf;
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(_handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return;
    }

    ROS_WARN("Device [%d]: Init GetFrameWorkThread", _camera_index);
    while (!_start_get_frame_wt) {
        /* waiting to start */
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ROS_WARN("Device [%d]: Start GetFrameWorkThread", _camera_index);
    
    while(!_exit_get_frame_wt) {

        int nRet = MV_OK;
        static MV_FRAME_OUT stFrameOut = {0};
        static uint64_t seq = 1;
        static auto last_rcv_time = std::chrono::high_resolution_clock::now();

        // nRet = MV_CC_GetImageBuffer(_handle, &stFrameOut, (1000.f / _HIKCAMERA_PARAM.AcquisitionLineRate) * 1.5); 
        nRet = MV_CC_GetImageBuffer(_handle, &stFrameOut, 1000); 
        if (nRet != MV_OK) {
            ROS_ERROR("Device [%d]: Get Image fail! nRet [0x%x]\n", _camera_index, nRet);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        auto rcv_time = std::chrono::high_resolution_clock::now();

        auto seq_interval = std::chrono::nanoseconds(1000000000 / _HIKCAMERA_PARAM.AcquisitionLineRate);
        seq += ((rcv_time - last_rcv_time) + seq_interval / 2) / seq_interval;

        // Log retrieved frame information
        std::string debug_msg;
        debug_msg = "Device [" + std::to_string(_camera_index) + "] GetOneFrame,nFrameNum[" +
                    std::to_string(stFrameOut.stFrameInfo.nFrameNum) + "], DeviceTimeStamp: [" +
                    std::to_string(nDevTimeStampCov(stFrameOut.stFrameInfo.nDevTimeStampHigh, stFrameOut.stFrameInfo.nDevTimeStampLow)) + "], Width: [" +
                    std::to_string(stFrameOut.stFrameInfo.nWidth) + "], Height: [" +
                    std::to_string(stFrameOut.stFrameInfo.nHeight) + "], nFrameLen: [" +
                    std::to_string(stFrameOut.stFrameInfo.nFrameLen)+ "]\n";
        if (IS_SHOW_FRAME_INFO) {
            ROS_INFO_STREAM(debug_msg.c_str());
        }

        if (_HIKCAMERA_PARAM.ImageCompressionMode == HIK_IMAGE_COMPRESSION_MODE_HB) {
            MV_CC_HB_DECODE_PARAM stDecodeParam = {0};
            // Lossless compression decoding
            stDecodeParam.pSrcBuf = stFrameOut.pBufAddr;
            stDecodeParam.nSrcLen = stFrameOut.stFrameInfo.nFrameLen;
            if (pDstBuf == NULL)
            {
                pDstBuf = (unsigned char *)malloc(sizeof(unsigned char) * (stParam.nCurValue));
                if (NULL == pDstBuf)
                {
                    ROS_ERROR("malloc pDstData fail !\n");
                    nRet = MV_E_RESOURCE;
                    break;
                }
            }
            stDecodeParam.pDstBuf = pDstBuf;
            stDecodeParam.nDstBufSize = stParam.nCurValue;
            nRet = MV_CC_HB_Decode(_handle, &stDecodeParam);
            if (nRet != MV_OK) {
                ROS_ERROR("Device [%d]: In Frame [%d] | Seq [%ld] | TimeStamp [%ld], MV_CC_HB_Decode fail! nRet [0x%x]\n", _camera_index, 
                            stFrameOut.stFrameInfo.nFrameNum, seq, nDevTimeStampCov(stFrameOut.stFrameInfo.nDevTimeStampHigh, stFrameOut.stFrameInfo.nDevTimeStampLow),
                            nRet);

                if (stFrameOut.pBufAddr != NULL) {
                    nRet = MV_CC_FreeImageBuffer(_handle, &stFrameOut);
                    if (nRet != MV_OK) {
                        ROS_ERROR("Device [%d]: Free Image Buffer fail! nRet [0x%x]\n", _camera_index, nRet);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            stFrameDecBuf.pBufAddr = stDecodeParam.pDstBuf;
            stFrameDecBuf.stFrameInfo.nWidth = stDecodeParam.nWidth;
            stFrameDecBuf.stFrameInfo.nHeight = stDecodeParam.nHeight;

            cv::Mat cv_image = imageMvToCv(stFrameDecBuf, _HIKCAMERA_PARAM.PixelFormat), output;
            _ImageQueue.push(ImageQueuePacket(cv_image, stFrameOut.stFrameInfo, seq));

        } else {
            cv::Mat cv_image = imageMvToCv(stFrameOut, _HIKCAMERA_PARAM.PixelFormat), output;
            _ImageQueue.push(ImageQueuePacket(cv_image, stFrameOut.stFrameInfo, seq));
        }
        

        if (stFrameOut.pBufAddr != NULL) {
            nRet = MV_CC_FreeImageBuffer(_handle, &stFrameOut);
            if (nRet != MV_OK) {
                ROS_ERROR("Device [%d]: Free Image Buffer fail! nRet [0x%x]\n", _camera_index, nRet);
            }
        }

    }
    ROS_WARN("Device [%d]: Stop GetFrameWorkThread", _camera_index);
}

void Hikcamera::RosPublishWorkThread() {

    image_transport::ImageTransport it(*_nh);
    image_transport::Publisher pub = it.advertise(_ROS_PARAM.image_publish_topic + std::to_string(_camera_index), 1);

    ROS_WARN("Device [%d]: Init RosPublishWorkThread", _camera_index);
    while (!_start_ros_publish_wt) {
        /* waiting to start */
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ROS_WARN("Device [%d]: Start RosPublishWorkThread", _camera_index);
    
    while(!_exit_ros_publish_wt) {
        
        auto imagePacket = _ImageQueue.wait_and_pop();
        if (imagePacket) {
            sensor_msgs::ImagePtr msg = imageCvToRos(imagePacket->image, _HIKCAMERA_PARAM.PixelFormat);
            pub.publish(msg);
        }

    }
    ROS_WARN("Device [%d]: Stop RosPublishWorkThread", _camera_index);

}

void Hikcamera::ImageReceivedCallBackWorkThread()
{
    ROS_WARN("Device [%d]: Init ImageReceivedCallBackWorkThread", _camera_index);
    while (!_start_image_received_callback_wt) {
        /* waiting to start */
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ROS_WARN("Device [%d]: Start ImageReceivedCallBackWorkThread", _camera_index);
    
    while(!_exit_image_received_callback_wt) {

        auto imagePacket = _ImageQueue.wait_and_pop();
        if (imagePacket) {
            _image_received_cb(imagePacket->image, 
                            nDevTimeStampCov(imagePacket->stFrameInfo.nDevTimeStampHigh, imagePacket->stFrameInfo.nDevTimeStampLow),
                            imagePacket->seq);
        }

    }
    ROS_WARN("Device [%d]: Stop ImageReceivedCallBackWorkThread", _camera_index);
}

cv::Mat Hikcamera::imageMvToCv(const MV_FRAME_OUT& stFrameOut, HIK_PIXEL_FORMAT mv_format) {
    switch (mv_format) {
        case HIK_PIXEL_FORMAT_Mono8:
            return cv::Mat(stFrameOut.stFrameInfo.nHeight, stFrameOut.stFrameInfo.nWidth, CV_8UC1, stFrameOut.pBufAddr);
        case HIK_PIXEL_FORMAT_Mono10:
        case HIK_PIXEL_FORMAT_Mono12:
        case HIK_PIXEL_FORMAT_Mono16:
            return cv::Mat(stFrameOut.stFrameInfo.nHeight, stFrameOut.stFrameInfo.nWidth, CV_16UC1, stFrameOut.pBufAddr);
        case HIK_PIXEL_FORMAT_RGB8Packed:
            return cv::Mat(stFrameOut.stFrameInfo.nHeight, stFrameOut.stFrameInfo.nWidth, CV_8UC3, stFrameOut.pBufAddr);
        case HIK_PIXEL_FORMAT_YUV422_8:
        case HIK_PIXEL_FORMAT_YUV422_8_UYVY:
            return cv::Mat(stFrameOut.stFrameInfo.nHeight, stFrameOut.stFrameInfo.nWidth, CV_8UC2, stFrameOut.pBufAddr);
        case HIK_PIXEL_FORMAT_BayerGR8:
        case HIK_PIXEL_FORMAT_BayerRG8:
        case HIK_PIXEL_FORMAT_BayerGB8:
        case HIK_PIXEL_FORMAT_BayerBG8:
            return cv::Mat(stFrameOut.stFrameInfo.nHeight, stFrameOut.stFrameInfo.nWidth, CV_8UC1, stFrameOut.pBufAddr);
        default:
            throw std::runtime_error("Unsupported pixel format");
    }
}

sensor_msgs::ImagePtr Hikcamera::imageCvToRos(const cv::Mat& cv_image, HIK_PIXEL_FORMAT mv_format) {

    cv::Mat output;
    switch (mv_format) {
        case HIK_PIXEL_FORMAT_Mono8:
            output = cv_image;  // 直接使用输入的单通道 8 位灰度图像
            break;
        case HIK_PIXEL_FORMAT_Mono10:
        case HIK_PIXEL_FORMAT_Mono12:
        case HIK_PIXEL_FORMAT_Mono16:
            {
                // 计算最大值
                double maxVal = (mv_format == HIK_PIXEL_FORMAT_Mono10) ? 1023.0 :
                                (mv_format == HIK_PIXEL_FORMAT_Mono12) ? 4095.0 : 65535.0;
                cv_image.convertTo(output, CV_8U, 255.0 / maxVal);  // 缩放到 8 位
            }
            break;  
        case HIK_PIXEL_FORMAT_RGB8Packed:
            output = cv_image;
            break;
        case HIK_PIXEL_FORMAT_YUV422_8:
        case HIK_PIXEL_FORMAT_YUV422_8_UYVY:
            cv::cvtColor(cv_image, output, cv::COLOR_YUV2RGB_UYVY);
            break;
        case HIK_PIXEL_FORMAT_BayerGR8:
            cv::cvtColor(cv_image, output, cv::COLOR_BayerGR2RGB);
            break;
        case HIK_PIXEL_FORMAT_BayerRG8:
            cv::cvtColor(cv_image, output, cv::COLOR_BayerRG2RGB);
            break;
        case HIK_PIXEL_FORMAT_BayerGB8:
            cv::cvtColor(cv_image, output, cv::COLOR_BayerGB2RGB);
            break;
        case HIK_PIXEL_FORMAT_BayerBG8:
            cv::cvtColor(cv_image, output, cv::COLOR_BayerBG2RGB);
            break;
        case HIK_PIXEL_FORMAT_BayerGB10:
        case HIK_PIXEL_FORMAT_BayerGB12:
        case HIK_PIXEL_FORMAT_BayerGB12Packed:
            {
                cv::Mat scaled;
                cv_image.convertTo(scaled, CV_8U, 1.0 / 16.0);  
                cv::cvtColor(scaled, output, cv::COLOR_BayerGB2RGB);
            }
            break;
        default:
            throw std::runtime_error("Unsupported pixel format for RGB conversion");
    }

    switch (mv_format) {
        case HIK_PIXEL_FORMAT_Mono8:
        case HIK_PIXEL_FORMAT_Mono10:
        case HIK_PIXEL_FORMAT_Mono12:
        case HIK_PIXEL_FORMAT_Mono16:
            return cv_bridge::CvImage(std_msgs::Header(), "mono8", output).toImageMsg(); 
        case HIK_PIXEL_FORMAT_RGB8Packed:
        case HIK_PIXEL_FORMAT_YUV422_8:
        case HIK_PIXEL_FORMAT_YUV422_8_UYVY:
        case HIK_PIXEL_FORMAT_BayerGR8:
        case HIK_PIXEL_FORMAT_BayerRG8:
        case HIK_PIXEL_FORMAT_BayerGB8:
        case HIK_PIXEL_FORMAT_BayerBG8:
        case HIK_PIXEL_FORMAT_BayerGB10:
        case HIK_PIXEL_FORMAT_BayerGB12:
        case HIK_PIXEL_FORMAT_BayerGB12Packed:
            return cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg(); 
        default:
            throw std::runtime_error("Unsupported pixel format for RGB conversion");
    }
}


// nDevTimeStampHigh + nDevTimeStampLow to ns timestamp
uint64_t Hikcamera::nDevTimeStampCov(unsigned int nDevTimeStampHigh, unsigned int nDevTimeStampLow) {
    return ((uint64_t)nDevTimeStampHigh << 32) | nDevTimeStampLow;
}

// ns timestamp to date time
std::string Hikcamera::timestampToDateTime(uint64_t timestamp_ns) {
    // 将纳秒转换为秒
    auto seconds = std::chrono::seconds(timestamp_ns / 1000000000);
    // 获取剩余的纳秒
    auto nanoseconds = std::chrono::nanoseconds(timestamp_ns % 1000000000);

    // 构造时间点
    std::chrono::time_point<std::chrono::system_clock> tp(seconds + nanoseconds);

    // 转化为time_t以使用ctime获取可读时间
    std::time_t readable_time = std::chrono::system_clock::to_time_t(tp);

    // 使用gmtime或localtime转换为tm结构体，然后格式化输出
    std::tm* timeinfo = std::localtime(&readable_time);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);

    return std::string(buffer);
}

// ns timestamp to sec，format xxx.xxx
std::string Hikcamera::timestampToSeconds(uint64_t timestamp_ns) {
    // 计算整数秒
    uint64_t seconds = timestamp_ns / 1000000000;
    // 计算剩余的纳秒部分
    uint64_t remaining_ns = timestamp_ns % 1000000000;

    // 使用stringstream来构建完整格式的字符串
    std::stringstream ss;
    ss << seconds << "." << remaining_ns; // 直接拼接整数秒和剩余纳秒
    return ss.str();
}

bool Hikcamera::printDeviceInfo()
{
    if (NULL == _pDeviceInfo)
    {
        printf("The Pointer of pstHIKDevInfo is NULL!\n");
        return false;
    }
    if (_pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((_pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((_pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((_pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (_pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // print current ip and user defined name
        printf("[device_index %d]:\n", _camera_index);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , _pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}



