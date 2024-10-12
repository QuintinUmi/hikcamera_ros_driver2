#ifndef _HIKCAM_PARAM_CFG_H_
#define _HIKCAM_PARAM_CFG_H_

/*Image Format Control*/
typedef enum _MV_PIXEL_FORMAT_ // PixelFormat
{
    MV_PIXEL_FORMAT_Mono8 = 0x01080001,
    MV_PIXEL_FORMAT_Mono10 = 0x01100003,
    MV_PIXEL_FORMAT_Mono10Packed = 0x010C0004,
    MV_PIXEL_FORMAT_Mono12 = 0x01100005,
    MV_PIXEL_FORMAT_Mono12Packed = 0x010C0006, 
    MV_PIXEL_FORMAT_Mono16 = 0x01100007,
    MV_PIXEL_FORMAT_RGB8Packed = 0x02180014,
    MV_PIXEL_FORMAT_YUV422_8 = 0x02100032,
    MV_PIXEL_FORMAT_YUV422_8_UYVY = 0x0210001F,
    MV_PIXEL_FORMAT_BayerGR8 = 0x01080008,
    MV_PIXEL_FORMAT_BayerRG8 = 0x01080009,
    MV_PIXEL_FORMAT_BayerGB8 = 0x0108000A,
    MV_PIXEL_FORMAT_BayerBG8 = 0x0108000B,
    MV_PIXEL_FORMAT_BayerGB10 = 0x0110000e,
    MV_PIXEL_FORMAT_BayerGB12 = 0x01100012, 
    MV_PIXEL_FORMAT_BayerGB12Packed = 0x010C002C
}MV_PIXEL_FORMAT;


typedef enum _MV_IMAGE_COMPRESSION_MODE_ // ImageCompressionMode
{
    MV_IMAGE_COMPRESSION_MODE_OFF = 0,
    MV_IMAGE_COMPRESSION_MODE_JPEG = 1,
    MV_IMAGE_COMPRESSION_MODE_HB = 2,
}MV_IMAGE_COMPRESSION_MODE;



/*Acquisition Control*/
typedef enum _MV_ACQUISITION_LINE_RATE_ENABLE_ // AcquisitionLineRateEnable
{
    MV_ACQUISITION_LINE_RATE_ENABLE_OFF = 0,
    MV_ACQUISITION_LINE_RATE_ENABLE_ON = 1
}MV_ACQUISITION_LINE_RATE_ENABLE;


typedef enum _MV_TRIGGER_MODE_ // TriggerMode
{
    MV_TRIGGER_MODE_OFF = 0,
    MV_TRIGGER_MODE_ON = 1 
}MV_TRIGGER_MODE;


// nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_Line0);
typedef enum _MV_TRIGGER_SOURCE_ // TriggerSource
{
    MV_TRIGGER_SOURCE_Line0 = 0, 
    MV_TRIGGER_SOURCE_Line1 = 1, 
    MV_TRIGGER_SOURCE_Line2 = 2,
    MV_TRIGGER_SOURCE_Line3 = 3,
    MV_TRIGGER_SOURCE_Counter0 = 4, 
    MV_TRIGGER_SOURCE_Software = 7,
    MV_TRIGGER_SOURCE_FrequencyConverter = 8,
    MV_TRIGGER_SOURCE_Action1_SPECIAL = -1
}MV_TRIGGER_SOURCE;

// nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", MV_Action1);
#define MV_TRIGGER_SOURCE_Action1 "Action1"

#define ActionDeviceKey 1
#define ActionGroupKey 1
#define ActionGroupMask 1


typedef enum _MV_TRIGGER_ACTIVATION_ // TriggerActivation
{
    MV_TRIGGER_ACTIVATION_RisingEdge = 0,
    MV_TRIGGER_ACTIVATION_FallingEdge = 1,
    MV_TRIGGER_ACTIVATION_LevelHigh = 2,
    MV_TRIGGER_ACTIVATION_LevelLow = 3
}MV_TRIGGER_ACTIVATION;


typedef enum _MV_EXPOSURE_AUTO_ //ExposureAuto
{
    MV_EXPOSURE_AUTO_OFF = 0,
    MV_EXPOSURE_AUTO_ONCE = 1,
    MV_EXPOSURE_AUTO_CONTINUOUS = 2
}MV_EXPOSURE_AUTO;


typedef enum _MV_GAIN_AUTO_ // GainAuto
{
    MV_GAIN_AUTO_OFF = 0,
    MV_GAIN_AUTO_ONCE = 1,
    MV_GAIN_AUTO_CONTINUOUS = 2
}MV_GAIN_AUTO;



/*Digital IO Control*/
typedef enum _MV_LINE_SELECTOR_ // LineSelector
{
    MV_LINE_SELECTOR_Line0 = 0, 
    MV_LINE_SELECTOR_Line1 = 1, 
    MV_LINE_SELECTOR_Line2 = 2,
    MV_LINE_SELECTOR_Line3 = 3,
    MV_LINE_SELECTOR_Line4 = 4
}MV_LINE_SELECTOR;


typedef enum _MV_LINE_MODE_ // LineMode
{
    MV_LINE_MODE_Input = 0,
    MV_LINE_MODE_Output = 1,
    MV_LINE_MODE_Trigger = 2,
    MV_LINE_MODE_Strobe = 8
}MV_LINE_MODE;


typedef enum _MV_LINE_SOURCE_ // LineSource
{
    MV_LINE_SOURCE_ExposureStartActive = 0,
    MV_LINE_SOURCE_SoftTriggerActive = 5,
    MV_LINE_SOURCE_HardTriggerActive = 6
}MV_LINE_SOURCE;


typedef enum _MV_STROBE_ENABLE_ //StrobeEnable
{
    MV_STROBE_ENABLE_OFF = 0,
    MV_STROBE_ENABLE_ON = 1
}MV_STROBE_ENABLE;



/*Transport Layer Control*/
typedef enum _MV_GEV_IEEE_1588_ // GevIEEE1588
{
    MV_GEV_IEEE_1588_OFF = 0,
    MV_GEV_IEEE_1588_ON = 1
}MV_GEV_IEEE_1588;

#endif