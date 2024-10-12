#ifndef _CAMERA_MANAGER_H_
#define _CAMERA_MANAGER_H_

#include "Hikcamera.h"
#include <map>

namespace hikcamera_ros_driver2 {

    class CameraManager {
        public:
            CameraManager();
            ~CameraManager();

            int enumerateDevices();

            bool addCamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index, ros::NodeHandle* nh);
            bool addAllCameras(MV_CC_DEVICE_INFO_LIST* pDeviceList, ros::NodeHandle* nh);
            bool removeCamera(uint8_t camera_index);
            bool removeAllCameras();

            bool initAllCameras();

            void startAllCamerasGrabbing();
            void stopAllCamerasGrabbing();

            Hikcamera* getCamera(uint8_t camera_index);

        private:
            std::map<uint8_t, Hikcamera*> _cameras; // 使用map管理相机，key为相机索引
    };

    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
}

#endif 