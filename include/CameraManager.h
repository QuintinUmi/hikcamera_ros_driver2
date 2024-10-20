#ifndef _CameraManager_H_
#define _CameraManager_H_

#include "Hikcamera.h"
#include <map>
#include <algorithm>

namespace hikcamera_ros_driver2 {

    class CameraManager {
        public:
            CameraManager(ros::NodeHandle* nh);
            ~CameraManager();

            int enumerateDevices();
            int enumerateDevices(MV_CC_DEVICE_INFO_LIST* outStDeviceList);
            static bool compareDeviceInfo(const MV_CC_DEVICE_INFO* lhs, const MV_CC_DEVICE_INFO* rhs);

            bool addCamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index);
            bool addAllCameras();
            bool addAllCameras(MV_CC_DEVICE_INFO_LIST* pDeviceList);
            bool removeCamera(uint8_t camera_index);
            bool removeAllCameras();

            bool initAllCameras(Hikcamera::WORK_THREAD_MODE work_thread_mode = Hikcamera::WORK_THREAD_MODE_ROS_PUBLISH);

            bool startAllCamerasGrabbing();
            bool stopAllCamerasGrabbing();

            std::map<uint8_t, std::shared_ptr<Hikcamera>> getCameraList();
            std::shared_ptr<Hikcamera> getCamera(uint8_t camera_index);

        private:
            ros::NodeHandle* _nh;

            MV_CC_DEVICE_INFO_LIST _stDeviceList;
            std::map<uint8_t, std::shared_ptr<Hikcamera>> _cameras; // 使用map管理相机，key为相机索引
    };

    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
}

#endif 