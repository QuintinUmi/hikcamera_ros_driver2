#include "Camera_Manager.h"
#include <map>

using namespace hikcamera_ros_driver2;

CameraManager::CameraManager() {}
CameraManager::~CameraManager() {
    removeAllCameras();
}

int CameraManager::enumerateDevices() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        ROS_ERROR("Enum Devices fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    if (stDeviceList.nDeviceNum > 0) {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo) {
                continue;
            }
            printDeviceInfo(pDeviceInfo);
        }
    } else {
        ROS_ERROR("Find No Devices!\n");
        return -1;
    }

    return nRet;
}

bool CameraManager::addCamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index, ros::NodeHandle* nh) {
    if (_cameras.find(camera_index) != _cameras.end()) {
        return false;
    }
    Hikcamera* newCamera = new Hikcamera(pDeviceInfo, camera_index, nh);
    _cameras[camera_index] = newCamera;
    return true;
}

bool CameraManager::addAllCameras(MV_CC_DEVICE_INFO_LIST* pDeviceList, ros::NodeHandle* nh) {
    if (pDeviceList == nullptr) {
        return false;
    }

    for (unsigned int i = 0; i < pDeviceList->nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* pDeviceInfo = pDeviceList->pDeviceInfo[i];
        if (pDeviceInfo == nullptr) {
            continue; 
        }
        addCamera(pDeviceInfo, i, nh); 
    }
    return true;
}

bool CameraManager::removeCamera(uint8_t camera_index) {
    auto it = _cameras.find(camera_index);
    if (it == _cameras.end()) {
        return false;
    }
    it->second->stopGrabbing();
    it->second->deinitDevice();
    delete it->second;
    _cameras.erase(it);
    return true;
}

bool CameraManager::removeAllCameras() {
    for (auto& cam : _cameras) {
        cam.second->stopGrabbing();
        cam.second->deinitDevice();
        delete cam.second;
    }
    _cameras.clear();
    return true;
}

bool CameraManager::initAllCameras() {
    for (auto& cam : _cameras) {
        if (cam.second->init() != 0) {
            return false;
        }
    }
    return true;
}

void CameraManager::startAllCamerasGrabbing() {
    for (auto& cam : _cameras) {
        cam.second->startGrabbing();
    }
}
void CameraManager::stopAllCamerasGrabbing() {
    for (auto& cam : _cameras) {
        cam.second->stopGrabbing();
    }
}

Hikcamera* CameraManager::getCamera(uint8_t camera_index) {
    auto it = _cameras.find(camera_index);
    if (it != _cameras.end()) {
        return it->second;
    }
    return nullptr;
}



