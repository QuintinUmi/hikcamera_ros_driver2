#include "CameraManager.h"
#include <map>

using namespace hikcamera_ros_driver2;

#define SHOW_DEBUG_LOG true

CameraManager::CameraManager(ros::NodeHandle* nh) : _nh(nh) {}
CameraManager::~CameraManager() {
    removeAllCameras();
}

int CameraManager::enumerateDevices() {
    memset(&_stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &_stDeviceList);
    if (MV_OK != nRet) {
        ROS_ERROR("Enum Devices fail! nRet [0x%x]\n", nRet);
        return nRet;
    }

    if (_stDeviceList.nDeviceNum > 0) {
        ROS_INFO("Find Devices:\n");
        for (unsigned int i = 0; i < _stDeviceList.nDeviceNum; i++) {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = _stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo) {
                continue;
            }
            if (SHOW_DEBUG_LOG) printDeviceInfo(pDeviceInfo);
        }
    } else {
        ROS_ERROR("Find No Devices!\n");
        return -1;
    }

    return nRet;
}
int CameraManager::enumerateDevices(MV_CC_DEVICE_INFO_LIST* outStDeviceList) {
    int nRet = MV_OK;
    nRet = enumerateDevices();
    *outStDeviceList = _stDeviceList;
    return nRet;
}

bool CameraManager::addCamera(MV_CC_DEVICE_INFO* pDeviceInfo, uint8_t camera_index) {
    if (pDeviceInfo == nullptr) {
        return false; 
    }
    if (_cameras.find(camera_index) != _cameras.end()) {
        return false; 
    }
    std::shared_ptr<Hikcamera> newCamera = std::make_shared<Hikcamera>(pDeviceInfo, camera_index, _nh);
    _cameras[camera_index] = newCamera;
    if (SHOW_DEBUG_LOG) _cameras[camera_index]->printDeviceInfo();
    return true;
}

bool CameraManager::addAllCameras() {
    bool cRet = true;
    if(enumerateDevices() != MV_OK) cRet = false;
    cRet = addAllCameras(&_stDeviceList) && cRet;
    return cRet;
}
bool CameraManager::addAllCameras(MV_CC_DEVICE_INFO_LIST* pDeviceList) {
    if (pDeviceList == nullptr) {
        return false;
    }

    for (unsigned int i = 0; i < pDeviceList->nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* pDeviceInfo = pDeviceList->pDeviceInfo[i];
        if (pDeviceInfo == nullptr) {
            continue; 
        }
        addCamera(pDeviceInfo, i); 
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
    _cameras.erase(it); 
    return true;
}

bool CameraManager::removeAllCameras() {
    for (auto& cam : _cameras) {
        cam.second->stopGrabbing();
        cam.second->deinitDevice();
    }
    _cameras.clear(); 
    return true;
}

bool CameraManager::initAllCameras(Hikcamera::WORK_THREAD_MODE work_thread_mode) {
    bool cRet = true;
    if (work_thread_mode == Hikcamera::WORK_THREAD_MODE_ROS_PUBLISH) {
        for (auto& cam : _cameras) {
            if (cam.second->init() != 0) {
                cRet = false;
            }
        }
    } else if (work_thread_mode == Hikcamera::WORK_THREAD_MODE_IMAGE_CALLBACK) {
        for (auto& cam : _cameras) {
            if (cam.second->init(false) != 0) {
                cRet = false;
            }
        }
    } else {
        for (auto& cam : _cameras) {
            if (cam.second->init(false) != 0) {
                cRet = false;
            }
        }
    }
    
    return cRet;
}

bool CameraManager::startAllCamerasGrabbing() {
    int cRet = true;
    for (auto& cam : _cameras) {
        if(cam.second->startGrabbing() != MV_OK) cRet = false;
    }
    return cRet;
}
bool CameraManager::stopAllCamerasGrabbing() {
    int cRet = true;
    for (auto& cam : _cameras) {
        if(cam.second->stopGrabbing() != MV_OK) cRet = false;
    }
    return cRet;
}

std::map<uint8_t, std::shared_ptr<Hikcamera>> CameraManager::getCameraList() {
    return _cameras;
}

std::shared_ptr<Hikcamera> CameraManager::getCamera(uint8_t camera_index) {
    auto it = _cameras.find(camera_index);
    if (it != _cameras.end()) {
        return it->second;
    }
    return nullptr;
}



