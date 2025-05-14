#include "GigEVisionTools.h"

using namespace gvtools;

GigEVisionActionCommand::GigEVisionActionCommand() :    _sockfd(-1),
                                                        _nBroadcastAddress("255.255.255.255"), 
                                                        _nDeviceKey(0x00000000),
                                                        _nGroupKey(0x00000000),
                                                        _nGroupMask(0x00000000) {
    memset(_msg, 0, sizeof(_msg));
    if (!_createAndConfigureSocket()) {
        perror("Failed to create and configure socket during initialization!");
    }
}

GigEVisionActionCommand::~GigEVisionActionCommand() {
    if (_sockfd >= 0) {
        close(_sockfd);
        _sockfd = -1;
    }
}

void GigEVisionActionCommand::setConfig(std::string broadcastAddress, int broadcastPort, 
                                        uint16_t deviceKey, uint16_t groupKey, uint16_t groupMask) {
    setIpConfig(broadcastAddress, broadcastPort);
    setActionCmdConfig(deviceKey, groupKey, groupMask);
}

void GigEVisionActionCommand::setIpConfig(std::string broadcastAddress, int broadcastPort) {
    _nBroadcastAddress = broadcastAddress;
    _nBroadcastPort = broadcastPort;
    _configureAddress();
}

void GigEVisionActionCommand::setActionCmdConfig(uint16_t deviceKey, uint16_t groupKey, uint16_t groupMask) {
    _nDeviceKey = deviceKey;
    _nGroupKey = groupKey;
    _nGroupMask = groupMask;
    if (!_msgActionCmdGenerate()) {
        perror("Failed to generate Action Command message!");
    }
}

bool GigEVisionActionCommand::send_msg() {
    if (_sockfd < 0) {  // 检查 Socket 是否有效
        perror("Socket is invalid, attempting to reinitialize...");
        if (!_reinitializeSocket()) {
            perror("Failed to reinitialize socket");
            return false;
        }
    }

    int ret = sendto(_sockfd, _msg, _msg_length, 0,
                     reinterpret_cast<struct sockaddr*>(&_addr), sizeof(_addr));
    if (ret < 0) {
        perror("Broadcast failed");
        return false;
    }
    return true;
}

bool GigEVisionActionCommand::isDeviceConnected() {
    // 简单的实现，可以发送一个心跳消息或 PING 请求
    int ret = sendto(_sockfd, _msg, _msg_length, 0,
                     reinterpret_cast<struct sockaddr*>(&_addr), sizeof(_addr));
    return (ret >= 0);  // 如果发送成功，则认为设备在线
}

bool GigEVisionActionCommand::resetConnection() {
    return _reinitializeSocket();
}

bool GigEVisionActionCommand::_createAndConfigureSocket() {
    _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_sockfd < 0) {
        perror("Socket creation failed");
        return false;
    }

    int broadcastEnable = 1;
    if (setsockopt(_sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) {
        perror("Failed to set broadcast option");
        close(_sockfd);
        return false;
    }

    return true;
}

bool GigEVisionActionCommand::_reinitializeSocket() {
    if (_sockfd >= 0) {
        close(_sockfd);  // 关闭旧的 Socket
    }
    return _createAndConfigureSocket();
}

void GigEVisionActionCommand::_configureAddress() {
    memset(&_addr, 0, sizeof(_addr));
    _addr.sin_family = AF_INET;
    _addr.sin_port = htons(_nBroadcastPort);
    _addr.sin_addr.s_addr = inet_addr(_nBroadcastAddress.c_str());
}

bool GigEVisionActionCommand::_msgActionCmdGenerate() {
    _msg[0] = 0x42;
    _msg[1] = 0x01;
    _msg[2] = 0x01; _msg[3] = 0x00;
    _msg[4] = 0x00; _msg[5] = 0x0c;
    _msg[6] = 0xff; _msg[7] = 0xff;

    _msg[8] = (_nDeviceKey >> 24) & 0xFF; 
    _msg[9] = (_nDeviceKey >> 16) & 0xFF; 
    _msg[10] = (_nDeviceKey >> 8) & 0xFF; 
    _msg[11] = _nDeviceKey & 0xFF;   

    _msg[12] = (_nGroupKey >> 24) & 0xFF; 
    _msg[13] = (_nGroupKey >> 16) & 0xFF; 
    _msg[14] = (_nGroupKey >> 8) & 0xFF;
    _msg[15] = _nGroupKey & 0xFF;   
       
    _msg[16] = (_nGroupMask >> 24) & 0xFF; 
    _msg[17] = (_nGroupMask >> 16) & 0xFF;
    _msg[18] = (_nGroupMask >> 8) & 0xFF;
    _msg[19] = _nGroupMask & 0xFF;   

    _msg_length = sizeof(_msg);
    return (_msg_length == 20);
}