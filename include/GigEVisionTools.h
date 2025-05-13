#ifndef _GIGE_VISION_TOOLS_H_
#define _GIGE_VISION_TOOLS_H_

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <thread>

namespace gvtools
{
    class GigEVisionActionCommand {

        public:
            GigEVisionActionCommand();
            ~GigEVisionActionCommand();

            void setConfig(std::string broadcastAddress, int broadcastPort, uint16_t deviceKey, uint16_t groupKey, uint16_t groupMask);
            void setIpConfig(std::string broadcastAddress, int broadcastPort);
            void setActionCmdConfig(uint16_t deviceKey, uint16_t groupKey, uint16_t groupMask);

            bool send_msg();
            bool isDeviceConnected();  // 检查设备是否在线
            bool resetConnection();

        private:
            int _sockfd;
            sockaddr_in _addr;

            std::string _nBroadcastAddress;
            int _nBroadcastPort;
            uint16_t _nDeviceKey;
            uint16_t _nGroupKey;
            uint16_t _nGroupMask;

            unsigned char _msg[20];
            int _msg_length;

        public:
            bool _createAndConfigureSocket();
            bool _reinitializeSocket();  // 重新初始化 Socket
            void _configureAddress();
            bool _msgActionCmdGenerate();
    };
}

#endif