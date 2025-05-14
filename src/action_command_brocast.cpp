#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

#include "GigEVisionTools.h"

#define SHOW_AC_MSG_INFO false

using namespace gvtools;
using namespace std::chrono;

void sigintHandler(int sig) {
    ros::shutdown();
}

void monitorDevice(GigEVisionActionCommand& gv) {
    while (ros::ok()) {
        if (!gv.isDeviceConnected()) {
            ROS_WARN("Device connection lost. Attempting to reconnect...");
            if (!gv.resetConnection()) {
                ROS_ERROR("Failed to reconnect to device.");
            } else {
                ROS_INFO("Reconnected to device.");
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "multi_cam_ros_pub");
    ros::NodeHandle nh;

    signal(SIGINT, sigintHandler);

    GigEVisionActionCommand gv;
    std::string udp_broadcast_addr;
    int udp_broadcast_port;
    int ActionDeviceKey, ActionGroupMask, ActionGroupKey;
    nh.param("udp_broadcast_addr", udp_broadcast_addr, std::string("192.168.1.255"));
    nh.param("udp_broadcast_port", udp_broadcast_port, 3956);
    nh.param("ActionDeviceKey", ActionDeviceKey, 0x00000001);
    nh.param("ActionGroupMask", ActionGroupMask, 0x00000001);
    nh.param("ActionGroupKey", ActionGroupKey, 0x00000001);
    gv.setConfig(udp_broadcast_addr, udp_broadcast_port, ActionDeviceKey, ActionGroupMask, ActionGroupKey);

    int AcquisitionLineRate, interval;
    nh.param("AcquisitionLineRate", AcquisitionLineRate, 20);
    interval = 1000 / AcquisitionLineRate;

    auto next_time = steady_clock::now();
    next_time += milliseconds(interval);

    int offset_t_ms = -1;
    nh.param("offset_t_ms", offset_t_ms, 0);

    std::thread monitorThread(monitorDevice, std::ref(gv));
    monitorThread.detach();

    while (ros::ok()) {
        if (!ros::master::check()) {
            ROS_WARN("ROS Master is not available. Waiting for reconnection...");
            while (!ros::master::check() && ros::ok()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (!ros::ok()) break;
            ROS_INFO("Reconnected to ROS Master.");
        }

        if (offset_t_ms >= 0) {
            std::this_thread::sleep_until(next_time + milliseconds(offset_t_ms));
        } else {
            std::this_thread::sleep_until(next_time - milliseconds(-offset_t_ms));
        }

        bool sendSuccess = false;
        for (int retry = 0; retry < 3; ++retry) {
            if (gv.send_msg()) {
                sendSuccess = true;
                break;
            }
            ROS_WARN("Retrying to send Action Command...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!sendSuccess) {
            ROS_ERROR("Send Action Command failed after retries!");
            continue;
        }

        if (SHOW_AC_MSG_INFO) {
            std::cout << "Broadcast message sent at "
                      << duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()
                      << " ms" << std::endl;
        }

        next_time += milliseconds(interval - duration_cast<milliseconds>(next_time.time_since_epoch()).count() % interval);
    }

    ROS_INFO("Shutting down...");
    return 0;
}