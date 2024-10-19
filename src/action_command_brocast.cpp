#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include "GigEVisionTools.h"

#define SHOW_AC_MSG_INFO false

using namespace gvtools;

void sigintHandler(int sig) {
    ros::shutdown();  
}

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "multi_cam_ros_pub");
    ros::NodeHandle nh;

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

    using namespace std::chrono;
    auto next_time = system_clock::now();
    int offset_t_ms = -1;
    nh.param("offset_t_ms", offset_t_ms, 0);
    
    next_time += milliseconds(interval - duration_cast<milliseconds>(next_time.time_since_epoch()).count() % interval);

    while (ros::ok() && ros::master::check()) {

        if (offset_t_ms >= 0)   std::this_thread::sleep_until(next_time + milliseconds(offset_t_ms));
        else                    std::this_thread::sleep_until(next_time - milliseconds(-offset_t_ms));

        if (!gv.send_msg()) {
            ROS_ERROR("Send Action Command Failed!");
            continue;
        }

        if (SHOW_AC_MSG_INFO) {
            std::cout << "Broadcast message sent at "
                    << duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()
                    << " ms" << std::endl;
        }

        // next_time += milliseconds(interval);
        next_time += milliseconds(interval - duration_cast<milliseconds>(next_time.time_since_epoch()).count() % interval);
    }

    return 0;
}