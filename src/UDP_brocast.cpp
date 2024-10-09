#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <thread>

// 创建并配置socket的函数
int create_and_configure_socket() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return -1;
    }

    int broadcastEnable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) {
        perror("Error in setting broadcast option");
        close(sockfd);
        return -1;
    }

    return sockfd;
}

// 配置目的地址的函数
void configure_address(struct sockaddr_in& addr) {
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(3956);
    addr.sin_addr.s_addr = inet_addr("192.168.1.255");
}

// 发送数据的函数
bool send_data(int sockfd, struct sockaddr_in& addr, const unsigned char* buffer, int buffer_length) {
    int ret = sendto(sockfd, buffer, buffer_length, 0,
                     reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (ret < 0) {
        perror("Broadcast failed");
        return false;
    }
    return true;
}

int main() {
    int sockfd = create_and_configure_socket();
    if (sockfd < 0) {
        return 1;
    }

    struct sockaddr_in addr;
    configure_address(addr);

    unsigned char buffer[] = {
        0x42, 
        0x01, 
        0x01, 0x00, 
        0x00, 0x0c,
        0xff, 0xff, 
        0x00, 0x00, 0x00, 0x01, 
        0x00, 0x00, 0x00, 0x01, 
        0xff, 0xff, 0xff, 0xff
    };
    int buffer_length = sizeof(buffer);

    using namespace std::chrono;
    auto next_time = system_clock::now();
    // 调整到下一个整100毫秒时刻
    next_time += milliseconds(100 - duration_cast<milliseconds>(next_time.time_since_epoch()).count() % 100);

    while (true) {
        // 等待直到下一个整100毫秒
        std::this_thread::sleep_until(next_time);

        if (!send_data(sockfd, addr, buffer, buffer_length)) {
            break;
        }

        std::cout << "Broadcast message sent at "
                  << duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()
                  << " ms" << std::endl;

        // 设置下一个整100毫秒时刻
        next_time += milliseconds(100);
    }

    close(sockfd);
    return 0;
}