#include <stdio.h>
#include "EthernetTask.hpp"
#include "GlobalVariableManager.hpp"

#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"

void udp_controller_task(void *arg) {
    _TaskConfigUDP* args = static_cast<_TaskConfigUDP*>(arg);
    const char* bindIP = args->bindIP;
    const char* UDP_DEST_IP = args->destionationIP;
    uint16_t UDP_DEST_PORT = args->UDP_DESTINATION_PORT;

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr(bindIP);
    bind_addr.sin_port = htons(UDP_DEST_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        printf("UDP[%s]: socket() errno=%d\n", bindIP, errno);
        vTaskDelete(nullptr);
    }

    bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    struct sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_DEST_IP);
    dest_addr.sin_port = htons(UDP_DEST_PORT);

    constexpr uint32_t recvBufferSize = 1024;
    uint8_t recvBuffer[recvBufferSize];

    while (1) {
        // Read packet from Controller (Unsure if this is working correctly, not tested yet.)
        ssize_t len = recvfrom(sock, recvBuffer, sizeof(recvBuffer), MSG_DONTWAIT, NULL, NULL);

        if (len > 0) {
            ssize_t lenSent = globalVariableManager.setUdpFromPeripheralBuffer(recvBuffer, len);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(nullptr);
}

void udp_peripheral_task(void *arg) {
    _TaskConfigUDP* args = static_cast<_TaskConfigUDP*>(arg);
    const char* bindIP = args->bindIP;
    const char* UDP_DEST_IP = args->destionationIP;
    uint16_t UDP_DEST_PORT = args->UDP_DESTINATION_PORT;

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr(bindIP);
    bind_addr.sin_port = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        printf("UDP[%s]: socket() errno=%d\n", bindIP, errno);
        vTaskDelete(nullptr);
    }

    bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    struct sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_DEST_IP);
    dest_addr.sin_port = htons(UDP_DEST_PORT);

    printf("UDP sender %s -> %s:%d\n", bindIP, UDP_DEST_IP, UDP_DEST_PORT);

    static uint32_t iteration = 0;

    constexpr uint32_t maxRecvBuffer = 1024;
    constexpr uint32_t packetBufferSize = 16;
    uint8_t packet[packetBufferSize + maxRecvBuffer];

    uint32_t header = 0b111;

    while (1) {
        uint32_t recvBufferSize = globalVariableManager.getUdpFromPeripheralBuffer(packet + 16, maxRecvBuffer);

        // Send packet to Controller
        uint32_t iter = iteration++;
        float pos = globalVariableManager.getAngle();
        auto time = esp_timer_get_time();

        memcpy(packet + 0, &header, 4);
        memcpy(packet + 4, &iter, 4);
        memcpy(packet + 8, &time, 4);
        memcpy(packet + 12, &pos, 4);

        int sent = sendto(sock, packet, packetBufferSize + recvBufferSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            printf("UDP[%s]: sendto() errno=%d\n", bindIP, errno);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(nullptr);
}

void tcp_peripheral_task(void *arg) {
    // Not implemented any actual commands or anything yet. Only a simple echo.

    _TaskConfigTCP* args = static_cast<_TaskConfigTCP*>(arg);
    const char* bindIP = args->bindIP;
    uint16_t TCP_LISTEN_PORT = args->TCP_LISTEN_PORT;

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr(bindIP);
    bind_addr.sin_port = htons(TCP_LISTEN_PORT);

    int listen_sock;
    while (1) {
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            printf("TCP[%s]: socket() errno=%d\n", bindIP, errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(listen_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            printf("TCP[%s]: bind() errno=%d\n", bindIP, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (listen(listen_sock, 1) < 0) {
            printf("TCP[%s]: listen() errno=%d\n", bindIP, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP[%s] listening on port %d\n", bindIP, TCP_LISTEN_PORT);

        struct sockaddr_in client_addr = {};
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            printf("TCP[%s]: accept() errno=%d\n", bindIP, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP[%s] client connected\n", bindIP);
        close(listen_sock);

        uint8_t buf[1024];
        while (1) {
            int len = recv(client_sock, buf, sizeof(buf), 0);
            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bindIP, len, errno);
                break;
            }
            int sent = send(client_sock, buf, len, 0);
            if (sent < 0) {
                printf("TCP[%s]: send() errno=%d\n", bindIP, errno);
                break;
            }
        }

        close(client_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

EthernetTask::EthernetTask(EthernetTaskConfig &config)
    : _config(config),
      _eth0(config.cW5500_0_CS, config.cW5500_0_INT, 0),
      _eth1(config.cW5500_1_CS, config.cW5500_1_INT, 1) {
    gpio_set_level(config.cW5500_0_CS, 1);
    gpio_set_level(config.cW5500_1_CS, 1);
    gpio_set_direction(config.cW5500_0_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.cW5500_1_CS, GPIO_MODE_OUTPUT);

    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(_eth0.init());
    ESP_ERROR_CHECK(_eth1.init());

    _eth0.set_static_ip(config.cW5500_0_IP, config.cW5500_NETMASK, config.cW5500_0_GW);
    _eth1.set_static_ip(config.cW5500_1_IP, config.cW5500_NETMASK, config.cW5500_1_GW);
}

void EthernetTask::begin() {
    _TaskConfigUDP udpConfigPeripheral{
        .bindIP = _config.cW5500_0_IP,
        .destionationIP = _config.cW5500_1_IP,
        .UDP_DESTINATION_PORT = _config.cUDP_DESTINATION_PORT,
    };

    _TaskConfigUDP udpConfigController{
        .bindIP = _config.cW5500_1_IP,
        .destionationIP = _config.cW5500_0_IP,
        .UDP_DESTINATION_PORT = _config.cUDP_DESTINATION_PORT,
    };

    _TaskConfigTCP tcpConfig{
        .bindIP = _config.cW5500_0_IP,
        .TCP_LISTEN_PORT = _config.cTCP_LISTEN_PORT,
    };

    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(udp_controller_task, "udp_eth1", 8192, &udpConfigController, 12, nullptr);
    xTaskCreate(udp_peripheral_task, "udp_eth0", 8192, &udpConfigPeripheral, 12, nullptr);
    xTaskCreate(tcp_peripheral_task, "tcp_eth0", 8192, &tcpConfig, 12, nullptr);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

bool EthernetTask::isLinkUp(int ix) {
    char ifname[8];

    esp_err_t err;
    if (ix == 0) {
        err = esp_netif_get_netif_impl_name(_eth0.netif(), ifname);
    } else if (ix == 1) {
        err = esp_netif_get_netif_impl_name(_eth1.netif(), ifname);
    } else {
        return false;
    }

    if (err == ESP_OK) {
        struct netif *n = netif_find(ifname);

        if (n && netif_is_link_up(n)) {
            return true;
        } else {
            return false;
        }
    }

    return false;
}

void EthernetTask::printIP() {
    esp_netif_ip_info_t ip;
    char ip_s[16], mask_s[16];
    if (esp_netif_get_ip_info(_eth0.netif(), &ip) == ESP_OK) {
        esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
        esp_ip4addr_ntoa(&ip.netmask, mask_s, sizeof(mask_s));
        printf("W5500[0] IP=%s MASK=%s\n", ip_s, mask_s);
    }
    if (esp_netif_get_ip_info(_eth1.netif(), &ip) == ESP_OK) {
        esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
        esp_ip4addr_ntoa(&ip.netmask, mask_s, sizeof(mask_s));
        printf("W5500[1] IP=%s MASK=%s\n", ip_s, mask_s);
    }
}