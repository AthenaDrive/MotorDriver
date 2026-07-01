#include <stdio.h>
#include <bitset>

#include "EthernetTask.hpp"
#include "GlobalVariableManager.hpp"

#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "sys/select.h"
#include "lwip/netif.h"

void udp_as_controller_task(void *arg) {
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

    constexpr uint32_t recvBufferCapacity = 1024;
    uint8_t recvBuffer[recvBufferCapacity];

    constexpr uint32_t sendBufferCapacity = 1024;
    uint8_t sendBuffer[sendBufferCapacity];
    uint32_t sendBufferSize = 0;

    while (1) {
        // Read from downstream
        ssize_t lenRecv = recvfrom(sock, recvBuffer, sizeof(recvBuffer), MSG_DONTWAIT, NULL, NULL);
        if (lenRecv > 0) {
            ssize_t lenSent = globalVariableManager.setUdpFromPeripheralBuffer(recvBuffer, lenRecv);
        }

        // Write to downstream
        sendBufferSize = globalVariableManager.getUdpFromControllerBuffer(sendBuffer, sendBufferCapacity);
        if (sendBufferSize > 0) {
            ssize_t lenSent = sendto(sock, sendBuffer, sendBufferSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(nullptr);
}

void udp_as_peripheral_task(void *arg) {
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

    printf("UDP sender %s -> %s:%d\n", bindIP, UDP_DEST_IP, UDP_DEST_PORT);

    // Packet to controller will probably never be this big.
    // As of 18/06/26, excel protocol gives a max size of 60 bytes.
    constexpr uint32_t packetBufferSize = 128;

    // Gives a lower bound of max motors downstream.
    // Motors probably uses less than packetBufferSize,
    // This therefore *probably* gives 32+ motors.
    // But only 17 is guaranteed (16 + this one).
    constexpr uint32_t maxRecvBuffer = packetBufferSize * 16;
    uint8_t packet[packetBufferSize + maxRecvBuffer];

    uint32_t header;
    uint32_t offset = 0;
    uint32_t recvBufferSize;
    static uint32_t iteration = 0;

    uint32_t recvHeader;
    uint32_t recvOffset = 0;
    uint8_t recvPacket[packetBufferSize + maxRecvBuffer];

    while (1) {
        offset = 0;

        // Send to upstream
        header = globalVariableManager.getUdpAsPeripheralHeader();

        memcpy(packet + offset, &header, 4);
        offset += 4;

        std::bitset<32> headerBits(header);
        for (int i = 0; i < 32; i++) {
            if (headerBits[i]) {
                switch (i)
                {
                    case 0: {
                        uint32_t time = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                        memcpy(packet + offset, &time, 4);
                    } break;

                    case 1: {
                        float pos = globalVariableManager.getAngle();
                        memcpy(packet + offset, &pos, 4);
                    } break;

                    case 2: {
                        float vel = globalVariableManager.getVelocity();
                        memcpy(packet + offset, &vel, 4);
                    } break;

                    case 3: {
                        float acc = globalVariableManager.getAcceleration();
                        memcpy(packet + offset, &acc, 4);
                    } break;

                    case 4: {
                        float torque = globalVariableManager.getTorque();
                        memcpy(packet + offset, &torque, 4);
                    } break;

                    case 5: {
                        float phaseA = globalVariableManager.getIa();
                        memcpy(packet + offset, &phaseA, 4);
                    } break;

                    case 6: {
                        float phaseB = globalVariableManager.getIb();
                        memcpy(packet + offset, &phaseB, 4);
                    } break;

                    case 7: {
                        float phaseC = globalVariableManager.getIc();
                        memcpy(packet + offset, &phaseC, 4);
                    } break;

                    case 8: {
                        float busCurrent = globalVariableManager.getBusCurrent();
                        memcpy(packet + offset, &busCurrent, 4);
                    } break;

                    case 9: {
                        float busVoltage = globalVariableManager.getBusVoltage();
                        memcpy(packet + offset, &busVoltage, 4);
                    } break;

                    case 10: {
                        uint32_t errorRegister = 0; // TODO!
                        memcpy(packet + offset, &errorRegister, 4);
                    } break;

                    case 11: {
                        uint32_t loopTimeFOC = globalVariableManager.getAvgLoopTimeFOC();
                        memcpy(packet + offset, &loopTimeFOC, 4);
                    } break;

                    case 12: {
                        uint32_t loopTimeSecondary = globalVariableManager.getAvgLoopTimeSecondary();
                        memcpy(packet + offset, &loopTimeSecondary, 4);
                    } break;
                
                    default: {
                        // Invalid header or comm protocol updated.
                    } break;
                }
            
                offset += 4;
            }
        }

        recvBufferSize = globalVariableManager.getUdpFromPeripheralBuffer(packet + offset, maxRecvBuffer);
        int sent = sendto(sock, packet, offset + recvBufferSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            // printf("UDP[%s]: sendto() errno=%d\n", bindIP, errno);
            // Some error (maybe no link)
        }

        // Read from upstream
        recvOffset = 0;
        ssize_t lenRecv = recvfrom(sock, recvPacket, sizeof(recvPacket), MSG_DONTWAIT, NULL, NULL);
        if (lenRecv > 0) {
            
            memcpy(&recvHeader, recvPacket + recvOffset, 4);
            recvOffset += 4;

            std::bitset<32> recvHeaderBits(recvHeader);
            for (int i = 0; i < 32; i++) {
                if (recvHeaderBits[i]) {
                    switch (i)
                    {
                        case 0: {
                            float torqueSetpoint;
                            memcpy(&torqueSetpoint, recvPacket + recvOffset, 4);
                            globalVariableManager.setTorqueSetpoint(torqueSetpoint);
                        } break;

                        case 1: {
                            float velocitySetpoint;
                            memcpy(&velocitySetpoint, recvPacket + recvOffset, 4);
                            globalVariableManager.setVelocitySetpoint(velocitySetpoint);
                        } break;

                        case 2: {
                            float positionSetpoint;
                            memcpy(&positionSetpoint, recvPacket + recvOffset, 4);
                            globalVariableManager.setPositionSetpoint(positionSetpoint);
                        } break;
                    
                        default: {
                            // Invalid header or comm protocol updated.
                        } break;
                        }

                    recvOffset += 4;
                }
            }
        
            if (lenRecv > recvOffset) {
                globalVariableManager.setUdpFromControllerBuffer(recvPacket + recvOffset, lenRecv - recvOffset);
            }
        }


        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(nullptr);
}

void tcp_as_peripheral_task(void *arg) {
    _TaskConfigTCP* args = static_cast<_TaskConfigTCP*>(arg);
    const char* bindIP = args->bindIP;
    uint16_t TCP_LISTEN_PORT = args->TCP_PORT;

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr(bindIP);
    bind_addr.sin_port = htons(TCP_LISTEN_PORT);

    int listen_sock;
    while (1) {
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            printf("TCP[%s]: socket() errno=%d\n", bindIP, errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(listen_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            printf("TCP[%s]: bind() errno=%d\n", bindIP, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (listen(listen_sock, 1) < 0) {
            printf("TCP[%s]: listen() errno=%d\n", bindIP, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        printf("TCP[%s] listening on port %d\n", bindIP, TCP_LISTEN_PORT);

        struct sockaddr_in client_addr = {};
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            printf("TCP[%s]: accept() errno=%d\n", bindIP, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        printf("TCP[%s] client connected\n", bindIP);
        close(listen_sock);

        uint32_t header;
        uint32_t lengthPrefix;

        constexpr uint32_t outBufCapacity = 1024;
        uint8_t outBuf[outBufCapacity];
        uint32_t outBufOffset;
        uint32_t outgoingLengthPrefix;
        uint32_t inBufCommandOffset;
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10));

            ssize_t len = recv(client_sock, &lengthPrefix, sizeof(lengthPrefix), MSG_WAITALL);
            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bindIP, len, errno);
                break;
            }

            if ((lengthPrefix >> 16) != 63609) {
                printf("TCP out of sync?\n");
                // Out of sync..? Problem for later!
                continue; // Maybe break? Seems exessive to break connection.
            }

            uint8_t message[lengthPrefix & 0xFFFF];
            len = recv(client_sock, message, sizeof(message), MSG_WAITALL);

            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bindIP, len, errno);
                break;
            }

            if (len < 4) {
                // Ouf of sync..? Problem for later!
                printf("TCP[%s]: Invalid header (len: %d < 4)\n", bindIP, len);
                continue;
            }

            memcpy(&header, message, 4);
            std::bitset<32> headerBits(header);

            outBufOffset = 4;
            outgoingLengthPrefix = (63609) << 16;

            memcpy(outBuf + outBufOffset, &header, 4);
            outBufOffset += 4;

            inBufCommandOffset = 4;
            if (headerBits[0]) {
                // Command

                for (int i = 1; i < 32; i++) {
                    if (!headerBits[i]) { continue; }

                    printf("Setting bit %i \n", i);

                    switch (i)
                    {
                        case 1: {
                            float torqueSetpoint;
                            memcpy(&torqueSetpoint, message + inBufCommandOffset, 4);
                            globalVariableManager.setTorqueSetpoint(torqueSetpoint);
                        } break;

                        case 2: {
                            float torqueKp;
                            memcpy(&torqueKp, message + inBufCommandOffset, 4);
                            globalVariableManager.setTorqueKp(torqueKp);
                        } break;

                        case 3: {
                            float torqueKd;
                            memcpy(&torqueKd, message + inBufCommandOffset, 4);
                            globalVariableManager.setTorqueKd(torqueKd);
                        } break;

                        case 4: {
                            // torqueKi, not used.
                        } break;

                        case 5: {
                            float torqueLimit;
                            memcpy(&torqueLimit, message + inBufCommandOffset, 4);
                            globalVariableManager.setTorqueLimit(torqueLimit);
                        } break;

                        case 6: {
                            float velocitySetpoint;
                            memcpy(&velocitySetpoint, message + inBufCommandOffset, 4);
                            globalVariableManager.setVelocitySetpoint(velocitySetpoint);
                        } break;

                        case 7: {
                            float velocityKp;
                            memcpy(&velocityKp, message + inBufCommandOffset, 4);
                            globalVariableManager.setVelocityKp(velocityKp);
                        } break;

                        case 8: {
                            float velocityKd;
                            memcpy(&velocityKd, message + inBufCommandOffset, 4);
                            globalVariableManager.setVelocityKd(velocityKd);
                        } break;

                        case 9: {
                            float velocityKi;
                            memcpy(&velocityKi, message + inBufCommandOffset, 4);
                            globalVariableManager.setVelocityKi(velocityKi);
                        } break;

                        case 10: {
                            float velocityLimit;
                            memcpy(&velocityLimit, message + inBufCommandOffset, 4);
                            globalVariableManager.setVelocityLimit(velocityLimit);
                        } break;

                        case 11: {
                            float positionSetpoint;
                            memcpy(&positionSetpoint, message + inBufCommandOffset, 4);
                            globalVariableManager.setPositionSetpoint(positionSetpoint);
                        } break;

                        case 12: {
                            float positionKp;
                            memcpy(&positionKp, message + inBufCommandOffset, 4);
                            globalVariableManager.setPositionKp(positionKp);
                        } break;

                        case 13: {
                            float positionKd;
                            memcpy(&positionKd, message + inBufCommandOffset, 4);
                            globalVariableManager.setPositionKd(positionKd);
                        } break;

                        case 14: {
                            float positionKi;
                            memcpy(&positionKi, message + inBufCommandOffset, 4);
                            globalVariableManager.setPositionKi(positionKi);
                        } break;

                        case 15: {
                            uint32_t drivingMode;
                            memcpy(&drivingMode, message + inBufCommandOffset, 4);
                            globalVariableManager.setDrivingMode(drivingMode);
                        } break;

                        case 16: {
                            uint32_t currentLimitBus;
                            memcpy(&currentLimitBus, message + inBufCommandOffset, 4);
                            globalVariableManager.setCurrentLimitBus(currentLimitBus);
                        } break;

                        case 17: {
                            uint32_t currentLimitPhase;
                            memcpy(&currentLimitPhase, message + inBufCommandOffset, 4);
                            globalVariableManager.setCurrentLimitPhase(currentLimitPhase);
                        } break;

                        case 18: {
                            uint32_t polePairs;
                            memcpy(&polePairs, message + inBufCommandOffset, 4);
                            globalVariableManager.setNumPolePairs(polePairs);
                        } break;

                        case 19: {
                            uint32_t udpDataHeader;
                            memcpy(&udpDataHeader, message + inBufCommandOffset, 4);
                            globalVariableManager.setUdpAsPeripheralHeader(udpDataHeader);
                        } break;

                        case 20: {
                            uint32_t errorFlags = 0; // TODO!
                            memcpy(&errorFlags, message + inBufCommandOffset, 4);
                            globalVariableManager.setErrorFlags(errorFlags);
                        } break;
                    
                    default:
                        break;
                    }

                    inBufCommandOffset += 4;
                }

            } else {
                // Reading data

                for (int i = 1; i < 32; i++) {
                    if (!headerBits[i]) { continue; }

                    printf("Reading bit %i \n", i);

                    switch (i)
                    {
                        case 1: {
                            float busVoltage = globalVariableManager.getBusVoltage();
                            memcpy(outBuf + outBufOffset, &busVoltage, 4);
                        } break;

                        case 2: {
                            float busCurrent = globalVariableManager.getBusCurrent();
                            memcpy(outBuf + outBufOffset, &busCurrent, 4);
                        } break;

                        case 3: {
                            uint32_t ledStatus = globalVariableManager.getLedStatus();
                            memcpy(outBuf + outBufOffset, &ledStatus, 4);
                        } break;

                        case 4: {
                            uint32_t buttonStatus = globalVariableManager.getButtonStatus();
                            memcpy(outBuf + outBufOffset, &buttonStatus, 4);
                        } break;

                        case 5: {
                            float currentLimitBus = globalVariableManager.getCurrentLimitBus();
                            memcpy(outBuf + outBufOffset, &currentLimitBus, 4);
                        } break;

                        case 6: {
                            float currentLimitPhase = globalVariableManager.getCurrentLimitPhase();
                            memcpy(outBuf + outBufOffset, &currentLimitPhase, 4);
                        } break;

                        case 7: {
                            uint32_t boardState = globalVariableManager.getBoardState();
                            memcpy(outBuf + outBufOffset, &boardState, 4);
                        } break;

                        case 8: {
                            uint32_t drivingMode = globalVariableManager.getDrivingMode();
                            memcpy(outBuf + outBufOffset, &drivingMode, 4);
                        } break;

                        case 9: {
                            uint32_t numPolePairs = globalVariableManager.getNumPolePairs();
                            memcpy(outBuf + outBufOffset, &numPolePairs, 4);
                        } break;

                        case 10: {
                            float phaseRMSVoltage = globalVariableManager.getPhaseRMSVoltage();
                            memcpy(outBuf + outBufOffset, &phaseRMSVoltage, 4);
                        } break;

                        case 11: {
                            uint32_t errorFlags = globalVariableManager.getErrorFlags();
                            memcpy(outBuf + outBufOffset, &errorFlags, 4);
                        } break;
                    
                    default:
                        break;
                    }

                    outBufOffset += 4;
                }

            }

            printf("Sending %li bytes.\n", len - inBufCommandOffset);
            globalVariableManager.setTcpFromControllerBuffer(message + inBufCommandOffset, len - inBufCommandOffset);

            auto newLenght = globalVariableManager.getTcpFromPeripheralBuffer(outBuf + outBufOffset, outBufCapacity - outBufOffset);

            outgoingLengthPrefix += outBufOffset + newLenght - 4;
            memcpy(outBuf, &outgoingLengthPrefix, 4);

            int sent = send(client_sock, outBuf, outBufOffset + newLenght, 0);
            // printf("Sent %d bytes of data.\n", sent);
            if (sent < 0) {
                printf("TCP[%s]: send() errno=%d\n", bindIP, errno);
                break;
            }
        }

        close(client_sock);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tcp_as_controller_task(void *arg) {
    _TaskConfigTCP* args = static_cast<_TaskConfigTCP*>(arg);

    const char* serverIP = args->bindIP;
    uint16_t TCP_SERVER_PORT = args->TCP_PORT;

    struct sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(serverIP);
    server_addr.sin_port = htons(TCP_SERVER_PORT);

    constexpr uint32_t bufCapacity = 1024;
    uint8_t sendBuffer[bufCapacity];
    uint8_t recvBuffer[bufCapacity];
    uint32_t recvPrefix;

    while (1) {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            printf("TCP[%s]: socket() errno=%d\n", serverIP, errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        printf("TCP[%s]: connecting...\n", serverIP);

        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            printf("TCP[%s]: connect() errno=%d\n", serverIP, errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP[%s] connected!\n", serverIP);

        while (1) {
            uint32_t sendSize = globalVariableManager.getTcpFromControllerBuffer(sendBuffer, bufCapacity);
            if (sendSize > 0) {
                ssize_t lenSent = send(sock, sendBuffer, sendSize, 0);
                if (lenSent < 0) {
                    printf("TCP[%s]: send() errno=%d\n", serverIP, errno);
                    break;
                }
            }

            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sock, &readfds);
            struct timeval tv = {0, 0};

            int sel = select(sock + 1, &readfds, NULL, NULL, &tv);
            if (sel < 0) {
                printf("TCP[%s]: select() errno=%d\n", serverIP, errno);
                break;
            }

            if (sel > 0) {
                ssize_t lenPrefix = recv(sock, &recvPrefix, sizeof(recvPrefix), MSG_WAITALL);
                if (lenPrefix <= 0) {
                    break;
                }

                if ((recvPrefix >> 16) != 63609) {
                    continue;
                }

                ssize_t len = recv(sock, recvBuffer, recvPrefix & 0xFFFF, MSG_WAITALL);
                if (len <= 0) {
                    printf("TCP[%s]: disconnected errno=%d\n", serverIP, errno);
                    break;
                }

                if (len == (recvPrefix & 0xFFFF)) {
                    globalVariableManager.setTcpFromPeripheralBuffer(recvBuffer, len);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        close(sock);
        printf("TCP[%s]: reconnecting...\n", serverIP);
        vTaskDelay(pdMS_TO_TICKS(100));
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
    _TaskConfigUDP udpConfigPeripheral {
        .bindIP = _config.cW5500_0_IP,
        .destionationIP = _config.cW5500_1_IP,
        .UDP_DESTINATION_PORT = _config.cUDP_DESTINATION_PORT,
    };

    _TaskConfigUDP udpConfigController {
        .bindIP = _config.cW5500_1_IP,
        .destionationIP = _config.cW5500_0_IP,
        .UDP_DESTINATION_PORT = _config.cUDP_DESTINATION_PORT,
    };

    _TaskConfigTCP tcpConfigPeripheral {
        .bindIP = _config.cW5500_0_IP,
        .TCP_PORT = _config.cTCP_LISTEN_PORT,
    };

    _TaskConfigTCP tcpConfigController {
        .bindIP = _config.cW5500_0_IP,
        .TCP_PORT = _config.cTCP_LISTEN_PORT,
    };

    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(udp_as_controller_task, "udp_eth1", 8192, &udpConfigController, 12, nullptr);
    xTaskCreate(udp_as_peripheral_task, "udp_eth0", 8192, &udpConfigPeripheral, 12, nullptr);
    xTaskCreate(tcp_as_controller_task, "tcp_eth1", 8192, &tcpConfigController, 12, nullptr);
    xTaskCreate(tcp_as_peripheral_task, "tcp_eth0", 8192, &tcpConfigPeripheral, 12, nullptr);
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

bool EthernetTask::isLinkUp(esp_netif_t* netifInstance) {
    char ifname[8];
    esp_err_t err = esp_netif_get_netif_impl_name(netifInstance, ifname);

    if (err == ESP_OK) {
        struct netif* n = netif_find(ifname);
        if (n && netif_is_link_up(n)) {
            return true;
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