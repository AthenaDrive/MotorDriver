#include <stdio.h>
#include <bitset>

#include "EthernetTask.hpp"
#include "Discovery.hpp"
#include "GlobalVariableManager.hpp"

#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "sys/select.h"
#include "lwip/netif.h"
#include <net/if.h>

static void bind_socket_to_netif(int sock, const char *ifKey) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ifKey);
    if (!netif) {
        printf("bind_to_netif: ifKey '%s' not found\n", ifKey);
        return;
    }
    char ifname[16];
    if (esp_netif_get_netif_impl_name(netif, ifname) != ESP_OK) {
        printf("bind_to_netif: get_impl_name failed for '%s'\n", ifKey);
        return;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strlcpy(ifr.ifr_name, ifname, sizeof(ifr.ifr_name));
    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifr, sizeof(ifr)) < 0) {
        printf("SO_BINDTODEVICE(%s): errno=%d\n", ifname, errno);
    }
}

// ---- UDP Controller (ETH_1 - upstream port) ----
void udp_as_controller_task(void *arg) {
    _TaskConfigUDP* args = static_cast<_TaskConfigUDP*>(arg);
    const char* bindIP = args->bindIP;
    const char* UDP_DEST_IP = args->destionationIP;
    const char* ifKey = args->ifKey;
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

    bind_socket_to_netif(sock, ifKey);
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
        ssize_t lenRecv = recvfrom(sock, recvBuffer, sizeof(recvBuffer), MSG_DONTWAIT, NULL, NULL);
        if (lenRecv > 0) {
            globalVariableManager.setUdpFromPeripheralBuffer(recvBuffer, lenRecv);
        }

        sendBufferSize = globalVariableManager.getUdpFromControllerBuffer(sendBuffer, sendBufferCapacity, true);
        if (sendBufferSize > 0) {
            sendto(sock, sendBuffer, sendBufferSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(nullptr);
}

    // ---- UDP Peripheral (ETH_0 - upstream port) ----
void udp_as_peripheral_task(void *arg) {
    _TaskConfigUDP* args = static_cast<_TaskConfigUDP*>(arg);
    const char* bindIP = args->bindIP;
    const char* UDP_DEST_IP = args->destionationIP;
    const char* ifKey = args->ifKey;
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

    bind_socket_to_netif(sock, ifKey);
    bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    struct sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_DEST_IP);
    dest_addr.sin_port = htons(UDP_DEST_PORT);

    printf("UDP sender %s -> %s:%d\n", bindIP, UDP_DEST_IP, UDP_DEST_PORT);

    constexpr uint32_t packetBufferSize = 128;
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
                        uint32_t errorRegister = 0;
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
                    } break;
                }

                offset += 4;
            }
        }

        recvBufferSize = globalVariableManager.getUdpFromPeripheralBuffer(packet + offset, maxRecvBuffer, true);
        int sent = sendto(sock, packet, offset + recvBufferSize, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            // Fucky wucky!
        }

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

                        case 3: {
                            uint32_t drivingMode;
                            memcpy(&drivingMode, recvPacket + recvOffset, 4);
                            globalVariableManager.setDrivingMode(drivingMode);
                        }

                        default: {
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

// ---- TCP Peripheral (ETH_0 - downstream listener) ----
void tcp_as_peripheral_task(void *arg) {
    _TaskConfigTCP* args = static_cast<_TaskConfigTCP*>(arg);
    const char* bindIP = args->bindIP;
    const char* ifKey = args->ifKey;
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

        bind_socket_to_netif(listen_sock, ifKey);

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
                continue;
            }

            uint8_t message[lengthPrefix & 0xFFFF];
            len = recv(client_sock, message, sizeof(message), MSG_WAITALL);

            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bindIP, len, errno);
                break;
            }

            if (len < 4) {
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
                            float currentLimitBus;
                            memcpy(&currentLimitBus, message + inBufCommandOffset, 4);
                            globalVariableManager.setCurrentLimitBus(currentLimitBus);
                        } break;

                        case 17: {
                            float currentLimitPhase;
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
                            uint32_t errorFlags = 0;
                            // Currently this clears the register, might be better to clear certain bits, or even set register?
                            // memcpy(&errorFlags, message + inBufCommandOffset, 4);
                            globalVariableManager.setErrorFlags(errorFlags);
                        } break;

                    default:
                        break;
                    }

                    inBufCommandOffset += 4;
                }

            } else {
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

            auto newLenght = globalVariableManager.getTcpFromPeripheralBuffer(outBuf + outBufOffset, outBufCapacity - outBufOffset, true);

            outgoingLengthPrefix += outBufOffset + newLenght - 4;
            memcpy(outBuf, &outgoingLengthPrefix, 4);

            int sent = send(client_sock, outBuf, outBufOffset + newLenght, 0);
            printf("Sent %d bytes of data. Newlength: %lu\n", sent, newLenght);
            for (int ixOutBuf = 0; ixOutBuf < outBufOffset + newLenght; ixOutBuf++) {
                printf("%u, ", outBuf[ixOutBuf]);
            }
            printf("\n");

            if (sent < 0) {
                printf("TCP[%s]: send() errno=%d\n", bindIP, errno);
                break;
            }
        }

        close(client_sock);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---- TCP Controller (ETH_1 - upstream connector) ----
void tcp_as_controller_task(void *arg) {
    _TaskConfigTCP* args = static_cast<_TaskConfigTCP*>(arg);

    const char* serverIP = args->bindIP;
    const char* ifKey = args->ifKey;
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

        bind_socket_to_netif(sock, ifKey);

        printf("TCP[%s]: connecting...\n", serverIP);

        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            printf("TCP[%s]: connect() errno=%d\n", serverIP, errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP[%s] connected!\n", serverIP);

        while (1) {
            uint32_t sendSize = globalVariableManager.getTcpFromControllerBuffer(sendBuffer, bufCapacity, true);
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

// ============================================================
// EthernetTask implementation
// ============================================================

EthernetTask::EthernetTask(EthernetTaskConfig &config)
    : _config(config),
      _boardPosition(-1),
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

    // Set temporary IPs to prevent DHCP from running during discovery.
    // These will be overwritten in begin() after discovery completes.
    _eth0.set_static_ip("192.168.0.200", _config.cW5500_NETMASK, _config.cW5500_GW);
    _eth1.set_static_ip("192.168.0.201", _config.cW5500_NETMASK, _config.cW5500_GW);
}

void EthernetTask::begin() {
    // -------------------------------------------------------
    // Phase 1: Discover board position (or use static config)
    // -------------------------------------------------------
    if (_config.cUseAutoIP) {
        printf("ETH: Starting auto-IP discovery...\n");
        _boardPosition = discovery_run(_config.cDiscRetries, _config.cDiscTimeoutMs);
        printf("ETH: Board position = %d\n", _boardPosition);
    } else {
        _boardPosition = 0;
    }

    // -------------------------------------------------------
    // Phase 2: Configure IPs from position
    // -------------------------------------------------------
    // IP scheme: PC=192.168.0.1
    //   Board N: ETH_1 = 192.168.0.(N*2+2)
    //            ETH_0 = 192.168.0.(N*2+3)
    // -------------------------------------------------------
    {
        // ETH_0 = upstream (faces PC/previous board), gets lower IP
        // ETH_1 = downstream (faces next board), gets higher IP
        snprintf(_eth0_ip, sizeof(_eth0_ip), "192.168.0.%u", (unsigned)(_boardPosition * 2 + 2));
        snprintf(_eth1_ip, sizeof(_eth1_ip), "192.168.0.%u", (unsigned)(_boardPosition * 2 + 3));
        _eth0.set_static_ip(_eth0_ip, _config.cW5500_NETMASK, _config.cW5500_GW);
        _eth1.set_static_ip(_eth1_ip, _config.cW5500_NETMASK, _config.cW5500_GW);
    }

    printf("ETH: Configured IPs — ETH_0=%s  ETH_1=%s\n", _eth0_ip, _eth1_ip);

    // -------------------------------------------------------
    // Phase 3: Start discovery responder on ETH_1 (downstream, faces next board)
    // -------------------------------------------------------
    if (_config.cUseAutoIP) {
        discovery_start_responder(_boardPosition);
    }

    // -------------------------------------------------------
    // Phase 4: Compute daisy-chain destination IPs
    // -------------------------------------------------------
    // UDP peripheral (telemetry) on ETH_0 sends upstream → PC (or previous board)
    // UDP controller (relay) on ETH_1 sends downstream → next board
    // -------------------------------------------------------
    if (_boardPosition == 0) {
        snprintf(_controllerDestIP, sizeof(_controllerDestIP), "192.168.0.1");
    } else {
        snprintf(_controllerDestIP, sizeof(_controllerDestIP), "192.168.0.%u",
                 (unsigned)((_boardPosition - 1) * 2 + 3));
    }

    snprintf(_peripheralDestIP, sizeof(_peripheralDestIP), "192.168.0.%u",
             (unsigned)((_boardPosition + 1) * 2 + 2));

    printf("ETH: Controller dest=%s  Peripheral dest=%s\n", _controllerDestIP, _peripheralDestIP);

    // -------------------------------------------------------
    // Phase 5: Build task configs and spawn tasks
    // -------------------------------------------------------
    // ETH_0 = upstream (faces PC) → UDP peripheral (telemetry), TCP server (PC connects to us)
    // ETH_1 = downstream (faces next board) → UDP controller (relay), TCP client (we connect to next board)
    _TaskConfigUDP udpConfigController {
        .bindIP = _eth0_ip,
        .destionationIP = _controllerDestIP,
        .UDP_DESTINATION_PORT = _config.cUDP_DESTINATION_PORT,
        .ifKey = "ETH_0",
    };

    _TaskConfigUDP udpConfigPeripheral {
        .bindIP = _eth1_ip,
        .destionationIP = _peripheralDestIP,
        .UDP_DESTINATION_PORT = _config.cUDP_DESTINATION_PORT,
        .ifKey = "ETH_1",
    };

    // TCP server on ETH_0 — PC connects TO us
    _TaskConfigTCP tcpConfigServer {
        .bindIP = _eth0_ip,
        .TCP_PORT = _config.cTCP_LISTEN_PORT,
        .ifKey = "ETH_0",
    };

    // TCP client on ETH_1 — we connect TO the next board's server (ETH_0)
    _TaskConfigTCP tcpConfigClient {
        .bindIP = _peripheralDestIP,      // next board's ETH_0 IP (server)
        .TCP_PORT = _config.cTCP_LISTEN_PORT,
        .ifKey = "ETH_1",
    };

    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(udp_as_peripheral_task, "udp_eth0", 8192, &udpConfigController, 12, nullptr);
    xTaskCreate(udp_as_controller_task, "udp_eth1", 8192, &udpConfigPeripheral, 12, nullptr);
    xTaskCreate(tcp_as_peripheral_task, "tcp_srv", 8192, &tcpConfigServer, 12, nullptr);
    xTaskCreate(tcp_as_controller_task, "tcp_cli", 8192, &tcpConfigClient, 12, nullptr);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

bool EthernetTask::isLinkUp(int ix) {
    char ifname[16];
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
        }
    }
    return false;
}

bool EthernetTask::isLinkUp(esp_netif_t* netifInstance) {
    char ifname[16];
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
