#include <stdio.h>

#include "EthernetTask.hpp"
#include "Discovery.hpp"
#include "GlobalVariableManager.hpp"
#include "Protocol.hpp"

#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "sys/select.h"
#include "lwip/netif.h"
#include <net/if.h>

// Remember the upstream peer seen in a packet (PC on board 0, previous board's
// ETH_1 further down the chain). 0 and broadcast/multicast are not usable as a
// telemetry destination and are ignored.
static void learn_controller_ip(std::atomic_uint32_t *slot, const struct sockaddr_in &peer) {
    if (!slot) {
        return;
    }
    const uint32_t addr = peer.sin_addr.s_addr;   // network byte order
    if (addr == 0 || addr == 0xFFFFFFFFu) {
        return;
    }
    if ((ntohl(addr) >> 28) == 0xEu) {            // 224.0.0.0/4
        return;
    }
    const uint32_t previous = slot->exchange(addr, std::memory_order_relaxed);
    if (previous != addr) {
        const uint32_t host = ntohl(addr);
        printf("ETH: controller IP learned: %u.%u.%u.%u\n",
               (unsigned)(host >> 24) & 0xFFu, (unsigned)(host >> 16) & 0xFFu,
               (unsigned)(host >> 8) & 0xFFu, (unsigned)host & 0xFFu);
    }
}

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

        vTaskDelay(pdMS_TO_TICKS(2));
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
    const uint32_t fallbackDest = dest_addr.sin_addr.s_addr;

    printf("UDP sender %s -> %s:%d\n", bindIP, UDP_DEST_IP, UDP_DEST_PORT);

    constexpr uint32_t packetBufferSize = 128;
    constexpr uint32_t maxRecvBuffer = packetBufferSize * 16;
    uint8_t packet[packetBufferSize + maxRecvBuffer];

    uint32_t recvBufferSize;
    uint8_t recvPacket[packetBufferSize + maxRecvBuffer];

    while (1) {
        // Telemetry: our header and values, then the opaque frames relayed up
        // from the boards downstream of us.
        uint32_t header = globalVariableManager.getUdpAsPeripheralHeader();
        uint32_t ownSize = protocol::encode_udp_data(header, packet, sizeof(packet));
        if (ownSize == 0) {
            printf("UDP[%s]: telemetry does not fit in %u bytes\n", bindIP, (unsigned)sizeof(packet));
        } else {
            // Follow the peer we last heard from; the static scheme is only the
            // bootstrap value (the PC's address depends on DHCP).
            const uint32_t upstream = args->controllerIp->load(std::memory_order_relaxed);
            dest_addr.sin_addr.s_addr = upstream ? upstream : fallbackDest;

            recvBufferSize = globalVariableManager.getUdpFromPeripheralBuffer(
                packet + ownSize, sizeof(packet) - ownSize, true);
            int sent = sendto(sock, packet, ownSize + recvBufferSize, 0,
                              (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (sent < 0) {
                // Fucky wucky!
            }
        }

        // Commands: our header and values, then the frames for downstream boards.
        struct sockaddr_in from = {};
        socklen_t fromLen = sizeof(from);
        ssize_t lenRecv = recvfrom(sock, recvPacket, sizeof(recvPacket), MSG_DONTWAIT,
                                   (struct sockaddr *)&from, &fromLen);
        if (lenRecv > 0) {
            learn_controller_ip(args->controllerIp, from);
            int32_t used = protocol::decode_udp_command(recvPacket, (uint32_t)lenRecv);
            if (used < 0) {
                printf("UDP[%s]: truncated command frame (%d bytes)\n", bindIP, (int)lenRecv);
            } else if ((uint32_t)lenRecv > (uint32_t)used) {
                globalVariableManager.setUdpFromControllerBuffer(
                    recvPacket + used, (uint32_t)lenRecv - (uint32_t)used);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
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
        // The TCP peer is the same upstream device that receives telemetry
        // (PC on board 0, previous board otherwise). Learn it even if the tool
        // never sends a UDP command.
        learn_controller_ip(args->controllerIp, client_addr);
        close(listen_sock);

        uint32_t header;
        uint32_t lengthPrefix;

        // TCP frames carry a chain's worth of relayed sections, so they are
        // bounded by the GVM relay buffers (1024 B) plus one board's values.
        // A fixed buffer keeps the size off the caller's 8 KB task stack; a VLA
        // sized by an untrusted 16-bit prefix did not.
        constexpr uint32_t maxFramePayload = 2048;
        uint8_t message[maxFramePayload];

        constexpr uint32_t outBufCapacity = 1024;
        uint8_t outBuf[outBufCapacity];

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10));

            ssize_t len = recv(client_sock, &lengthPrefix, sizeof(lengthPrefix), MSG_WAITALL);
            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bindIP, len, errno);
                break;
            }

            if ((lengthPrefix >> 16) != protocol::kTcpIdentifier) {
                // Without a valid prefix there is no safe way to find the next
                // frame boundary; reconnect instead of guessing.
                printf("TCP[%s]: bad frame identifier, dropping connection\n", bindIP);
                break;
            }

            const uint32_t payloadLength = lengthPrefix & 0xFFFF;

            if (payloadLength > maxFramePayload) {
                printf("TCP[%s]: frame too large (%u bytes), dropping connection\n",
                       bindIP, (unsigned)payloadLength);
                break;
            }

            if (payloadLength < 4) {
                // Too short to hold a header. Consume it so the stream stays in
                // sync, then ignore the frame.
                uint8_t scratch[4];
                if (payloadLength > 0 &&
                    recv(client_sock, scratch, payloadLength, MSG_WAITALL) <= 0) {
                    printf("TCP[%s]: client disconnected\n", bindIP);
                    break;
                }
                printf("TCP[%s]: invalid header (%u bytes)\n", bindIP, (unsigned)payloadLength);
                continue;
            }

            len = recv(client_sock, message, payloadLength, MSG_WAITALL);

            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bindIP, len, errno);
                break;
            }

            if (len < 4) {
                // MSG_WAITALL only returns short on disconnect/error.
                printf("TCP[%s]: short frame (%d bytes), dropping connection\n", bindIP, (int)len);
                break;
            }

            memcpy(&header, message, 4);

            if (header & 1u) {
                // Command frame: apply our fields, relay the rest downstream.
                int32_t used = protocol::decode_tcp_command(message, (uint32_t)len);
                if (used < 0) {
                    printf("TCP[%s]: truncated command frame (%d bytes)\n", bindIP, (int)len);
                    continue;
                }
                globalVariableManager.setTcpFromControllerBuffer(
                    message + used, (uint32_t)len - (uint32_t)used);
            } else {
                // Data request: the header only selects fields and carries no
                // values itself, so everything after it is relay data.
                globalVariableManager.setTcpFromControllerBuffer(message + 4, (uint32_t)len - 4u);
            }

            // Echo the request header and fill in the fields it selects.
            uint32_t payloadSize = protocol::encode_tcp_data(header, outBuf + 4, outBufCapacity - 4);
            if (payloadSize == 0) {
                printf("TCP[%s]: response does not fit in %u bytes\n", bindIP,
                       (unsigned)(outBufCapacity - 4));
                continue;
            }

            uint32_t relayed = globalVariableManager.getTcpFromPeripheralBuffer(
                outBuf + 4 + payloadSize, outBufCapacity - 4 - payloadSize, true);

            uint32_t outgoingLengthPrefix =
                (protocol::kTcpIdentifier << 16) | (payloadSize + relayed);
            memcpy(outBuf, &outgoingLengthPrefix, 4);

            int sent = send(client_sock, outBuf, 4 + payloadSize + relayed, 0);
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
      _eth0(config.cW5500_0_CS, config.cW5500_0_INT, 0, config.cDhcpServer),
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

    // Only the board facing the PC serves leases. Downstream ETH_0 ports face
    // another board's ETH_1, which uses a static address and no DHCP client.
    if (_config.cDhcpServer && _boardPosition == 0) {
        esp_err_t dhcpErr = _eth0.start_dhcp_server();
        printf("ETH: DHCP server on ETH_0: %s\n", esp_err_to_name(dhcpErr));
    }

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
    // Stored on the object (not the stack) because the tasks keep these alive.
    // The peripheral task lives on ETH_0 and talks upstream (controller/PC); the
    // controller task lives on ETH_1 and talks downstream (next board).
    _udpConfigPeripheral = { _eth0_ip, _controllerDestIP,
                             _config.cUDP_DESTINATION_PORT, "ETH_0", &_controllerIp };
    _udpConfigController = { _eth1_ip, _peripheralDestIP,
                             _config.cUDP_DESTINATION_PORT, "ETH_1", &_controllerIp };
    _tcpConfigServer = { _eth0_ip, _config.cTCP_LISTEN_PORT, "ETH_0", &_controllerIp };
    _tcpConfigClient = { _peripheralDestIP, _config.cTCP_LISTEN_PORT, "ETH_1", &_controllerIp };

    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreatePinnedToCore(udp_as_peripheral_task, "udp_eth0", 8192, &_udpConfigPeripheral, 12, nullptr, 0);
    xTaskCreatePinnedToCore(udp_as_controller_task, "udp_eth1", 8192, &_udpConfigController, 12, nullptr, 0);
    xTaskCreatePinnedToCore(tcp_as_peripheral_task, "tcp_srv", 12288, &_tcpConfigServer, 12, nullptr, 0);
    xTaskCreatePinnedToCore(tcp_as_controller_task, "tcp_cli", 8192, &_tcpConfigClient, 12, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    logLinkStatus("begin");
    xTaskCreatePinnedToCore(linkMonitorTask, "eth_link", 4096, this, 5, nullptr, 0);
}

void EthernetTask::logLinkStatus(const char *when) {
    printf("ETH: link ETH_0=%s ETH_1=%s (%s)\n",
           isLinkUp(0) ? "UP" : "DOWN",
           isLinkUp(1) ? "UP" : "DOWN",
           when ? when : "");
    _eth0.log_phy_status(when);
    _eth1.log_phy_status(when);
}

void EthernetTask::linkMonitorTask(void *arg) {
    EthernetTask *self = static_cast<EthernetTask *>(arg);
    bool prev0 = self->isLinkUp(0);
    bool prev1 = self->isLinkUp(1);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        bool link0 = self->isLinkUp(0);
        bool link1 = self->isLinkUp(1);
        const bool changed = (link0 != prev0 || link1 != prev1);
        if (changed) {
            self->logLinkStatus("change");
            prev0 = link0;
            prev1 = link1;
        }

        // esp_netif_up() does not start the DHCP server, so a board that booted
        // without a cable (or lost it and came back) has to be re-armed here.
        // start_dhcp_server() is a no-op while the server is already running.
        if (link0 && self->_config.cDhcpServer && self->_boardPosition == 0) {
            esp_err_t err = self->_eth0.start_dhcp_server();
            if (err != ESP_OK && changed) {
                printf("ETH: DHCP server start failed: %s\n", esp_err_to_name(err));
            }
        }
    }
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
