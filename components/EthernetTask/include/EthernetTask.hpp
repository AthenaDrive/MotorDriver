#pragma once

#include <atomic>

#include "driver/gpio.h"
#include "W5500.hpp"

struct EthernetTaskConfig {
    gpio_num_t cW5500_0_CS;
    gpio_num_t cW5500_1_CS;
    gpio_num_t cW5500_0_INT;
    gpio_num_t cW5500_1_INT;

    // Static fallback IPs (used if discovery is disabled or fails)
    const char* cW5500_NETMASK;
    const char* cW5500_GW;

    // Ports
    uint32_t cTCP_LISTEN_PORT;
    uint32_t cUDP_DESTINATION_PORT;

    // Auto-IP discovery
    bool      cUseAutoIP;
    int       cDiscRetries;
    int       cDiscTimeoutMs;

    // Serve DHCP to the device on ETH_0 (normally the PC). Only board 0 does
    // this; the daisy-chain links stay static.
    bool      cDhcpServer;
};

struct _TaskConfigUDP {
    const char* bindIP;
    const char* destionationIP;
    uint32_t UDP_DESTINATION_PORT;
    const char* ifKey;
    // Upstream peer learned from traffic (network byte order, 0 = none). Used
    // instead of destionationIP once known.
    std::atomic_uint32_t* controllerIp;
};

struct _TaskConfigTCP {
    const char* bindIP;
    uint32_t TCP_PORT;
    const char* ifKey;
    std::atomic_uint32_t* controllerIp;
};

class EthernetTask {
public:
    EthernetTask(EthernetTaskConfig &config);
    void begin();

    bool isLinkUp(int ix);
    static bool isLinkUp(esp_netif_t* netifInstance);
    void printIP();
    void logLinkStatus(const char *when);

private:
    EthernetTaskConfig _config;
    int   _boardPosition;
    W5500 _eth0;
    W5500 _eth1;

    char _eth0_ip[24];
    char _eth1_ip[24];
    char _controllerDestIP[24];
    char _peripheralDestIP[24];

    // Task arguments outlive begin(); these used to be stack locals passed by
    // pointer to tasks that kept reading them after begin() returned.
    _TaskConfigUDP _udpConfigPeripheral{};
    _TaskConfigUDP _udpConfigController{};
    _TaskConfigTCP _tcpConfigServer{};
    _TaskConfigTCP _tcpConfigClient{};

    // Address of whatever is upstream of us (PC on board 0, previous board's
    // ETH_1 otherwise), learned from the first UDP packet or TCP connection.
    // The static destination IP is the fallback until then.
    std::atomic_uint32_t _controllerIp{0};

    static void linkMonitorTask(void *arg);
};
