#pragma once

#include "CommunicationConfig.hpp"
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
};

struct _TaskConfigUDP {
    const char* bindIP;
    const char* destionationIP;
    uint32_t UDP_DESTINATION_PORT;
    const char* ifKey;
};

struct _TaskConfigTCP {
    const char* bindIP;
    uint32_t TCP_PORT;
    const char* ifKey;
};

class EthernetTask {
public:
    EthernetTask(EthernetTaskConfig &config);
    void begin();

    bool isLinkUp(int ix);
    static bool isLinkUp(esp_netif_t* netifInstance);
    void printIP();

private:
    EthernetTaskConfig _config;
    int   _boardPosition;
    W5500 _eth0;
    W5500 _eth1;

    char _eth0_ip[24];
    char _eth1_ip[24];
    char _controllerDestIP[24];
    char _peripheralDestIP[24];
};
