
#include "driver/gpio.h"
#include "W5500.hpp"

struct EthernetTaskConfig {
    gpio_num_t cW5500_0_CS;
    gpio_num_t cW5500_1_CS;
    gpio_num_t cW5500_0_INT;
    gpio_num_t cW5500_1_INT;
    const char* cW5500_0_IP;
    const char* cW5500_0_GW;
    const char* cW5500_1_IP;
    const char* cW5500_1_GW;
    const char* cW5500_NETMASK;
    uint32_t cTCP_LISTEN_PORT;
    uint32_t cUDP_DESTINATION_PORT;
};

struct _TaskConfigUDP {
    const char* bindIP;
    const char* destionationIP;
    uint32_t UDP_DESTINATION_PORT;
};

struct _TaskConfigTCP {
    const char* bindIP;
    uint32_t TCP_LISTEN_PORT;
};

class EthernetTask {
public:
    EthernetTask(EthernetTaskConfig &config);
    void begin();

    bool isLinkUp(int ix);
    void printIP();

private:
    EthernetTaskConfig _config;
    W5500 _eth0;
    W5500 _eth1;
};