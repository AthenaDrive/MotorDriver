#pragma once
#include <stdint.h>

#define DISCOVERY_PORT      5003
#define DISCOVERY_MAGIC     0x41544C41  // "ATLA"
#define DISCOVERY_VER       1

#define DISC_REQ  0
#define DISC_RESP 1

#pragma pack(push, 1)
struct DiscoveryPacket {
    uint32_t magic;
    uint8_t  type;
    uint8_t  version;
    uint8_t  position;
    uint8_t  reserved;
    uint8_t  sender_mac[6];
};
#pragma pack(pop)

// Returns this board's position in the daisy chain (0 = first).
// On return, both interfaces are reconfigured to sequential IPs.
int discovery_run(int disc_retries, int disc_timeout_ms);

// Spawns a FreeRTOS task that listens on ETH_0 for downstream
// discovery requests and responds with our position.
void discovery_start_responder(int my_position);
