#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"
#include <net/if.h>

#include "Discovery.hpp"

static void bind_socket_to_netif(int sock, const char *ifKey) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ifKey);
    if (!netif) {
        printf("DISC bind: ifKey '%s' not found\n", ifKey);
        return;
    }
    char ifname[16];
    if (esp_netif_get_netif_impl_name(netif, ifname) != ESP_OK) {
        printf("DISC bind: get_impl_name failed for '%s'\n", ifKey);
        return;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strlcpy(ifr.ifr_name, ifname, sizeof(ifr.ifr_name));
    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifr, sizeof(ifr)) < 0) {
        printf("DISC SO_BINDTODEVICE(%s): errno=%d\n", ifname, errno);
    }
}

// Wait for PHY link on a given netif, up to timeout_ms.
static bool wait_for_link(const char *ifKey, int timeout_ms) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ifKey);
    if (!netif) return false;

    char ifname[16];
    if (esp_netif_get_netif_impl_name(netif, ifname) != ESP_OK) return false;

    int waited = 0;
    while (waited < timeout_ms) {
        struct netif *n = netif_find(ifname);
        if (n && netif_is_link_up(n)) return true;
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    return false;
}

static int create_discovery_socket(const char *ifKey, uint16_t port) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        printf("DISC socket(%s): errno=%d\n", ifKey, errno);
        return -1;
    }

    bind_socket_to_netif(sock, ifKey);

    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // Bind to the interface's own IP, not INADDR_ANY, so that two
    // interfaces can bind the same port without conflict.
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ifKey);
    if (!netif) {
        printf("DISC: no netif for '%s'\n", ifKey);
        close(sock);
        return -1;
    }
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        printf("DISC: no IP for '%s'\n", ifKey);
        close(sock);
        return -1;
    }

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ip_info.ip.addr;
    addr.sin_port = htons(port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("DISC bind(%s): errno=%d\n", ifKey, errno);
        close(sock);
        return -1;
    }

    return sock;
}

static void send_req(int sock) {
    DiscoveryPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic   = DISCOVERY_MAGIC;
    pkt.type    = DISC_REQ;
    pkt.version = DISCOVERY_VER;
    pkt.position = 0;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_ETH);
    memcpy(pkt.sender_mac, mac, 6);

    struct sockaddr_in bcast = {};
    bcast.sin_family = AF_INET;
    bcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    bcast.sin_port = htons(DISCOVERY_PORT);

    sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&bcast, sizeof(bcast));
}

// Returns -1 on timeout, or upstream board's position on response
static int recv_resp(int sock, int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };

    int sel = select(sock + 1, &fds, NULL, NULL, &tv);
    if (sel <= 0) return -1;

    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    DiscoveryPacket pkt;
    ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &fromlen);
    if (n < (ssize_t)sizeof(pkt)) return -1;
    if (pkt.magic != DISCOVERY_MAGIC || pkt.type != DISC_RESP) return -1;

    return (int)pkt.position;
}

// Check if a REQ arrived on the ETH_0 listener socket
// Returns true if at least one REQ was received (and discarded for now)
static bool check_req_on_eth0(int sock, int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };

    int sel = select(sock + 1, &fds, NULL, NULL, &tv);
    if (sel <= 0) return false;

    char buf[sizeof(DiscoveryPacket)];
    ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
    if (n < (ssize_t)sizeof(DiscoveryPacket)) return false;

    DiscoveryPacket *pkt = (DiscoveryPacket *)buf;
    return (pkt->magic == DISCOVERY_MAGIC && pkt->type == DISC_REQ);
}

int discovery_run(int retries, int timeout_ms) {
    printf("DISC: starting discovery (retries=%d, timeout=%dms)\n", retries, timeout_ms);

    // Random initial stagger (0-500ms) to desync simultaneous boots
    vTaskDelay(pdMS_TO_TICKS(esp_random() % 500));
    printf("DISC: staggered start\n");

    // ETH_0 = upstream (faces PC/previous board)
    bool eth0_link = wait_for_link("ETH_0", 5000);
    printf("DISC: ETH_0 link=%s\n", eth0_link ? "UP" : "DOWN");

    if (!eth0_link) {
        printf("DISC: no upstream cable -> position 0\n");
        return 0;
    }

    // sock_up = upstream (ETH_0): send REQ, receive RESP
    // sock_down = downstream (ETH_1): listen for downstream REQs
    int sock_up   = create_discovery_socket("ETH_0", DISCOVERY_PORT);
    int sock_down = create_discovery_socket("ETH_1", DISCOVERY_PORT);
    if (sock_up < 0 || sock_down < 0) {
        if (sock_up >= 0) close(sock_up);
        if (sock_down >= 0) close(sock_down);
        printf("DISC: socket creation failed, defaulting to position 0\n");
        return 0;
    }

    int position = -1;

    auto try_recv = [&](int sock) -> bool {
        int upstream_pos = recv_resp(sock, 0);
        if (upstream_pos >= 0) {
            position = upstream_pos + 1;
            printf("DISC: upstream pos=%d -> my position=%d\n", upstream_pos, position);
            return true;
        }
        return false;
    };

    for (int attempt = 0; attempt < retries; attempt++) {
        send_req(sock_up);

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock_up, &readfds);
        FD_SET(sock_down, &readfds);
        struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };

        int sel = select((sock_up > sock_down ? sock_up : sock_down) + 1,
                         &readfds, NULL, NULL, &tv);

        if (sel > 0) {
            if (FD_ISSET(sock_up, &readfds) && try_recv(sock_up)) break;
            if (FD_ISSET(sock_down, &readfds)) {
                check_req_on_eth0(sock_down, 0);
            }
        }

        printf("DISC: attempt %d/%d, no response\n", attempt + 1, retries);
    }

    if (position < 0) {
        for (int extra = 0; extra < 4 && position < 0; extra++) {
            send_req(sock_up);
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sock_up, &readfds);
            struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
            int sel = select(sock_up + 1, &readfds, NULL, NULL, &tv);
            if (sel > 0) try_recv(sock_up);
        }
    }

    if (position < 0) {
        position = 0;
        printf("DISC: no upstream found -> position 0\n");
    }

    close(sock_up);
    close(sock_down);
    return position;
}

// ---- Responder task (runs on ETH_0, helps downstream boards) ----

struct ResponderArg {
    int my_position;
};

static void responder_task(void *arg) {
    ResponderArg *ra = (ResponderArg *)arg;
    int my_pos = ra->my_position;
    delete ra;

    printf("DISC_RESP: starting responder on ETH_1 (my pos=%d)\n", my_pos);

    int sock = create_discovery_socket("ETH_1", DISCOVERY_PORT);
    if (sock < 0) {
        printf("DISC_RESP: socket failed\n");
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    while (1) {
        DiscoveryPacket pkt;
        ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &fromlen);
        if (n < (ssize_t)sizeof(pkt)) continue;
        if (pkt.magic != DISCOVERY_MAGIC || pkt.type != DISC_REQ) continue;

        // Respond with our position
        DiscoveryPacket resp;
        memset(&resp, 0, sizeof(resp));
        resp.magic    = DISCOVERY_MAGIC;
        resp.type     = DISC_RESP;
        resp.version  = DISCOVERY_VER;
        resp.position = (uint8_t)my_pos;

        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_ETH);
        memcpy(resp.sender_mac, mac, 6);

        sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr *)&from, fromlen);
    }

    close(sock);
    vTaskDelete(nullptr);
}

void discovery_start_responder(int my_position) {
    ResponderArg *arg = new ResponderArg;
    arg->my_position = my_position;
    xTaskCreate(responder_task, "disc_resp", 4096, arg, 8, nullptr);
}
