#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/netif.h"
#include "I2CBase.hpp"
#include "SPIBase.hpp"
#include "MCP23017.hpp"
#include "LM75A.hpp"
#include "INA238.hpp"
#include "LSM6DSO.hpp"
#include "AS5047P.hpp"
#include "DRV8323.hpp"
#include "SdCard.hpp"
#include "W5500.hpp"
#include "MCPWMDriver.hpp"
#include "ADCOneshot.hpp"
#include "Pinout.hpp"
#include "Config.hpp"

float position;

// Shared ADC sample — written by adc_sampling_task, read by main loop
static volatile struct {
    int raw[3];
} s_adc;

static TaskHandle_t s_adc_task_handle = nullptr;

static bool IRAM_ATTR on_pwm_peak(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t *, void *) {
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(s_adc_task_handle, &wake);
    return wake == pdTRUE;
}

static void adc_sampling_task(void *arg) {
    auto *adc = static_cast<ADCOneshot *>(arg);
    s_adc_task_handle = xTaskGetCurrentTaskHandle();

    int r0, r1, r2;
    uint32_t skip = 0;

    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_1, 0);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(GPIO_NUM_1, 1);
        if (++skip < 3) {
            gpio_set_level(GPIO_NUM_1, 0);
            continue;
        }
        skip = 0;

        
        adc->read_raw(ADCOneshot::CHANNEL_A, r0);
        adc->read_raw(ADCOneshot::CHANNEL_B, r1);
        adc->read_raw(ADCOneshot::CHANNEL_C, r2);
        s_adc.raw[0] = r0; s_adc.raw[1] = r1; s_adc.raw[2] = r2;
        gpio_set_level(GPIO_NUM_1, 0);
        // TODO: signal FOC task that new ADC sample is ready
    }
}

static const char *UDP_DEST_IP = "192.168.0.17";
static const uint16_t UDP_DEST_PORT = 5000;

static void udp_send_task(void *arg) {
    const char *bind_ip = static_cast<const char *>(arg);

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr(bind_ip);
    bind_addr.sin_port = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        printf("UDP[%s]: socket() errno=%d\n", bind_ip, errno);
        vTaskDelete(nullptr);
    }

    bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    struct sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_DEST_IP);
    dest_addr.sin_port = htons(UDP_DEST_PORT);

    printf("UDP sender %s -> %s:%d\n", bind_ip, UDP_DEST_IP, UDP_DEST_PORT);

    // Header ...00011
    // => 2 values sent.

    static uint32_t iteration = 0;
    uint8_t packet[16];

    uint32_t header = 0b111;

    while (1) {
        uint32_t iter = iteration++;
        float pos = position;
        auto time = esp_timer_get_time();
        // printf("Sending: %li, %f.\n", iter, pos);

        memcpy(packet + 0, &header, 4);
        memcpy(packet + 4, &iter, 4);
        memcpy(packet + 8, &time, 4);
        memcpy(packet + 12, &pos, 4);

        int sent = sendto(sock, packet, sizeof(packet), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            printf("UDP[%s]: sendto() errno=%d\n", bind_ip, errno);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    close(sock);
    vTaskDelete(nullptr);
}

static void tcp_echo_task(void *arg) {
    const char *bind_ip = static_cast<const char *>(arg);

    struct sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr(bind_ip);
    bind_addr.sin_port = htons(TCP_LISTEN_PORT);

    int listen_sock;
    while (1) {
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            printf("TCP[%s]: socket() errno=%d\n", bind_ip, errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(listen_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            printf("TCP[%s]: bind() errno=%d\n", bind_ip, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (listen(listen_sock, 1) < 0) {
            printf("TCP[%s]: listen() errno=%d\n", bind_ip, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP[%s] listening on port %d\n", bind_ip, TCP_LISTEN_PORT);

        struct sockaddr_in client_addr = {};
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            printf("TCP[%s]: accept() errno=%d\n", bind_ip, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP[%s] client connected\n", bind_ip);
        close(listen_sock);

        uint8_t buf[1024];
        while (1) {
            int len = recv(client_sock, buf, sizeof(buf), 0);
            if (len <= 0) {
                printf("TCP[%s]: client disconnected (len=%d errno=%d)\n", bind_ip, len, errno);
                break;
            }
            int sent = send(client_sock, buf, len, 0);
            if (sent < 0) {
                printf("TCP[%s]: send() errno=%d\n", bind_ip, errno);
                break;
            }
        }

        close(client_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void) {
    // --- I2C bus & sensors ---
    I2CBase i2c(PIN_SDA, PIN_SCL, I2C_FREQ);
    ESP_ERROR_CHECK(i2c.init());

    MCP23017 mcp(i2c, MCP23017_ADDR);
    LM75A lm75(i2c, LM75AD_ADDR);
    INA238 ina(i2c, INA238_ADDR);
    LSM6DSO lsm(i2c, LSM6DSO_ADDR);

    ESP_ERROR_CHECK(mcp.init());
    ESP_ERROR_CHECK(lm75.init());
    ESP_ERROR_CHECK(ina.init());
    ina.set_adc_config(INA238_ADC_CONFIG);
    ina.calibrate(INA238_SHUNT_OHM, INA238_MAX_CURRENT_A);
    ESP_ERROR_CHECK(lsm.init());

    mcp.pin_mode(MCP_PIN_A0, true);
    mcp.pin_mode(MCP_PIN_A3, false);

    mcp.pin_mode(DRV8323_INLA, true);
    mcp.pin_mode(DRV8323_INLB, true);
    mcp.pin_mode(DRV8323_INLC, true);
    mcp.digital_write(DRV8323_INLA, false);
    mcp.digital_write(DRV8323_INLB, false);
    mcp.digital_write(DRV8323_INLC, false);

    mcp.pin_mode(DRV8323_ENABLE, true);
    mcp.digital_write(DRV8323_ENABLE, true);

    // --- SPI Bus 0: encoder ---
    gpio_set_level(DRV8323_CS, 1);
    gpio_set_direction(DRV8323_CS, GPIO_MODE_OUTPUT);

    SPIBase spi0(SPI2_HOST, SPI0_CLK, SPI0_PICO, SPI0_POCI);
    ESP_ERROR_CHECK(spi0.init());

    AS5047P enc(spi0, AS5047P_CS);
    ESP_ERROR_CHECK(enc.init());

    DRV8323 drv(spi0, DRV8323_CS);
    ESP_ERROR_CHECK(drv.init());
    drv.set_3x_pwm_mode();

    // --- SPI Bus 1: SD card + 2x W5500 ---
    gpio_set_level(W5500_0_CS, 1);
    gpio_set_level(W5500_1_CS, 1);
    gpio_set_direction(W5500_0_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(W5500_1_CS, GPIO_MODE_OUTPUT);

    // Both SdCard (sdspi_host) and W5500 (esp_eth) use SPI3_HOST directly,
    // so initialize it manually here rather than through SPIBase.
    spi_bus_config_t bus1_cfg = {};
    bus1_cfg.mosi_io_num = SPI1_PICO;
    bus1_cfg.miso_io_num = SPI1_POCI;
    bus1_cfg.sclk_io_num = SPI1_CLK;
    bus1_cfg.quadwp_io_num = -1;
    bus1_cfg.quadhd_io_num = -1;
    bus1_cfg.data4_io_num = -1;
    bus1_cfg.data5_io_num = -1;
    bus1_cfg.data6_io_num = -1;
    bus1_cfg.data7_io_num = -1;
    bus1_cfg.max_transfer_sz = 4096;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus1_cfg, SPI_DMA_CH_AUTO));

    SdCard sd(SPI3_HOST, SD_CARD_CS);
    ESP_ERROR_CHECK(sd.init());

    // Network stack init (needed by W5500 ethernet driver)
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Install ISR service for W5500 Ethernet interrupt pins
    gpio_install_isr_service(0);

    W5500 eth0(W5500_0_CS, W5500_0_INT, 0);
    W5500 eth1(W5500_1_CS, W5500_1_INT, 1);

    ESP_ERROR_CHECK(eth0.init());
    ESP_ERROR_CHECK(eth1.init());

    // Wait up to 12 seconds for each port's link to come up
    auto wait_link = [](W5500 &eth, int idx) {
        char ifname[8];
        if (esp_netif_get_netif_impl_name(eth.netif(), ifname) != ESP_OK) {
            printf("W5500[%d] cannot get interface name\n", idx);
            return false;
        }
        for (int i = 0; i < 20; i++) {
            struct netif *n = netif_find(ifname);
            if (n && netif_is_link_up(n)) {
                printf("W5500[%d] link UP (if=%s)\n", idx, ifname);
                return true;
            }
            if (i % 10 == 0)
                printf("W5500[%d] waiting for link...\n", idx);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        printf("W5500[%d] link DOWN (check cable / wiring)\n", idx);
        return false;
    };
    wait_link(eth0, 0);
    eth0.set_static_ip(W5500_0_IP, W5500_NETMASK, W5500_0_GW);
    wait_link(eth1, 1);
    eth1.set_static_ip(W5500_1_IP, W5500_NETMASK, W5500_1_GW);

    // Print assigned IP to confirm static IP took effect
    {
        esp_netif_ip_info_t ip;
        char ip_s[16], mask_s[16];
        if (esp_netif_get_ip_info(eth0.netif(), &ip) == ESP_OK) {
            esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
            esp_ip4addr_ntoa(&ip.netmask, mask_s, sizeof(mask_s));
            printf("W5500[0] IP=%s MASK=%s\n", ip_s, mask_s);
        }
        if (esp_netif_get_ip_info(eth1.netif(), &ip) == ESP_OK) {
            esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
            esp_ip4addr_ntoa(&ip.netmask, mask_s, sizeof(mask_s));
            printf("W5500[1] IP=%s MASK=%s\n", ip_s, mask_s);
        }
    }

    // Start UDP sender on eth0 only
    xTaskCreate(udp_send_task, "udp_eth0", 4096, (void *)W5500_0_IP, 12, nullptr);

    // Start TCP echo client on eth0 only
    xTaskCreate(tcp_echo_task, "tcp_echo", 4096, (void *)W5500_0_IP, 12, nullptr);

    // --- ADC: phase current sensing (DRV8323 CSA outputs) ---
    ADCOneshot adc;
    ESP_ERROR_CHECK(adc.init(DRV8323_SHUNT_OHM, DRV8323_CSA_GAIN, DRV8323_VREF));

    // ADC sampling task — starts first so s_adc_task_handle is valid before PWM ISR fires
    xTaskCreate(adc_sampling_task, "adc_sample", 4096, &adc, 10, nullptr);
    vTaskDelay(pdMS_TO_TICKS(10));  // ensure task sets handle before PWM starts

    // --- MCPWM: DRV8323 3-phase PWM ---
    MCPWMDriver pwm;
    ESP_ERROR_CHECK(pwm.init(30000, 40000000, on_pwm_peak, nullptr));

    vTaskDelay(pdMS_TO_TICKS(500));
    printf("All sensors, SD card, Ethernet, PWM, and ADC initialized.\n");

    float temp, vbus, vshunt, current, power;
    float ax, ay, az, gx, gy, gz, lsm_temp, angle;

    while (1) {
        bool switchSignal = false;
        mcp.digital_read(MCP_PIN_A3, switchSignal);
        mcp.digital_write(MCP_PIN_A0, switchSignal);

        // Example: ramp duty on all 3 phases from 0% to 50%
        static float duty = 0.0f;
        static int dir = 1;
        duty += 0.5f * dir;
        if (duty >= 50.0f) dir = -1;
        if (duty <= 0.0f) dir = 1;
        pwm.set_duty(MCPWMDriver::CHANNEL_A, duty);
        pwm.set_duty(MCPWMDriver::CHANNEL_B, duty);
        pwm.set_duty(MCPWMDriver::CHANNEL_C, duty);

        // Read latest ADC sample (sampled by adc_sampling_task on PWM peak)
        int r0 = s_adc.raw[0], r1 = s_adc.raw[1], r2 = s_adc.raw[2];
        float a0, a1, a2, mv0, mv1, mv2;
        adc.raw_to_current(r0, a0);
        adc.raw_to_current(r1, a1);
        adc.raw_to_current(r2, a2);
        adc.calibrate_raw(r0, mv0);
        adc.calibrate_raw(r1, mv1);
        adc.calibrate_raw(r2, mv2);

        if (lm75.read_temperature(temp) == ESP_OK) {
            // printf("LM75A: %.2f C\n", temp);
        }

        if (ina.read_bus_voltage(vbus) == ESP_OK &&
            ina.read_shunt_voltage(vshunt) == ESP_OK &&
            ina.read_current(current) == ESP_OK &&
            ina.read_power(power) == ESP_OK) {
                // printf("INA238: %.3fV %.3fmV %.3fA %.3fW\n", vbus, vshunt, current, power);
            }

        if (lsm.read_accel(ax, ay, az) == ESP_OK &&
            lsm.read_gyro(gx, gy, gz) == ESP_OK &&
            lsm.read_temperature(lsm_temp) == ESP_OK) {
                // printf("LSM6DSO: accel(%.2f %.2f %.2f) gyro(%.2f %.2f %.2f) %.2fC\n",
                //    ax, ay, az, gx, gy, gz, lsm_temp);
            }
            

        if (enc.pipeline_read_angle(angle) == ESP_OK) {
            // printf("AS5047P: %.2f deg\n", angle);
            position = angle;
        }

        {
            uint16_t drv_fault, drv_vgs;
            if (drv.read_fault_status(drv_fault) == ESP_OK) {
                if (drv.has_fault(drv_fault, DRV8323::FAULT_FLT)) {
                    // printf("DRV8323: FAULT=0x%04X\n", drv_fault);
                }
            }
            if (drv.read_vgs_status(drv_vgs) == ESP_OK) {
                if (drv_vgs) {
                    // printf("DRV8323: VGS=0x%04X\n", drv_vgs);
                }
            }
        }

        // printf("PWM duty: %.1f%% | ADC: raw(%d %d %d) mV(%.0f %.0f %.0f) A(%.3f %.3f %.3f)\n",
        //     duty, r0, r1, r2, mv0, mv1, mv2, a0, a1, a2);

        // FILE *f = fopen("/sdcard/test.txt", "a");
        // if (f) {
        //     fprintf(f, "angle=%.2f\n", angle);
        //     fclose(f);
        // }

        // printf("---\n");

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
