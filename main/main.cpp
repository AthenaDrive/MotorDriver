#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "I2CBase.hpp"
#include "SPIBase.hpp"
#include "MCP23017.hpp"
#include "LM75A.hpp"
#include "INA238.hpp"
#include "LSM6DSO.hpp"
#include "SdCard.hpp"
#include "EthernetTask.hpp"
#include "FOCTask.hpp"
#include "GlobalVariableManager.hpp"
#include "Pinout.hpp"
#include "Config.hpp"

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
    mcp.pin_mode(MCP_PIN_A1, true);
    mcp.pin_mode(MCP_PIN_A2, true);
    mcp.pin_mode(MCP_PIN_A3, false);

    mcp.pin_mode(DRV8323_INLA, true);
    mcp.pin_mode(DRV8323_INLB, true);
    mcp.pin_mode(DRV8323_INLC, true);
    mcp.digital_write(DRV8323_INLA, false);
    mcp.digital_write(DRV8323_INLB, false);
    mcp.digital_write(DRV8323_INLC, false);

    mcp.pin_mode(MCP_PIN_B2, false); // Motor FAULT
    mcp.pin_mode(DRV8323_ENABLE, true);
    mcp.digital_write(DRV8323_ENABLE, true);

    gpio_install_isr_service(0);
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

    // SdCard sd(SPI3_HOST, SD_CARD_CS);
    // ESP_ERROR_CHECK(sd.init());

    EthernetTaskConfig ethConfig{
        .cW5500_0_CS = W5500_0_CS,
        .cW5500_1_CS = W5500_1_CS,
        .cW5500_0_INT = W5500_0_INT,
        .cW5500_1_INT = W5500_1_INT,
        .cW5500_NETMASK = W5500_NETMASK,
        .cW5500_GW = W5500_GW,
        .cTCP_LISTEN_PORT = TCP_LISTEN_PORT,
        .cUDP_DESTINATION_PORT = UDP_DEST_PORT,
        .cUseAutoIP = W5500_USE_AUTO_IP,
        .cDiscRetries = DISC_RETRIES,
        .cDiscTimeoutMs = DISC_TIMEOUT_MS,
    };
    EthernetTask ethernetTask{ethConfig};
    ethernetTask.begin();

    printf("Wating for high voltage.\n");
    float startupVoltage = 0.0f;
    while (startupVoltage < 9.0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        printf(".");
        ina.read_bus_voltage(startupVoltage);
    }
    printf("\nPowered up!\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    mcp.digital_write(DRV8323_INLA, true);
    mcp.digital_write(DRV8323_INLB, true);
    mcp.digital_write(DRV8323_INLC, true);

    FOCTaskConfig focConfig{
        .cDRV8323_CS = DRV8323_CS,
        .cAS5047P_CS = AS5047P_CS,
        .cSPI0_CLK = SPI0_CLK,
        .cSPI0_PICO = SPI0_PICO,
        .cSPI0_POCI = SPI0_POCI,
    };
    FOCTask focTask{focConfig};
    focTask.begin();

    vTaskDelay(pdMS_TO_TICKS(500));
    printf("All sensors, SD card, Ethernet, PWM, and ADC initialized.\n");

    float temp, vbus, vshunt, current, power;
    float ax, ay, az, gx, gy, gz, lsm_temp, angle;
    
    while (1) {
        int64_t t0 = esp_timer_get_time();

        bool switch0, switch1;
        mcp.digital_read(DIP_SWITCH_0, switch0);
        mcp.digital_read(DIP_SWITCH_1, switch1);
        globalVariableManager.setButtonStatus((switch0 << 1) + switch1);

        uint32_t ledStatus = globalVariableManager.getLedStatus();
        mcp.digital_write(LED_0, (ledStatus & 1) == 1);
        mcp.digital_write(LED_1, (ledStatus & 2) == 2);
        mcp.digital_write(LED_2, (ledStatus & 4) == 4);

        if (lm75.read_temperature(temp) == ESP_OK) {
            // printf("LM75A: %.2f C\n", temp);
            globalVariableManager.setTemperature(temp);
        }

        if (ina.read_bus_voltage(vbus) == ESP_OK &&
            ina.read_shunt_voltage(vshunt) == ESP_OK &&
            ina.read_current(current) == ESP_OK &&
            ina.read_power(power) == ESP_OK) {
                // printf("INA238: %.3fV %.3fmV %.3fA %.3fW\n", vbus, vshunt, current, power);
                globalVariableManager.setBusVoltage(vbus);
                globalVariableManager.setBusCurrent(current);
            }

        if (lsm.read_accel(ax, ay, az) == ESP_OK &&
            lsm.read_gyro(gx, gy, gz) == ESP_OK &&
            lsm.read_temperature(lsm_temp) == ESP_OK) {
                // printf("LSM6DSO: accel(%.2f %.2f %.2f) gyro(%.2f %.2f %.2f) %.2fC\n",
                //    ax, ay, az, gx, gy, gz, lsm_temp);

                globalVariableManager.setAccX(ax);
                globalVariableManager.setAccY(ay);
                globalVariableManager.setAccZ(az);
                globalVariableManager.setGyroX(gx);
                globalVariableManager.setGyroY(gy);
                globalVariableManager.setGyroZ(gz);
            }

        int64_t t1 = esp_timer_get_time();
        globalVariableManager.setAvgLoopTimeSecondary(t1 - t0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
