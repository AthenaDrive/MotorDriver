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

    LM75A lm75(i2c, LM75AD_ADDR);
    INA238 ina(i2c, INA238_ADDR);
    LSM6DSO lsm(i2c, LSM6DSO_ADDR);

    ESP_ERROR_CHECK(lm75.init());
    ESP_ERROR_CHECK(ina.init());
    ina.set_adc_config(INA238_ADC_CONFIG);
    ina.calibrate(INA238_SHUNT_OHM, INA238_MAX_CURRENT_A);
    ESP_ERROR_CHECK(lsm.init());

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

    // Start the FOC task first so it can keep reading the encoder even when
    // the bus is unpowered. Its startup state machine waits for the bus and
    // only then initializes and enables the DRV8323.
    FOCTaskConfig focConfig{
        .cDRV8323_CS = DRV8323_CS,
        .cAS5047P_CS = AS5047P_CS,
        .cSPI0_CLK = SPI0_CLK,
        .cSPI0_PICO = SPI0_PICO,
        .cSPI0_POCI = SPI0_POCI,
        .cDRV8323_ENABLE = DRV8323_ENABLE,
        .cDRV8323_INL = DRV8323_INL,
        .cDRV8323_nFAULT = DRV8323_nFAULT,
        .cMinBusVoltage = 9.0f,
        .cStartupSettleMs = 500,
        .cUsePhaseBForFoc = false, // B current is still published, but FOC uses A and C.
    };
    FOCTask focTask{focConfig};
    focTask.begin();

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

    printf("All sensors, SD card, Ethernet, PWM, and ADC initialized.\n");

    float temp, vbus, vshunt, current, power;
    float ax, ay, az, gx, gy, gz, lsm_temp;
    
    while (1) {
        int64_t t0 = esp_timer_get_time();

        if (lm75.read_temperature(temp) == ESP_OK) {
            // printf("LM75A: %.2f C\n", temp);
            globalVariableManager.setTemperature(temp);
        }

        // Publish the bus voltage on its own so the FOC startup state machine
        // is not blocked by a failure in one of the other INA238 channels.
        if (ina.read_bus_voltage(vbus) == ESP_OK) {
            globalVariableManager.setBusVoltage(vbus);
        }

        if (ina.read_shunt_voltage(vshunt) == ESP_OK &&
            ina.read_current(current) == ESP_OK &&
            ina.read_power(power) == ESP_OK) {
                // printf("INA238: %.3fV %.3fmV %.3fA %.3fW\n", vbus, vshunt, current, power);
                globalVariableManager.setBusCurrent(-current); // Negative because i have switched up the wires.. stupid.
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
