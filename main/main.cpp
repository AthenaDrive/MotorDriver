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
#include "AS5047P.hpp"
#include "DRV8323.hpp"
#include "SdCard.hpp"
#include "MCPWMDriver.hpp"
#include "ADCOneshot.hpp"
#include "EthernetTask.hpp"
#include "GlobalVariableManager.hpp"
#include "Pinout.hpp"
#include "Config.hpp"

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

    mcp.pin_mode(MCP_PIN_B2, false); // Motor FAULT
    mcp.pin_mode(DRV8323_ENABLE, true);
    mcp.digital_write(DRV8323_ENABLE, true);

    // --- SPI Bus 0: encoder ---
    gpio_set_level(DRV8323_CS, 1);
    gpio_set_direction(DRV8323_CS, GPIO_MODE_OUTPUT);

    SPIBase spi0(SPI2_HOST, SPI0_CLK, SPI0_PICO, SPI0_POCI);
    ESP_ERROR_CHECK(spi0.init());

    AS5047P enc(spi0, AS5047P_CS);
    ESP_ERROR_CHECK(enc.init());

    DRV8323 drv(spi0, DRV8323_CS, 1, 500000);
    ESP_ERROR_CHECK(drv.init());
    drv.set_3x_pwm_mode();

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
        .cW5500_0_IP = W5500_0_IP,
        .cW5500_0_GW = W5500_0_GW,
        .cW5500_1_IP = W5500_1_IP,
        .cW5500_1_GW = W5500_1_GW,
        .cW5500_NETMASK = W5500_NETMASK,
        .cTCP_LISTEN_PORT = TCP_LISTEN_PORT,
        .cUDP_DESTINATION_PORT = UDP_DEST_PORT,
    };
    EthernetTask ethernetTask{ethConfig};
    ethernetTask.begin();

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

    drv.set_3x_pwm_mode();

    uint16_t drvReg;
    for (int i = 0; i < 8; i++) {
        auto err = drv.read_register(i, drvReg);
        if (err != ESP_OK) {
            printf("Error when reading register %i.\n", i);
        }
        printf("Register %i: %i\n", i, drvReg);
    }
    

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
            globalVariableManager.setAngle(angle);
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
