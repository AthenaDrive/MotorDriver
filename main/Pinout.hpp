#pragma once

#include "hal/gpio_types.h"

// ============================================================
// I2C Bus
// ============================================================
#define PIN_SDA     GPIO_NUM_11
#define PIN_SCL     GPIO_NUM_12

// ============================================================
// SPI Buses
// ============================================================
#define SPI0_CLK    GPIO_NUM_14
#define SPI0_PICO   GPIO_NUM_47
#define SPI0_POCI   GPIO_NUM_21

#define SPI1_CLK    GPIO_NUM_8
#define SPI1_PICO   GPIO_NUM_9
#define SPI1_POCI   GPIO_NUM_18

// ============================================================
// SPI Device Chip-Select Pins
// ============================================================
#define DRV8323_CS      GPIO_NUM_48
#define AS5047P_CS      GPIO_NUM_13
#define W5500_0_CS      GPIO_NUM_17
#define W5500_1_CS      GPIO_NUM_15

// ============================================================
// W5500 Interrupt Pins
// ============================================================
#define W5500_0_INT     GPIO_NUM_16
#define W5500_1_INT     GPIO_NUM_7

// ============================================================
// DRV8323 3-Phase PWM Inputs (3x PWM mode, low-side GPIO)
// ============================================================
#define DRV8323_INHA    GPIO_NUM_39
#define DRV8323_INHB    GPIO_NUM_40
#define DRV8323_INHC    GPIO_NUM_41

#define DRV8323_INL     GPIO_NUM_42

// ============================================================
// DRV8323 Control Pins
// ============================================================
#define DRV8323_nFAULT  GPIO_NUM_3
#define DRV8323_ENABLE  GPIO_NUM_38

// ============================================================
// Brake chopper PWM
// ============================================================
#define BRAKE_CHOPPER_PWM GPIO_NUM_1

// ============================================================
// ADC Phase Current Sensing
// ============================================================
#define ADC_CUR_A       GPIO_NUM_6
#define ADC_CUR_B       GPIO_NUM_5
#define ADC_CUR_C       GPIO_NUM_4


// ============================================================
// External pins
// ============================================================

#define CS_EXTERNAL     GPIO_NUM_10
