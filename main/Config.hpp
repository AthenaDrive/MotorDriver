#pragma once

// ============================================================
// I2C Bus Configuration
// ============================================================
#define I2C_FREQ        400000

// ============================================================
// I2C Device Addresses
// ============================================================

// INA238 (Power Monitor)
// A0, A1 pins set the address:
//   A1=GND, A0=GND  -> 0x40  (default)
//   A1=GND, A0=VCC  -> 0x41
//   A1=VCC, A0=GND  -> 0x44
//   A1=VCC, A0=VCC  -> 0x45
#define INA238_ADDR     0x40

// LM75AD (Temperature Sensor)
// A0, A1, A2 pins: 0x48 + (A2<<2 | A1<<1 | A0)
//   All GND -> 0x48 (default)
#define LM75AD_ADDR     0x48

// LSM6DSO (IMU - Accel + Gyro)
// SA0/SDO pin: 0x6A (SA0=GND) or 0x6B (SA0=VCC)
#define LSM6DSO_ADDR    0x6A

// MCP23017 (GPIO Expander)
// A0, A1, A2 pins: 0x20 + (A2<<2 | A1<<1 | A0)
//   All GND -> 0x20 (default)
#define MCP23017_ADDR   0x20

// ============================================================
// INA238 Power Monitor Configuration
// ============================================================
#define INA238_SHUNT_OHM        0.0035f
#define INA238_MAX_CURRENT_A    20.0f
#define INA238_ADC_CONFIG       (INA238::MODE_CONT_ALL | INA238::VBUSCT_1052US | INA238::VSHCT_1052US | INA238::VTCT_1052US)

// ============================================================
// DRV8323 Current Shunt Amplifier (CSA) Configuration
// ============================================================
#define DRV8323_CSA_GAIN     20.0f    // V/V — SPI reg CSA_CONTROL bits 14:12 (5/10/20/40)
#define DRV8323_SHUNT_OHM    0.0035f   // Ohms — external shunt resistor per phase
#define DRV8323_VREF         1.65f    // V — CSA reference voltage (VCC/2 typical)

// ============================================================
// W5500 Ethernet Static IP Configuration
// ============================================================
#define W5500_0_IP      "192.168.0.18"
#define W5500_0_GW      "192.168.0.1"
#define W5500_1_IP      "192.168.0.17"
#define W5500_1_GW      "192.168.0.1"
#define W5500_NETMASK   "255.255.255.0"

#define TCP_LISTEN_PORT 5001
#define UDP_DEST_PORT   5000