#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

struct UDPDataFromPeripheral {
	uint32_t index;
	uint32_t timestamp;
	float position;
	float velocity;
	float acceleration;
	float torque;
	float phaseCurrentA;
	float phaseCurrentB;
	float phaseCurrentC;
	uint32_t busCurrent;
	uint32_t busVoltage;
	uint32_t errorRegister;
	uint32_t loopTimeFOC;
	uint32_t loopTimeSecondary;
};

static_assert(sizeof(float) == 4, "Protocol requires 32-bit IEEE754 floats.");
static_assert(std::is_standard_layout_v<UDPDataFromPeripheral>,
			  "UDPDataFromPeripheral must remain standard-layout.");

inline constexpr size_t offsetsUDPFromPeripheral[] = {
	offsetof(UDPDataFromPeripheral, index),
	offsetof(UDPDataFromPeripheral, timestamp),
	offsetof(UDPDataFromPeripheral, position),
	offsetof(UDPDataFromPeripheral, velocity),
	offsetof(UDPDataFromPeripheral, acceleration),
	offsetof(UDPDataFromPeripheral, torque),
	offsetof(UDPDataFromPeripheral, phaseCurrentA),
	offsetof(UDPDataFromPeripheral, phaseCurrentB),
	offsetof(UDPDataFromPeripheral, phaseCurrentC),
	offsetof(UDPDataFromPeripheral, busCurrent),
	offsetof(UDPDataFromPeripheral, busVoltage),
	offsetof(UDPDataFromPeripheral, errorRegister),
	offsetof(UDPDataFromPeripheral, loopTimeFOC),
	offsetof(UDPDataFromPeripheral, loopTimeSecondary)
};

struct UDPDataFromController {
	float torqueSetpoint;
	float velocitySetpoint;
	float positionSetpoint;
};

static_assert(std::is_standard_layout_v<UDPDataFromController>,
			  "UDPDataFromController must remain standard-layout.");

inline constexpr size_t offsetsUDPFromController[] = {
	offsetof(UDPDataFromController, torqueSetpoint),
	offsetof(UDPDataFromController, velocitySetpoint),
	offsetof(UDPDataFromController, positionSetpoint)
};

static_assert(sizeof(UDPDataFromController) == 12,
			  "Unexpected UDPDataFromController size.");

static_assert(sizeof(UDPDataFromPeripheral) ==
				  sizeof(uint32_t) * 5 + sizeof(float) * 9,
			  "Unexpected UDPDataFromPeripheral size.");