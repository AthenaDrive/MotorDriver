#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

struct UDPDataFromPeripheral {
	uint32_t timestamp;
	float position;
	float velocity;
	float acceleration;
	float torque;
	float phaseCurrentA;
	float phaseCurrentB;
	float phaseCurrentC;
	float busCurrent;
	float busVoltage;
	uint32_t errorRegister;
	uint32_t loopTimeFOC;
	uint32_t loopTimeSecondary;
	float accX;
	float accY;
	float accZ;
	float gyroX;
	float gyroY;
	float gyroZ;

	float debugFloat0;
	float debugFloat1;
	float debugFloat2;
	float debugFloat3;
	int32_t debugInt0;
	int32_t debugInt1;
	uint32_t debugUint0;
	uint32_t debugUint1;
};

static_assert(sizeof(float) == 4, "Protocol requires 32-bit IEEE754 floats.");
static_assert(std::is_standard_layout_v<UDPDataFromPeripheral>,
			  "UDPDataFromPeripheral must remain standard-layout.");

inline constexpr size_t offsetsUDPFromPeripheral[] = {
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
	offsetof(UDPDataFromPeripheral, loopTimeSecondary),
	offsetof(UDPDataFromPeripheral, accX),
	offsetof(UDPDataFromPeripheral, accY),
	offsetof(UDPDataFromPeripheral, accZ),
	offsetof(UDPDataFromPeripheral, gyroX),
	offsetof(UDPDataFromPeripheral, gyroY),
	offsetof(UDPDataFromPeripheral, gyroZ),

	offsetof(UDPDataFromPeripheral, debugFloat0),
	offsetof(UDPDataFromPeripheral, debugFloat1),
	offsetof(UDPDataFromPeripheral, debugFloat2),
	offsetof(UDPDataFromPeripheral, debugFloat3),
	offsetof(UDPDataFromPeripheral, debugInt0),
	offsetof(UDPDataFromPeripheral, debugInt1),
	offsetof(UDPDataFromPeripheral, debugUint0),
	offsetof(UDPDataFromPeripheral, debugUint1)
};

struct UDPDataFromController {
	float torqueSetpoint;
	float velocitySetpoint;
	float positionSetpoint;
	uint32_t drivingMode;

	float debugFloat0;
	float debugFloat1;
	float debugFloat2;
	float debugFloat3;
	int32_t debugInt0;
	int32_t debugInt1;
	uint32_t debugUint0;
	uint32_t debugUint1;
};

static_assert(std::is_standard_layout_v<UDPDataFromController>,
			  "UDPDataFromController must remain standard-layout.");

inline constexpr size_t offsetsUDPFromController[] = {
	offsetof(UDPDataFromController, torqueSetpoint),
	offsetof(UDPDataFromController, velocitySetpoint),
	offsetof(UDPDataFromController, positionSetpoint),
	offsetof(UDPDataFromController, drivingMode),

	offsetof(UDPDataFromController, debugFloat0),
	offsetof(UDPDataFromController, debugFloat1),
	offsetof(UDPDataFromController, debugFloat2),
	offsetof(UDPDataFromController, debugFloat3),
	offsetof(UDPDataFromController, debugInt0),
	offsetof(UDPDataFromController, debugInt1),
	offsetof(UDPDataFromController, debugUint0),
	offsetof(UDPDataFromController, debugUint1)
};
