
#pragma once
#include <cstdint>

#include "PI.hpp"

struct ControllerParams {
	float dRegKp;	// V/A
	float dRegKi;	// V/(A*s)
	float qRegKp;	// V/A
	float qRegKi;	// V/(A*s)
	// Maximum d/q voltage vector magnitude in duty percentage points. One
	// point is busVoltage/100 volts, so 30 means a 30% modulation limit.
	float voltageLimitPercent;
};

struct Output {
	float phaseA;
	float phaseB;
	float phaseC;
};

class Controller {
public:
	Controller(const ControllerParams &params);

	// busVoltage is the measured DC link voltage (volts); the controller
	// converts its volt-domain output to duty percentage points with it.
	// dtSeconds is the measured FOC period, used by the PI integrators.
	Output update(float iqRef, float electricalPosition, float motorVelocity,
	              float busVoltage, float dtSeconds, float Ia, float Ib);

	// Mirror the beta axis. This is the software equivalent of swapping two
	// motor phase wires and makes the phase order irrelevant.
	void setPhaseOrder(bool reversed) { _betaSign = reversed ? -1 : 1; }
	bool isPhaseOrderReversed() const { return _betaSign < 0; }

	// Clear both integrators. Call on state transitions (for example when the
	// startup sequence hands over to closed-loop control).
	void reset() { _dReg.reset(); _qReg.reset(); }

	// Telemetry helpers. Iq is the measured q-axis current in amps, VqPercent
	// is the applied q-axis voltage in duty points after the vector limit.
	float getIq() const { return _parkIq; }
	float getVqPercent() const { return _parkVqPercent; }
	bool isSaturated() const { return _saturated; }

private:
	Output _output{};
	PI_Reg _dReg;
	PI_Reg _qReg;

	int8_t _betaSign = 1;

	const float _voltageLimitPercent;

	float _parkVqPercent = 0.0f;
	bool _saturated = false;

	float _clarkeIa = 0.0f;
	float _clarkeIb = 0.0f;
	float _parkId = 0.0f;
	float _parkIq = 0.0f;
	float _parkVd = 0.0f;
	float _parkVq = 0.0f;
	float _clarkeVa = 0.0f;
	float _clarkeVb = 0.0f;

	void _clarkeTransform(float Ia, float Ib);
	void _parkTransform(float sinElectricalPos, float cosElectricalPos);
	void _PILoop(float iqRef, float motorVelocity, float dtSeconds, float voltageLimit);
	void _parkInverse(float sinElectricalPos, float cosElectricalPos);
	void _clarkeInverse();
	void _subtractAvg();

	// Below this the bus measurement is not usable for the volts -> duty
	// conversion. The FOC state machine already requires a valid bus before
	// closed-loop operation, so this only catches a dead or stale reading.
	static constexpr float MIN_BUS_VOLTAGE = 1.0f;

	static constexpr float INV_SQRT_3 = 0.57735026918962f;
	static constexpr float SQRT_3_DIV_2 = 0.8660254037f;
	static constexpr float INV_3 = 1.0f / 3.0f;
};
