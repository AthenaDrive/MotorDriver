
#pragma once
#include <cstdint>

#include "PI.hpp"

struct ControllerParams {
	float dRegKp;
	float dRegKi;
	float qRegKp;
	float qRegKi;
	float maxD;
	float maxQ;
};

struct Output {
	float phaseA;
	float phaseB;
	float phaseC;
};

class Controller {
public:
	Controller(const ControllerParams &params);
	Output update(float iqRef, float electricalPosition, float motorVelocity, float Ia, float Ib);

	// Mirror the beta axis. This is the software equivalent of swapping two
	// motor phase wires and makes the phase order irrelevant.
	void setPhaseOrder(bool reversed) { _betaSign = reversed ? -1 : 1; }
	bool isPhaseOrderReversed() const { return _betaSign < 0; }

private:
	Output _output{};
	PI_Reg _dReg;
	PI_Reg _qReg;

	int8_t _betaSign = 1;

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
	void _PILoop(float iqRef, float motorVelocity);
	void _parkInverse(float sinElectricalPos, float cosElectricalPos);
	void _clarkeInverse();
	void _subtractAvg();
	// Bound..? Keep output in [-1, 1]

	static constexpr float INV_SQRT_3 = 0.57735026918962f;
    static constexpr float SQRT_3_DIV_2 = 0.8660254037;
	static constexpr float INV_3 = 1.0f / 3.0f;

};