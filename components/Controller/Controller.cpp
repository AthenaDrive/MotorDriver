#include "cmath"

#include "Controller.hpp"
#include "esp_attr.h"

// Single-precision sin/cos with a quadrant reduction and minimax polynomials.
// newlib's sinf/cosf run a full __kernel_rem_pio2f range reduction for every
// call (the electrical angle is outside [-pi/4, pi/4]), which was a large part
// of the control loop. The FOC angle is in [0, 4*pi), so rounding to a quadrant
// with a float->int conversion is safe here.
static inline void fastSinCos(float x, float &sinOut, float &cosOut) {
	constexpr float INV_PI_2 = 0.63661977236758134308f;
	constexpr float PI_2 = 1.57079632679489661923f;

	const float qf = x * INV_PI_2;
	const int q = static_cast<int>(qf + 0.5f);
	const float r = x - static_cast<float>(q) * PI_2;
	const float r2 = r * r;

	const float sinR = r * (1.0f + r2 * (-1.6666654611e-1f +
	                   r2 * (8.3321608736e-3f +
	                   r2 * (-1.9515295891e-4f +
	                   r2 * 2.7557314297e-6f))));

	const float cosR = 1.0f + r2 * (-0.5f +
	                   r2 * (4.1666667908e-2f +
	                   r2 * (-1.3888889225e-3f +
	                   r2 * (2.4801587642e-5f +
	                   r2 * -2.7557314297e-7f))));

	switch (q & 3) {
	case 0: sinOut = sinR;  cosOut = cosR;  break;
	case 1: sinOut = cosR;  cosOut = -sinR; break;
	case 2: sinOut = -sinR; cosOut = -cosR; break;
	default: sinOut = -cosR; cosOut = sinR; break;
	}
}

Controller::Controller(const ControllerParams& params)
	: _dReg(params.dRegKp, params.dRegKi),
	  _qReg(params.qRegKp, params.qRegKi),
	  _voltageLimitPercent(params.voltageLimitPercent) {}

IRAM_ATTR Output Controller::update(
	const float iqRef,
	const float electricalPosition,
	const float motorVelocity,
	const float busVoltage,
	const float dtSeconds,
	const float Ia, const float Ib)
{
	// The PI loops and the feedforward model work in volts, but the PWM takes
	// duty percentage points and one point is busVoltage/100 volts. Convert
	// here so Kp stays in V/A and the loop gain does not change when the
	// supply voltage changes. Without a usable bus measurement the bridge
	// stays at neutral; the FOC state machine gates closed-loop operation on
	// the bus voltage anyway.
	if (!(busVoltage > MIN_BUS_VOLTAGE)) {
		_saturated = false;
		_parkVqPercent = 0.0f;
		_output.phaseA = 0.0f;
		_output.phaseB = 0.0f;
		_output.phaseC = 0.0f;
		return _output;
	}

	const float pctPerVolt = 100.0f / busVoltage;

	float sinPos, cosPos;
	fastSinCos(electricalPosition, sinPos, cosPos);

	_clarkeTransform(Ia, Ib);
	_parkTransform(sinPos, cosPos);
	_PILoop(iqRef, motorVelocity, dtSeconds,
	        0.01f * _voltageLimitPercent * busVoltage);

	// Volts -> duty percentage points.
	_parkVd *= pctPerVolt;
	_parkVq *= pctPerVolt;
	_parkVqPercent = _parkVq;

	_parkInverse(sinPos, cosPos);
	_clarkeInverse();
	_subtractAvg();

	return _output;
}


IRAM_ATTR void Controller::_clarkeTransform(const float Ia, const float Ib) {
	_clarkeIa = Ia;
	// Bitshift is faster?
	// _betaSign swaps the roles of phase B and C, so a reversed motor phase
	// sequence still produces the same logical alpha/beta currents.
	_clarkeIb = _betaSign * (Ia + 2 * Ib) * INV_SQRT_3;
}

IRAM_ATTR void Controller::_parkTransform(const float sinElectricalPos, const float cosElectricalPos) {

	// Coult be faster with matrix mult?
	// Is just a simple roation matrix.
	_parkId = cosElectricalPos * _clarkeIa + sinElectricalPos * _clarkeIb;
	_parkIq = -sinElectricalPos * _clarkeIa + cosElectricalPos * _clarkeIb;
}

IRAM_ATTR void Controller::_PILoop(const float iqRef, const float motorVelocity,
                                   const float dtSeconds, const float voltageLimit) {
    // The PI zero is placed at Ki/Kp = R/L so it cancels the electrical pole.
    // Do not add motor_r * i as a separate feedforward: that cancels the same
    // pole a second time and turns the response into a lightly damped
    // second-order one (verified ~32% overshoot, ~50% with one sample of
    // feedback delay). The integrator absorbs the resistance instead.
    // Back-EMF and the d/q cross-coupling are real disturbances and stay.
    constexpr float idRef = 0.0f;
    const float errD = idRef - _parkId;
    const float errQ = iqRef - _parkIq;

    constexpr float motor_l = 0.000016f;   // Phase inductance (H)
    constexpr float motor_flux = 0.00292f; // Flux linkage (Wb)

    const float vdFF = -motorVelocity * motor_l * _parkIq;
    const float vqFF = motorVelocity * motor_l * _parkId
                     + motorVelocity * motor_flux;

    float vd = _dReg.update(errD, dtSeconds, voltageLimit) + vdFF;
    float vq = _qReg.update(errQ, dtSeconds, voltageLimit) + vqFF;

    // Limit the d/q vector magnitude, not each phase. Clipping the phases
    // after the inverse Clarke would rotate the applied vector and inject
    // current on the other axis when the loop saturates. The PI output alone
    // cannot be bounded because the feedforward above uses part of the same
    // voltage budget, so back-calculate the integrators from the voltage that
    // is actually applied.
    _saturated = false;
    const float vmag2 = vd * vd + vq * vq;
    const float limit2 = voltageLimit * voltageLimit;
    if (vmag2 > limit2) {
        const float scale = voltageLimit / std::sqrt(vmag2);
        vd *= scale;
        vq *= scale;
        _dReg.track(errD, vd - vdFF, voltageLimit);
        _qReg.track(errQ, vq - vqFF, voltageLimit);
        _saturated = true;
    }

    _parkVd = vd;
    _parkVq = vq;
}

IRAM_ATTR void Controller::_parkInverse(const float sinElectricalPos, const float cosElectricalPos) {
	_clarkeVa = cosElectricalPos * _parkVd - sinElectricalPos * _parkVq;
	_clarkeVb = sinElectricalPos * _parkVd + cosElectricalPos * _parkVq;
}

IRAM_ATTR void Controller::_clarkeInverse() {
	_output.phaseA = _clarkeVa;

	// Bitshift faster? Constexpr sqrt(3) / 2?
	// Mirror beta when the motor phase sequence is reversed. This swaps the
	// physical B and C outputs.
	const float vb = _betaSign * _clarkeVb;
	const float term0 = - _clarkeVa * 0.5f;
	const float term1 = vb * SQRT_3_DIV_2;
	_output.phaseB = term0 + term1;
	_output.phaseC = term0 - term1;
}

IRAM_ATTR void Controller::_subtractAvg() {
	float avg = (_output.phaseA + _output.phaseB + _output.phaseC) * INV_3;
	_output.phaseA -= avg;
	_output.phaseB -= avg;
	_output.phaseC -= avg;
}
