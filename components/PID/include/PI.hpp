#pragma once
// Using a separate PI controller will make the inner PI torque loop go faster.

// Voltage-mode PI used by the d/q current loops.
//
// Gains are in SI units: Kp in V/A and Ki in V/(A*s). The caller passes the
// measured loop period in seconds, because the startup windows are
// loop-counted and the FOC period is not guaranteed to stay at one value.
//
// This class only produces the PI part of the voltage command. The caller adds
// its feedforward terms and applies the modulation limit; when that limit
// clamps the total, it calls track() so the integrator is consistent with the
// voltage that can actually be applied.
class PI_Reg {
public:
    PI_Reg(float kp, float ki);

    // Advance the integrator and return the PI output in volts. The integral
    // state is hard-bounded to +/-integralLimit.
    float update(float error, float dt, float integralLimit);

    // Force the integrator so that (kp*error + integral) == output, clamped to
    // +/-integralLimit. Call this when the caller's voltage limit clips the
    // command, so the loop does not wind up against a limit it cannot see.
    void track(float error, float output, float integralLimit);

    void setKp(float kp);
    void setKi(float ki);

    // Clear the integrator. Call this on state transitions so a stale integral
    // from a previous run cannot kick the motor when closed-loop control starts.
    void reset() { _integral = 0.0f; }

private:
    float _kp, _ki;
    float _integral = 0.0f; // volts
};
