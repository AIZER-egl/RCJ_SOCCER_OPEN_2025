#include "pid.h"

#include <bits/algorithmfwd.h>

void PID::reset(PidParameters& pid) {
    pid.integralError = 0;
    pid.previousError = 0;
    pid.lastIteration_ms = 0;
}

void PID::compute(PidParameters& pid) {
    if (millis() - pid.lastIteration_ms >= pid.delay_ms) {
        const double pTerm = pid.kp * pid.error;
        const double delta_s = static_cast<double>(millis() - pid.lastIteration_ms) / 1000;

        pid.integralError += pid.error * delta_s;
        const double maxIntegralContribution = pid.maxOutput / 2.0;
        const double minIntegralContribution = -pid.maxOutput / 2.0;

        if (pid.ki != 0) {
            pid.integralError = std::clamp(pid.integralError, minIntegralContribution / pid.ki, maxIntegralContribution / pid.ki);
        } else {
            pid.integralError = 0;
        }
        const double iTerm = pid.ki * pid.integralError;

        double derivativeError = 0;
        if (delta_s > 1e-9) {
            derivativeError = (pid.error - pid.previousError) / delta_s;
        }
        const double dTerm = pid.kd * derivativeError;

        double output = pTerm + iTerm + dTerm;

        output = std::clamp(output, static_cast<double>(-pid.maxOutput), static_cast<double>(pid.maxOutput));

        if (std::abs(pid.error) < pid.errorThreshold) {
            output = 0;
            reset(pid);
        }

        pid.previousError = pid.error;
        pid.lastIteration_ms = millis();

        pid.output = output;
    }
}