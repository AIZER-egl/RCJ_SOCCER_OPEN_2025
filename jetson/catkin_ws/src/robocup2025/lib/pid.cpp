#include "pid.h"
#include <ros/ros.h>

uint64_t PID::millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void PID::reset(PidParameters& pid) {
    pid.firstRun = true;
    pid.errorSum = 0;
    pid.previousError = 0;
    pid.lastIterationTime = 0;
}

void PID::compute(PidParameters& pid) {
    if (pid.firstRun) {
        pid.previousError = pid.error;
        pid.firstRun = false;
        pid.lastIterationTime = PID::millis();
    }

    if (PID::millis() - pid.lastIterationTime >= pid.delayMiliseconds) {
        pid.lastIterationTime = PID::millis();
        pid.errorSum += static_cast<float>(pid.error) * static_cast<float>(static_cast<float>(pid.delayMiliseconds) / static_cast<float>(1000));

        if (std::abs(pid.errorSum) > pid.maxError) {
            int multiply = pid.errorSum / std::abs(pid.errorSum);
            pid.errorSum = pid.maxError * multiply;
        }

        float proportionalTerm = pid.kp * pid.error;
        float integralTerm = pid.ki * pid.errorSum;
        float derivativeTerm = pid.kd * (pid.error - pid.previousError);

        int output = proportionalTerm + integralTerm + derivativeTerm;

        if (std::abs(output) > pid.maxOutput) {
            int multiply = output / std::abs(output);
            output = pid.maxOutput * multiply;
        }

        if (std::abs(output) < pid.minOutput) {
            int multiply = output / std::abs(output);
            output = pid.minOutput * multiply;
        }

        if (std::abs(pid.error) < pid.errorThresholdPermission) {
            output = 0;
            reset(pid);
        }

        pid.previousError = pid.error;

        pid.output = output;
    }
}