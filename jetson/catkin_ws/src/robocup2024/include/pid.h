//
// Created by kali on 2/7/24.
//

#ifndef ROBOCUP_2024_PID_H
#define ROBOCUP_2024_PID_H

#include <cmath>
#include <cstdint>
#include <chrono>


class PID {
public:
    static uint64_t millis();

    struct PidParameters {
        bool firstRun = false;

        int maxOutput;
        int minOutput;
        float maxError;
        float errorThresholdPermission;

        float kp;
        float ki;
        float kd;

        float errorSum = 0;
        int previousError = 0;
        int delayMiliseconds = 20;
        unsigned long lastIterationTime = 0;

        int error;
        int output;
        int target;
    };

    static void reset(PidParameters& pid);
    static void compute(PidParameters& pid);
};


#endif //ROBOCUP_2024_PID_H
