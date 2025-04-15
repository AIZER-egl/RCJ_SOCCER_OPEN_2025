#ifndef PICO_LIB_PID_H
#define PICO_LIB_PID_H

#include <cmath>
#include <algorithm>
#include "../../pico-lib/time.h"

class PID {
public:
    struct  PidParameters {
        bool firstRun = false;

        int maxOutput{};
        float maxError{};
        float errorThreshold{};

        float kp{};
        float ki{};
        float kd{};

        int delay_ms = 20;
        double integralError = 0;
        double previousError = 0;
        unsigned long lastIteration_ms = 0;

        double error{};
        double output{};
        double target{};
    };
    static void reset(PidParameters& pid);
    static void compute(PidParameters& pid);
};


#endif //PICO_LIB_PID_H
