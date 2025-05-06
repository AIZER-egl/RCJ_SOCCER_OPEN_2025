#ifndef PICO_LIB_PID_H
#define PICO_LIB_PID_H

#include <iostream>
#include <bits/algorithmfwd.h>
#include <algorithm>
#include <cmath>
#include "../../pico-lib/time.h"

#define PID_ACCEPT_POSITIVES_ONLY true
#define PID_ACCEPT_NEGATIVES_ONLY false

class PID {
public:
    struct PidParameters {
        float kp{};
        float ki{};
        float kd{};

        double max_output = 0.0;
        double min_output = 0.0;
        float error_threshold = 0.0;

        double integral_error = 0.0;
        double previous_error = 0.0;
        unsigned long long last_iteration_ms = 0;

        bool first_run = true;

        double target = 0.0;
        double error = 0.0;
        double output = 0.0;

        // IF one_direction_only SET TO TRUE
        // it will only accept the values specified in the accept_positives
        bool one_direction_only = false;
        bool accept_type = PID_ACCEPT_POSITIVES_ONLY;

        // IF reset_within_threshold SET TO TRUE
        // it will execute PID::reset if within error_threshold
        bool reset_within_threshold = false;
    };
    static void reset(PidParameters& pid);
    static void compute(PidParameters& pid);
};


#endif //PICO_LIB_PID_H