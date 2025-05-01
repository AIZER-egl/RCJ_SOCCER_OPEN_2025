#include "pid.h"

void PID::reset(PidParameters& pid) {
    pid.integral_error = 0;
    pid.previous_error = 0;
    pid.output = 0;
    pid.error = 0;
    pid.target = 0;
    pid.last_iteration_ms = 0;
    pid.first_run = true;
}

void PID::compute(PidParameters& pid) {
    unsigned long long current_time_ms = millis();

    if (pid.first_run) {
        pid.last_iteration_ms = current_time_ms;
        pid.previous_error = pid.error;
        pid.first_run = false;

        const double initial_pTerm = pid.kp * pid.error;
        double initial_output = initial_pTerm;

        if (pid.min_output > 0.0 && std::fabs(initial_output) < pid.min_output) {
            initial_output = 0.0;
        }

        if (pid.max_output > 0.0) {
            pid.output = std::clamp(initial_output, -pid.max_output, pid.min_output);
        } else {
            pid.output = initial_output;
        }

        return;
    }

    const unsigned long long delta_time_ms = current_time_ms - pid.last_iteration_ms;

    if (delta_time_ms == 0) return;
    const double delta_time_s = static_cast<double>(delta_time_ms) / 1000.0;

    const double pTerm = pid.kp * pid.error;
    bool integrate = true;

    if (std::fabs(pid.error) <= pid.error_threshold) integrate = false;

    if (pid.max_output > 0.0) {
        // Using last runs output to decide (Anti Wind up)
        if (
            ((pid.output >= pid.max_output) && (pid.error > 0))
            || ((pid.output <= -pid.max_output) && (pid.error < 0))
            ) {
            integrate = false;
        }
    }

    if (integrate) {
        pid.integral_error += pid.error * delta_time_s;
    }

    const double iTerm = pid.ki * pid.integral_error;

    const double error_derivate = (pid.error - pid.previous_error) / delta_time_s;
    const double dTerm = pid.kd * error_derivate;

    double calculate_output = pTerm + iTerm + dTerm;

    if (pid.min_output > 0.0 && std::fabs(calculate_output) < pid.min_output) {
        calculate_output = 0.0;
        // Maybe try this?????
        // pid.integral_error = 0
    }

    if (pid.max_output > 0.0) {
        pid.output = std::clamp(calculate_output, -pid.max_output, pid.max_output);
    } else {
        pid.output = calculate_output;
    }

    if (pid.one_direction_only) {
        if (pid.accept_type == PID_ACCEPT_POSITIVES_ONLY && pid.output < 0) pid.output = 0;
        if (pid.accept_type == PID_ACCEPT_NEGATIVES_ONLY && pid.output > 0) pid.output = 0;
    }

    if (pid.reset_within_threshold && (std::fabs(pid.error) < pid.error_threshold)) {
        reset(pid);
    } else {
        pid.previous_error = pid.error;
        pid.last_iteration_ms = millis();
    }
}