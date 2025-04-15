//
// Created by kali on 4/1/24.
//

#ifndef ROBOCUP2024_CHASSIS_H
#define ROBOCUP2024_CHASSIS_H

#include <cmath>
#include "pid.h"
#include "angle.h"
#include "rpi-bitmask.h"

#define PI 3.1415926


class Chassis {
public:
    Chassis();

    template<typename T>
    static constexpr T clamp(const T& value, const T& low, const T& high) {
        return (value < low) * low + (value >= low && value <= high) * value + (value > high) * high;
    }

    static void moveS(int rpm);
    static void moveNE(int rpm);
    static void moveNW(int rpm);

    static int move(int rpm, int angle);
    static int move(int rpm, int angle, int yaw, PID::PidParameters& pid);
    static void rotate(int rpm);

    static bool rotateToAngle(float targetAngle, float yaw, PID::PidParameters& pid);
    static bool rotateToBall(float ballAngle, float yaw, PID::PidParameters& pid);

    static float map(float x, float in_min, float in_max, float out_min, float out_max);

    static void stop();
};


#endif //ROBOCUP2024_CHASSIS_H
