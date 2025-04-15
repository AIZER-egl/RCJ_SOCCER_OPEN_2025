//
// Created by kali on 4/1/24.
//

#include <algorithm>
#include <ros/ros.h>
#include "chassis.h"

Chassis::Chassis() {}


void Chassis::moveS(int rpm) {
    RPI_Bitmask::setBitmaskValue(std::abs(rpm), static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::S_SPEED));
    RPI_Bitmask::setBitmaskValue(rpm > 0, static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::S_DIR));
}

void Chassis::moveNE(int rpm) {
    RPI_Bitmask::setBitmaskValue(std::abs(rpm), static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::NE_SPEED));
    RPI_Bitmask::setBitmaskValue(rpm > 0, static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::NE_DIR));
}

void Chassis::moveNW(int rpm) {
    RPI_Bitmask::setBitmaskValue(std::abs(rpm), static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::NW_SPEED));
    RPI_Bitmask::setBitmaskValue(rpm > 0, static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::NW_DIR));
}

int Chassis::move(int rpm, int angle) {
    angle = Angle::to360(angle);
    angle = 360 - angle;



    float radians = angle * PI / 180;

    float s_speed = std::sin(radians + PI);
    float nw_speed = std::sin(radians + 5 * PI / 3);
    float ne_speed = std::sin(radians + PI / 3);

    float max_speed = std::max(std::abs(s_speed), std::max(std::abs(ne_speed), std::abs(nw_speed)));

    s_speed = s_speed / max_speed * rpm;
    ne_speed = ne_speed / max_speed * rpm;
    nw_speed = nw_speed / max_speed * rpm;

//    ROS_INFO("converted - a: %d, s: %f, ne: %f, nw: %f", angle, s_speed, ne_speed, nw_speed);

    moveS(s_speed);
    moveNE(ne_speed);
    moveNW(nw_speed);

    return angle;
}

int Chassis::move(int rpm, int angle, int yaw, PID::PidParameters& pid) {
    angle = Angle::to360(angle);
    angle = 360 - angle;

    float radians = angle * PI / 180;

    float s_speed = std::sin(radians + PI);
    float nw_speed = std::sin(radians + 5 * PI / 3);
    float ne_speed = std::sin(radians + PI / 3);

    float max_speed = std::max(std::abs(s_speed), std::max(std::abs(ne_speed), std::abs(nw_speed)));

    yaw = Angle::to180(yaw);
    pid.error = pid.target - yaw;
    PID::compute(pid);

    s_speed = s_speed / max_speed * rpm - pid.output;
    ne_speed = ne_speed / max_speed * rpm - pid.output;
    nw_speed = nw_speed / max_speed * rpm - pid.output;

    s_speed = Chassis::clamp(s_speed, static_cast<float>(-255), static_cast<float>(255));
    ne_speed = Chassis::clamp(ne_speed, static_cast<float>(-255), static_cast<float>(255));
    nw_speed = Chassis::clamp(nw_speed, static_cast<float>(-255), static_cast<float>(255));


    moveS(s_speed);
    moveNE(ne_speed);
    moveNW(nw_speed);

    ROS_INFO("yaw: %d, out: %d, acc_err: %f, s: %f, ne: %f, nw: %f", yaw, pid.output, pid.errorSum, s_speed, ne_speed, nw_speed);
    return angle;
}

void Chassis::rotate(int rpm) {
    moveS(rpm);
    moveNE(rpm);
    moveNW(rpm);
}

bool Chassis::rotateToAngle(float targetAngle, float yaw, PID::PidParameters& pid) {
    int error = Angle::to180(targetAngle - yaw);
    pid.error = error * -1;

    if (std::abs(error) <= pid.errorThresholdPermission) {
        ROS_INFO("Stopping");
        stop();
        return true;
    }

    PID::compute(pid);

    ROS_INFO("err: %d, yaw: %f, out: %d, sum: %f", error, yaw, pid.output, pid.errorSum);
    moveS(pid.output);
    moveNE(pid.output);
    moveNW(pid.output);

    return false;
}

bool Chassis::rotateToBall(float ballAngle, float yaw, PID::PidParameters& pid) {
    float ballAbsolute = Angle::omni_relative(ballAngle, yaw);
    int error = Angle::to180((ballAbsolute - yaw) * -1);

    // WARNING: Comppute function not called
    pid.error = error;

    if (std::abs(error) < 45) {
        ROS_INFO("Stopping");
        stop();
        return true;
    }

    ROS_INFO("err: %d", error);
    moveS(error);
    moveNE(error);
    moveNW(error);

    return false;
}

void Chassis::stop() {
    moveS(0);
    moveNE(0);
    moveNW(0);
}

float Chassis::map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
