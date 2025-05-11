#include "angle.h"

namespace Angle {
    float toRadians(float angle) {
        return angle * PI / 180;
    }

    float toDegrees(float angle) {
        return angle * 180 / PI;
    }

    bool valid(float angle) {
        return angle >= 0 && angle < 360;
    }

    float normalize(float angle) {
        while (angle < 0) {
            angle += 360;
        }

        while (angle >= 360) {
            angle -= 360;
        }

        return angle;
    }

    float difference(float angle1, float angle2) {
        float diff = angle1 - angle2;
        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        return diff;
    }

    float omni_relative(float angle, float yaw) {
        return normalize(angle + to180(yaw));
    }

    float to180(float angle) {
        angle = normalize(angle);
        if (angle>180) return -(360-angle);
        return angle;
    }

    float to360(float angle) {
        return angle > 0 ? angle : 360 + angle;
    }

    int quadrant(float angle) {
        angle = normalize(angle);
        if (angle < 90) return 1;
        if (angle < 180) return 2;
        if (angle < 270) return 3;
        return 4;
    }
}