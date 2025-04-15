//
// Created by kali on 4/4/24.
//

#ifndef ROBOCUP2024_ANGLE_H
#define ROBOCUP2024_ANGLE_H

#define PI 3.1415926

namespace Angle {
    float toRadians(float angle);
    float toDegrees(float angle);

    bool valid (float angle);

    float normalize(float angle);
    float difference(float angle1, float angle2);

    float omni_relative(float angle, float yaw);

    float to180(float angle);
    float to360(float angle);

    int quadrant(float angle);
}

#endif //ROBOCUP2024_ANGLE_H
