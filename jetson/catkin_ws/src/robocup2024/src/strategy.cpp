#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>
#include "rpi-bitmask.h"
#include "chassis.h"
#include "angle.h"
#include "pid.h"

#define DEFEND true
#define ATTACK false
#define MODE DEFEND

float omni_ballDistance;
float omni_ballAngle;

float ballCenterX;
float ballCenterY;
float goalCenterX;
float goalCenterY;
float goalCorner1X;
float goalCorner1Y;
float goalCorner2X;
float goalCorner2Y;
float goalAngle1;
float goalAngle2;
float goalAngle;
float ballAngle;
float ballDistance;
float goalCenterDistance;

PID::PidParameters anglePID;
PID::PidParameters defendPID;
PID::PidParameters chasePID;

void omni_message(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    omni_ballDistance = msg -> data[0];
    omni_ballAngle = msg -> data[1];
}

void front_message(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    ballCenterX = msg -> data[0];
    ballCenterY = msg -> data[1];
    goalCenterX = msg -> data[2];
    goalCenterY = msg -> data[3];
    goalCorner1X = msg -> data[4];
    goalCorner1Y = msg -> data[5];
    goalCorner2X = msg -> data[6];
    goalCorner2Y = msg -> data[7];
    goalAngle1 = msg -> data[8];
    goalAngle2 = msg -> data[9];
    goalCenterDistance = msg -> data[10];
    goalAngle = msg -> data[11];
    ballAngle = msg -> data[12];
    ballDistance = msg -> data[13];
}

void rpi_message(const std_msgs::UInt64::ConstPtr& msg) {
    RPI_Bitmask::setBitmask(msg -> data);
}

bool intercepts () {
    if (goalCorner1X == 9999) return false;
    if (goalCorner2X == 9999) return false;
    if (ballCenterX == 9999) return false;

    return (goalCorner1X < ballCenterX) && (ballCenterX < goalCorner2X);
}

uint64_t time123 = 0;
bool firstRun = true;
bool previouslySeen = false;
bool closeToGoal = false;
uint64_t lastTime = 0;
int i = 0;
int angle = 0;
bool up = true;

void followBall(int robot_yaw) {
    int offset = ballCenterX - 320;
    chasePID.target = 320;
    chasePID.error = offset;
    PID::compute(chasePID);

    ROS_INFO("Following ball, distance: %f, angle: %f, goal_d: %f, out: %d, error: %d", ballDistance, ballAngle, goalCenterDistance, chasePID.output, chasePID.error);
    if (chasePID.output == 0) {
        angle = Chassis::move(100, 0, robot_yaw, anglePID);
    } else {
        angle = Chassis::move(100, 0, robot_yaw + chasePID.output, anglePID);
    }
}

void attack(int robot_yaw) {
        int ballQuadrant = Angle::quadrant(omni_ballAngle);
    int bias = 0;
    if (ballQuadrant == 1 || ballQuadrant == 2) bias = 50;
    if (ballQuadrant == 3 || ballQuadrant == 4) bias = - 50;

    if (ballDistance < 10) bias += bias > 0 ? 20 : -20;

    int omni_ballDistance_copy = omni_ballDistance;
    if (omni_ballDistance_copy > 50) omni_ballDistance_copy = 50;

    int speed = Chassis::map(omni_ballDistance_copy, 0, 95, 70, 120);

    if (ballAngle == 9999) {
        ROS_INFO("Moving %d, speed: %d, yaw: %d, ball_angle: %f, quadrant: %d, bias: %d", static_cast<int>(omni_ballAngle + bias), speed, robot_yaw, omni_ballAngle, ballQuadrant, bias);
        angle = Chassis::move(speed, omni_ballAngle + bias, robot_yaw, anglePID);
        return;
    }

    followBall(robot_yaw);
}

void publish_task(ros::Publisher& pub, ros::Publisher& settings) {
    lastTime = PID::millis();
    while (ros::ok()) {
        int robot_yaw = RPI_Bitmask::bitmaskToValue(static_cast<int>(RPI_Bitmask::COMPASS_ADDRESS::YAW));

        if (omni_ballAngle != -1 || ballAngle != 9999) {
            previouslySeen = true;
            if (MODE == ATTACK) {
                 attack(robot_yaw);
            } else {
                attack(robot_yaw);
            }
        } else {
            Chassis::stop();
            PID::reset(anglePID);
            previouslySeen = false;
            ROS_INFO("Unable to locate ball");
        }


        std_msgs::UInt64 msg;
        msg.data = RPI_Bitmask::generateBitmask();
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 30));
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "strategy");
    ros::NodeHandle nh;

    ROS_INFO("Listening for messages");

    anglePID.kp = 0.32;
    anglePID.ki = 0.23;
    anglePID.kd = 0.21;
    anglePID.maxOutput = 150;
    anglePID.minOutput = 0;
    anglePID.maxError = 200;
    anglePID.errorThresholdPermission = 5;
    anglePID.target = 0;
    anglePID.delayMiliseconds = 20;

    defendPID.kp = 1;
    defendPID.ki = 0;
    defendPID.kd = 0;
    defendPID.maxOutput = 150;
    defendPID.minOutput = 0;
    defendPID.maxError = 200;
    defendPID.errorThresholdPermission = 5;
    defendPID.target = 0;
    defendPID.delayMiliseconds = 20;

    chasePID.kp = 1;
    chasePID.ki = 0;
    chasePID.kd = 0;
    chasePID.maxOutput = 90;
    chasePID.minOutput = 0;
    chasePID.maxError = 200;
    chasePID.errorThresholdPermission = 5;
    chasePID.target = 0;
    chasePID.delayMiliseconds = 20;

    ros::Subscriber omni_sub = nh.subscribe("omnicamera_topic", 10, omni_message);
    ros::Subscriber front_sub = nh.subscribe("frontcamera_topic", 10, front_message);
    ros::Subscriber rpi_sub = nh.subscribe("rpi2jetson", 10, rpi_message);
    ros::Publisher settings = nh.advertise<std_msgs::Bool>("settings", 10);
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("jetson2rpi", 10);

    std::thread publish_thread(publish_task, std::ref(pub), std::ref(settings));

    ros::spin();

    publish_thread.join();

    return 0;
}
