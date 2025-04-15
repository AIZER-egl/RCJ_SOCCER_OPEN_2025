#include <cmath>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

#include <opencv4/opencv2/opencv.hpp>

#include "preprocessing.h"
#include "blob_detection.h"
#include "gstreamer.h"

#define KEY_ESC 27
#define WIDTH 1080
#define HEIGHT 1080
#define PI 3.1415926
#define EULER 2.71828

#define SHOW_IMG

#define BLUE true
#define YELLOW false
bool TEAM = BLUE;

bool ballInterceptWithGoal(BlobDetection::Blob ball, BlobDetection::Blob goal) {
    bool leftIntercept = ball.x < goal.x;
    bool rightIntercept = (ball.x + ball.w) > (goal.x + goal.w);
    return leftIntercept && rightIntercept;
}

void settings_callback(const std_msgs::Bool::ConstPtr& msg) {
    TEAM = msg -> data;
}

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


int main (int argc, char **argv) {

    ros::init(argc, argv, "frontcamera");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("frontcamera_topic", 10);
    ros::Subscriber settings = nh.subscribe("settings", 10, settings_callback);

    ROS_INFO("Using OPENCV version %s", CV_VERSION);

    auto gstreamer = new Gstreamer();
    gstreamer -> set_sensor_id(0);
    gstreamer -> set_min_exposure_timerange(320000);
    gstreamer -> set_max_exposure_timerange(320000);
    gstreamer -> set_framerate(14);
    gstreamer -> set_width(2160);
    gstreamer -> set_height(2160);
    gstreamer -> set_awb_lock(false);
    gstreamer -> set_white_balance(Gstreamer::WhiteBalance::DAYLIGHT);

    ROS_INFO("Using command %s", (gstreamer -> get_command()).c_str());

    cv::VideoCapture cap(gstreamer -> get_command());

    if (!cap.isOpened()) {
        ROS_FATAL("Could not open camera");
        ros::shutdown();
        return 1;
    }

    for (unsigned long frame_id = 0;ros::ok();frame_id++) {
        cv::Mat frame;
        cap >> frame;

        preprocessing::resize(frame, WIDTH, HEIGHT);
//        cv::rectangle(frame, cv::Point(0, 0), cv::Point(WIDTH - 1, 30), cv::Scalar(255, 255, 255), cv::FILLED);

        cv::imwrite("/home/aizer/frontcamera/frame_" + std::to_string(frame_id) + ".jpg", frame);

        #ifdef SHOW_IMG
        cv::imshow("Front-Camera", frame);

        if (cv::waitKey(10) == KEY_ESC) {
            break;
        }
        #endif

    }

    cap.release();

    #ifdef SHOW_IMG
    cv::destroyAllWindows();
    #endif

    return 0;
}

