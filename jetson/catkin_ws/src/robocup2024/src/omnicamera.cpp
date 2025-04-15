#include <cmath>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv4/opencv2/opencv.hpp>

#include "preprocessing.h"
#include "blob_detection.h"

#define KEY_ESC 27
#define WIDTH 670
#define HEIGHT 380
#define PI 3.1415926
#define EULER 2.71828

bool g_shutdown_request = false;
cv::VideoCapture cap("nvarguscamerasrc sensor-id=1 exposuretimerange='210000 256000' ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)1280, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)1280, height=(int)1280, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");

void sigintHandler(int sig) {
	ROS_INFO("Shutdown request detected, shutting down");
	cap.release();
    g_shutdown_request = true;
	exit(0);
}

int getQuadrant(int x, int y) {
	if (x > 0 && y > 0) return 1;
	if (x < 0 && y > 0) return 2;
	if (x < 0 && y < 0) return 3;
	return 4;
}

double getAngle(float x, float y) {
	int quadrant = getQuadrant(x, y);
	double angle;
	switch (quadrant) {
		case 1:
			angle = std::abs(atan(y / x) * 180 / PI);
			break;
		case 2:
			angle = 180 - std::abs(atan(y / x) * 180 / PI);
			break;
		case 3:
			angle = 180 + std::abs(atan(y / x) * 180 / PI);
			break;
		case 4:
			angle = 360 - std::abs(atan(y / x) * 180 / PI);
			break;
	}
	angle -= 90;
    if (angle < 0) angle += 360;
    return angle;
}

float getDistance(float x, float y, bool inside) {
    float pixelDistance = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

    if (inside) {
        return (1038.9212) / (1 + 437.881 * std::pow(EULER, -0.0255 * pixelDistance));
    }

    return 2.4 + 3.69 * std::sin(0.07 * pixelDistance + 0.63);
}

int main (int argc, char **argv) {

    ros::init(argc, argv, "omnicamera");

    // signal(SIGINT, sigintHandler);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("omnicamera_topic", 10);

    ROS_INFO("Using OPENCV version %s", CV_VERSION);

    if (!cap.isOpened()) {
        ROS_FATAL("Could not open camera");
        ros::shutdown();
        return 1;
    }

    BlobDetection ballDetection;
    ballDetection.set_color_range(cv::Scalar(0, 16, 103), cv::Scalar(23, 75, 255));
    ballDetection.set_area(5, 100000);

    for (int frame_id = 0;ros::ok();frame_id++) {
        cv::Mat frame;
        cap >> frame;

        preprocessing::resize(frame, WIDTH, HEIGHT);
        preprocessing::contrast(frame, 1.2, 2.3);
        preprocessing::gamma_correction(frame, 1.9);
	    preprocessing::brightness(frame, 1.4);
        preprocessing::saturation(frame, 2);
    	cv::flip(frame, frame, 0);

        cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());
        cv::ellipse(mask, cv::Point(WIDTH / 2, HEIGHT / 2), cv::Size(200, 200), 0, 0, 360, cv::Scalar(255, 255, 255), -1);
        cv::bitwise_and(frame, mask, frame);


        int major_axis = 150;
        int minor_axis = 150;
        cv::ellipse(frame, cv::Point(WIDTH / 2, HEIGHT / 2), cv::Size(major_axis, minor_axis), 0, 0, 360, cv::Scalar(255, 255, 0), 2);
        std::vector<BlobDetection::Blob> blobs = ballDetection.detect(frame);

        float distance = -1;
        float angle = -1;
        if (!blobs.empty()) {
            BlobDetection::Blob biggestBlob = blobs.at(0);

            int cx = biggestBlob.x + biggestBlob.w / 2;
            int cy = biggestBlob.y + biggestBlob.h / 2;
            float x = static_cast<float>(biggestBlob.x - WIDTH / 2 + biggestBlob.w / 2);
            float y = static_cast<float>(biggestBlob.y - HEIGHT / 2 + biggestBlob.h / 2);

            cv::rectangle(frame,cv::Point(biggestBlob.x, biggestBlob.y),cv::Point(biggestBlob.x + biggestBlob.w, biggestBlob.y + biggestBlob.h),cv::Scalar(255, 0, 0),2);
            cv::line(frame,cv::Point(WIDTH / 2, HEIGHT / 2),cv::Point(x + WIDTH / 2, HEIGHT / 2),cv::Scalar(0, 255, 0),2);
            cv::line(frame,cv::Point(x + WIDTH / 2, HEIGHT / 2),cv::Point(x + WIDTH / 2, y + HEIGHT / 2),cv::Scalar(0, 255, 0),2);
            cv::line(frame,cv::Point(WIDTH / 2, HEIGHT / 2),cv::Point(x + WIDTH / 2, y + HEIGHT / 2),cv::Scalar(0, 255, 0),2);

            float ellipseEquation = (std::pow(cx - WIDTH / 2, 2) / std::pow(major_axis, 2) + (std::pow(cy - HEIGHT / 2, 2) / std::pow(minor_axis, 2)));
            angle = getAngle(x, y);
            distance = getDistance(x, y, ellipseEquation < 1);
        }

        ROS_INFO("oDistance %f", static_cast<float>(distance));
        ROS_INFO("oAngle %f", static_cast<float>(angle));

        std_msgs::Float32MultiArray msg;
        msg.data.resize(2);
        msg.data[0] = distance;
        msg.data[1] = angle;

        pub.publish(msg);
        cv::imwrite("/home/aizer/omnicamera/frame_" + std::to_string(frame_id) + ".jpg", frame);

//       cv::imshow("Omni-camera", frame);

  //     if (cv::waitKey(10) == KEY_ESC) {
    //       break;
      // }
    }

    ROS_INFO("Video loop finished");
    cap.release();
    //video.release();
    cv::destroyAllWindows();

    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = -1;
    msg.data[1] = -1;
    pub.publish(msg);

    return 0;
}
