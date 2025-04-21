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

#define SHOW_IMAGE
#define RECORD_VIDEO

int main (int argc, char **argv) {

    ros::init(argc, argv, "frontcamera");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("frontcamera_topic", 10);

    ROS_INFO("Using OPENCV version %s", CV_VERSION);

    #ifdef SHOW_IMAGE
        ROS_INFO("Show image ENABLED via compile-time define.");
    #else
        ROS_INFO("Show image DISABLED via compile-time define.");
    #endif

    std::string output_filename;
    bool record_video_flag = false;
    int recording_fps;
    #ifdef RECORD_VIDEO
        record_video_flag = true;
        recording_fps = 14;
        output_filename = "/home/aizer/Documents/RCJ_SOCCER_OPEN_2025/jetson/catkin_ws/src/robocup2024/out/frontcamera_out.mp4";
        ROS_INFO("Video recording ENABLED via compile-time define. FPS: %d, Output file: %s", recording_fps, output_filename.c_str());
    #else
        ROS_INFO("Video recording DISABLED via compile-time define.");
    #endif

    Gstreamer gstreamer;
    gstreamer.set_sensor_id(1);
    gstreamer.set_min_exposure_timerange(320000);
    gstreamer.set_max_exposure_timerange(320000);
    gstreamer.set_framerate(14);
    gstreamer.set_width(2160);
    gstreamer.set_height(2160);

    ROS_INFO("Using command %s", gstreamer.get_command().c_str());

    cv::VideoCapture cap(gstreamer.get_command());

    if (!cap.isOpened()) {
        ROS_FATAL("Could not open camera");
        ros::shutdown();
        return 1;
    }


    cv::VideoWriter video_writer;
    cv::Size frame_size(WIDTH, HEIGHT);

    if (record_video_flag) {
        int codec = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
        video_writer.open(output_filename, codec, recording_fps, frame_size, true);

        if (!video_writer.isOpened()) {
            ROS_ERROR("Could not open the output video file for write: %s", output_filename.c_str());
            ROS_ERROR("Check if the required codec (e.g., FFmpeg backend with MP4V support) is installed.");
            ROS_WARN("Disabling recording for this run due to error. Running anyways");
            record_video_flag = false;
        } else {
            ROS_INFO("Successfully opened video writer for file: %s", output_filename.c_str());
        }
    }

    for (unsigned long frame_id = 0;ros::ok();frame_id++) {
        cv::Mat frame;
        cap >> frame;

        preprocessing::resize(frame, WIDTH, HEIGHT);


        if (record_video_flag && video_writer.isOpened()) {
            video_writer.write(frame);
        }

        #ifdef SHOW_IMAGE
            cv::imshow("Front-Camera", frame);
            if (cv::waitKey(10) == KEY_ESC) {
                break;
            }
        #endif

        ros::spinOnce();
    }

    cap.release();

    if (record_video_flag && video_writer.isOpened()) {
        video_writer.release();
        ROS_INFO("Video writer released. Recording saved to %s", output_filename.c_str());
    }

    #ifdef SHOW_IMAGE
        cv::destroyAllWindows();
    #endif

    return 0;
}

