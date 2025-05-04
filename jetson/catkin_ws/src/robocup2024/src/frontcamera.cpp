#include <cmath>
#include <string>
#include <chrono>
#include <ctime>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/cuda.hpp>

#include "preprocessing.h"
#include "blob_detection.h"
#include "gstreamer.h"

#define KEY_ESC 27
#define WIDTH 1536
#define HEIGHT 946
#define PI 3.1415926
#define EULER 2.71828

//#define SHOW_IMAGE
#define RECORD_VIDEO

#define DISPLAY_HEIGHT 473
#define DISPLAY_WIDTH 768

#define INVALID_VALUE 999
#define FRAME_RATE 14

#define IGNORE_BEFORE_Y 51

void on_mouse (int event, int x, int y, int flags, void* userdata) {
	(void)userdata;
	(void)flags;

	if (event == cv::EVENT_LBUTTONDOWN) {
		ROS_INFO("(x: %d, y: %d)", x, y);
	}
}

std::string getCurrentDateTimeString() {
	auto now = std::chrono::system_clock::now();
	auto now_c = std::chrono::system_clock::to_time_t(now);
	struct tm *parts = std::localtime(&now_c);
	char buffer[80];
	strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", parts);
	return std::string(buffer);
}

long long getMillis() {
	auto now = std::chrono::system_clock::now();
	auto duration = now.time_since_epoch();
	auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
	long long millis_count = millis.count();
	return millis_count;
}

float get_distance_cm (float distance_pixels) {
	const double A = 1510000;
	const double omega = -422.8;
	const double beta = .000004322;
	const double C = -7.529;

	const double first_term = std::pow(std::pow(omega, 2) - std::pow(distance_pixels, 2), 2);
	const double second_term = 4 * std::pow(distance_pixels, 2) * std::pow(beta, 2);
	return A / std::sqrt(first_term + second_term) + C;
}

void drawIgnoreArea(cv::Mat& image, int ignore_before_y,
                    const cv::Scalar& color = cv::Scalar(0, 0, 0),
                    int thickness = cv::FILLED)
{
    if (image.empty()) {
        std::cerr << "Error: La imagen de entrada está vacía." << std::endl;
        return;
    }

    if (ignore_before_y <= 0) {
        return;
    }
    int width = image.cols;
    int actual_y = std::min(ignore_before_y, image.rows);
    cv::Point pt1(0, 0);
    cv::Point pt2(width, actual_y);
    cv::rectangle(image, pt1, pt2, color, thickness, cv::LINE_8, 0);
}


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
	#ifdef RECORD_VIDEO
		record_video_flag = true;
		output_filename = "/home/aizer/Documents/RCJ_SOCCER_OPEN_2025/jetson/catkin_ws/src/robocup2024/out/frontcamera_out_";
		output_filename += getCurrentDateTimeString();
		output_filename += ".mp4";

		ROS_INFO("Video recording ENABLED via compile-time define. FPS: %d, Output file: %s", FRAME_RATE, output_filename.c_str());
	#else
		ROS_INFO("Video recording DISABLED via compile-time define.");
	#endif

	Gstreamer gstreamer;
	gstreamer.set_sensor_id(0);
	gstreamer.set_framerate(FRAME_RATE);
	gstreamer.set_width(WIDTH);
	gstreamer.set_height(HEIGHT);

	ROS_INFO("Using command %s", gstreamer.get_command().c_str());

	cv::VideoCapture cap(gstreamer.get_command());

	if (!cap.isOpened()) {
		ROS_FATAL("Could not open camera");
		ros::shutdown();
		return 1;
	}

	if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
		ROS_FATAL("No Cuda Devices compatible with OpenCV were found");
		ros::shutdown();
		return 1;
	} else {
		ROS_INFO("Cuda Devices compatible with OpenCV were found, %d devices", cv::cuda::getCudaEnabledDeviceCount());
	}

	cv::VideoWriter video_writer;
	cv::Size frame_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);

	if (record_video_flag) {
		int codec = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
		video_writer.open(output_filename, codec, FRAME_RATE, frame_size, true);

		if (!video_writer.isOpened()) {
			ROS_ERROR("Could not open the output video file for write: %s", output_filename.c_str());
			ROS_ERROR("Check if the required codec (e.g., FFmpeg backend with MP4V support) is installed.");
			ROS_WARN("Disabling recording for this run due to error. Running anyways");
			record_video_flag = false;
		} else {
			ROS_INFO("Successfully opened video writer for file: %s", output_filename.c_str());
		}
	}

	BlobDetection ballDetection;
	ballDetection.set_color_range(cv::Scalar(0, 191, 156), cv::Scalar(43, 255, 199));
	ballDetection.set_area(50, 100000);

	unsigned long long ms_count = 0;
	float ball_angle = 0;
	float ball_distance = 0;
	for (unsigned long frame_id = 1;ros::ok();frame_id++) {
		long long startTimestamp = getMillis();
		cv::Mat frame_cpu;
		cap >> frame_cpu;
		if (frame_cpu.empty()) continue;

		preprocessing::resize(frame_cpu, DISPLAY_WIDTH, DISPLAY_HEIGHT);
		preprocessing::saturation(frame_cpu, 2);
		drawIgnoreArea(frame_cpu, IGNORE_BEFORE_Y);

		std::vector<BlobDetection::Blob> ballBlobs = ballDetection.detect(frame_cpu);
		BlobDetection::plot_blobs(frame_cpu, ballBlobs, cv::Scalar(255, 255, 255));

		auto ball = BlobDetection::biggest_blob(ballBlobs);
		
		if (!ball) {
			ball_angle = INVALID_VALUE;
			ball_distance = INVALID_VALUE;
		} else {
			const float ball_cx = ball.value().x + ball.value().width / 2;
			const float ball_cy = ball.value().y + ball.value().height / 2;

			const float view_ball_cx = ball_cx - DISPLAY_WIDTH / 2;
			const float view_ball_cy = DISPLAY_HEIGHT - ball_cy;

			ball_angle = atan(view_ball_cy / view_ball_cx) * 180 / PI;
			ball_distance = std::sqrt(std::pow(view_ball_cx, 2) + std::pow(view_ball_cy, 2));
			ball_distance = get_distance_cm(ball_distance);

			if (ball_angle > 0) {
				ball_angle = 90.0 - ball_angle;
			} else {
				ball_angle = -(90.0 + ball_angle);
			}
		}

		std_msgs::Float32MultiArray msg;
		msg.data.clear();

		msg.data.push_back(ball_angle);
		msg.data.push_back(ball_distance);

		pub.publish(msg);

		#ifdef RECORD_VIDEO
			if (record_video_flag && video_writer.isOpened()) {
				video_writer.write(frame_cpu);
			}		
		#endif

		#ifdef SHOW_IMAGE
			const std::string window_name = "Front-Camera";
			cv::namedWindow(window_name);
			cv::setMouseCallback(window_name, on_mouse, nullptr);
			cv::imshow(window_name, frame_cpu);
			if (cv::waitKey(1) == KEY_ESC) {
				break;
			}
		#endif

		ros::spinOnce();
		long long endTimestamp = getMillis();
		ms_count += endTimestamp - startTimestamp;
		ROS_INFO("Frame processed in %d milliseconds. Avg: %f.\nBall\nangle: %f degrees\ndistance: %f cm\nPublished: [angle=%f, distance=%f]",
			static_cast<int>(endTimestamp - startTimestamp), ms_count * 1.0 / (frame_id) * 1.0,
			ball_angle, ball_distance,
			msg.data[0], msg.data[1]);
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

