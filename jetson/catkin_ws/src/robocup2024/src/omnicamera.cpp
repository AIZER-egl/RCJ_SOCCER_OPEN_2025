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

#define SHOW_IMAGE
//#define RECORD_VIDEO

#define DISPLAY_HEIGHT 473
#define DISPLAY_WIDTH 768

#define ROBOT_ORIGIN_X 368
#define ROBOT_ORIGIN_Y 275

#define INVALID_VALUE 999
#define FRAME_RATE 20

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

int get_quadrant (float x, float y) {
	if (x > 0 && y > 0) return 1;
	if (x < 0 && y > 0) return 2;
	if (x < 0 && y < 0) return 3;
	return 4;
}

float get_angle (float x, float y) {
	int quadrant = get_quadrant(x, y);
	float angle = 0;

	switch (quadrant) {
		case 1:	angle = std::abs(atan(y / x) * 180 / PI); break;
		case 2:	angle = 180 - std::abs(atan(y / x) * 180 / PI); break;
		case 3:	angle = 180 + std::abs(atan(y / x) * 180 / PI); break;
		case 4:	angle = 360 - std::abs(atan(y / x) * 180 / PI); break;
	}

	angle -= 90;
	if (angle < 0) angle += 360;

	if (angle > 180) angle -= 360;
	return angle;
}

void applyCircularMask(const cv::Mat& frame, int center_x, int center_y, cv::Mat& output_frame, bool modify_inplace = false) {
	if (center_x < 0 || center_x >= frame.cols || center_y < 0 || center_y >= frame.rows) {
		std::cerr << "Error: El centro del círculo está fuera de los límites de la imagen." << std::endl;
		if (!modify_inplace) {
			frame.copyTo(output_frame);
		}
		return;
	}
	int dist_left = center_x;
	int dist_right = frame.cols - 1 - center_x;
	int dist_top = center_y;
	int dist_bottom = frame.rows - 1 - center_y;
	int radius = std::min({dist_left, dist_right, dist_top, dist_bottom});
	radius = std::max(0, radius);
	cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
	cv::circle(mask, cv::Point(center_x, center_y), radius, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
	if (modify_inplace) {
		cv::Mat masked_image;
		frame.copyTo(masked_image, mask);
		output_frame = masked_image;
	} else {
		output_frame = cv::Mat::zeros(frame.size(), frame.type());
		frame.copyTo(output_frame, mask);
	}
}

int main (int argc, char **argv) {

	ros::init(argc, argv, "omnicamera");

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("omnicamera_topic", 10);

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
		output_filename = "/home/aizer/Documents/RCJ_SOCCER_OPEN_2025/jetson/catkin_ws/src/robocup2024/out/omnicamera_out_";
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

	BlobDetection ballDetection;
	ballDetection.set_color_range(cv::Scalar(0, 200, 120), cv::Scalar(171, 255, 219));
	ballDetection.set_area(7, 100000);

	float ball_angle = 0;
	unsigned long long ms_count = 0;
	for (unsigned long frame_id = 1;ros::ok();frame_id++) {
		long long startTimestamp = getMillis();
		cv::Mat frame_cpu;
		cap >> frame_cpu;
		if (frame_cpu.empty()) continue;

		preprocessing::resize(frame_cpu, DISPLAY_WIDTH, DISPLAY_HEIGHT);
		preprocessing::contrast(frame_cpu, 1.4, 2.5);
		preprocessing::gamma_correction(frame_cpu, 1.9);
		preprocessing::brightness(frame_cpu, 1.4);
		preprocessing::saturation(frame_cpu, 2);
		cv::flip(frame_cpu, frame_cpu, 0);

		applyCircularMask(frame_cpu, ROBOT_ORIGIN_X, ROBOT_ORIGIN_Y, frame_cpu, true);

		std::vector<BlobDetection::Blob> ballBlobs = ballDetection.detect(frame_cpu);
		BlobDetection::plot_blobs(frame_cpu, ballBlobs, cv::Scalar(255, 255, 255));

		auto ball = BlobDetection::biggest_blob(ballBlobs);

		if (!ball) {
			ball_angle = INVALID_VALUE;
		} else {
			const float ball_cx = ball.value().x + ball.value().width / 2;
			const float ball_cy = ball.value().y + ball.value().height / 2;
			const float view_ball_cx = ball_cx - ROBOT_ORIGIN_X;
			const float view_ball_cy = ball_cy - ROBOT_ORIGIN_Y;

			ball_angle = get_angle(view_ball_cx, view_ball_cy);
		}

		std_msgs::Float32MultiArray msg;
		msg.data.clear();

		msg.data.push_back(ball_angle);

		pub.publish(msg);

		#ifdef RECORD_VIDEO
			if (record_video_flag && video_writer.isOpened()) {
				video_writer.write(frame_cpu);
			}
		#endif

		#ifdef SHOW_IMAGE
			const std::string window_name = "Omni-Camera";
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
		ROS_INFO("Frame processed in %d milliseconds. Avg: %f.\nBall\nangle: %f degrees", static_cast<int>(endTimestamp - startTimestamp), ms_count * 1.0 / (frame_id) * 1.0, ball_angle);
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

