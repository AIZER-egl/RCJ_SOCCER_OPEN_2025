#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/duration.h>
#include <ros/timer.h>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

#include "binarySerializationData.h"
#include "serializer.h"

#define INVALID_VALUE 999.0
#define CONTROL_LOOP_FREQUENCY 30.0

ros::Publisher pub_serial_tx;
BinarySerializationData data;
float omni_angle = INVALID_VALUE;
float front_angle = INVALID_VALUE;
float front_distance = INVALID_VALUE;
bool first_front_camera_detection = true;

std::mutex g_data_mutex;

bool g_new_pico_data_received = false;

unsigned long long millis() {
	auto currentTimePoint = std::chrono::steady_clock::now();
	auto durationSinceEpoch = currentTimePoint.time_since_epoch();
	auto millisSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(durationSinceEpoch);
	return static_cast<unsigned long long>(millisSinceEpoch.count());
}


void serialRxCallback(const std_msgs::ByteMultiArray::ConstPtr& msg) {
	std::optional<BinarySerializationData> received_data_opt = Serializer::deserialize(msg->data);

	if (!received_data_opt) {
		ROS_WARN("Deserialization failed (likely incorrect packet size: %zu bytes, expected %zu)",
			msg->data.size(), BINARY_SERIALIZATION_DATA_SIZE);
		return;
	}

	std::lock_guard<std::mutex> lock(g_data_mutex);

	data.compass_yaw = received_data_opt->compass_yaw;
	data.ldr_value = received_data_opt->ldr_value;

	g_new_pico_data_received = true;
}

void omnicamera_callback (const std_msgs::Float32MultiArray::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock(g_data_mutex);
	if (!msg->data.empty()) {
		omni_angle = msg->data[0];
	} else {
		ROS_WARN("Received empty data on /omnicamera_topic");
	}
}

void frontcamera_callback (const std_msgs::Float32MultiArray::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock(g_data_mutex);
	if (msg->data.size() >= 2) {
		front_angle = msg->data[0];
		front_distance = msg->data[1];
	} else {
		ROS_WARN("Received incomplete data (size %zu, expected 2) on /frontcamera_topic", msg->data.size());
		front_angle = INVALID_VALUE;
		front_distance = INVALID_VALUE;
	}
}

int signOf (float value) {
	if (value > 0.0f) return 1;
	if (value < 0.0f) return -1;
	return 0;
}

void controlLoopCallback(const ros::TimerEvent& event) {

	std::lock_guard<std::mutex> lock(g_data_mutex);

	if (!g_new_pico_data_received) return;

	unsigned long long current_time = millis();

	bool kick_requested = false;
	if (front_distance != INVALID_VALUE && front_distance < 2.5) {
		kick_requested = true;
	}

	if (kick_requested) {
		data.kicker_active = true;
		ROS_INFO("Kicker activated!");
	} else {
		data.kicker_active = false;
	}


	// If nothing is detected in front camera, leave flag set to true
	if (front_angle == INVALID_VALUE) first_front_camera_detection = true;

	if (front_angle != INVALID_VALUE) {
		if (first_front_camera_detection) {
			data.robot_stop = true;
			first_front_camera_detection = false;
		} else {
			data.robot_stop = false;
			data.robot_direction = static_cast<int16_t>(front_angle * 1.3);
			data.robot_speed = 70;
			data.robot_facing = 0;
		}
	} else if (omni_angle != INVALID_VALUE) {
		data.robot_stop = false;
		data.robot_direction = static_cast<int16_t>(omni_angle + (60.0 * signOf(static_cast<float>(data.robot_direction))));
		data.robot_speed = 60;
		data.robot_facing = 0;
	} else {
		data.robot_stop = false;
		data.robot_direction = 0;
		data.robot_speed = 0;
		data.robot_facing = 0;
	}

	std::vector<int8_t> bytes_to_send = Serializer::serialize(data);

	if (!bytes_to_send.empty()) {
		std_msgs::ByteMultiArray tx_msg;
		tx_msg.data = bytes_to_send;
		tx_msg.data.push_back('\n');

		pub_serial_tx.publish(tx_msg);
		ROS_INFO("(Dir=%d, Speed=%d, Kicker=%d, Yaw: %d, LDR: %d) [f_a: %f, f_d: %f - o_a: %f",
			data.robot_direction, data.robot_speed, data.kicker_active, data.compass_yaw, data.ldr_value, front_angle, front_distance, omni_angle);
	} else {
		ROS_ERROR("Serialization of control data failed (empty byte vector)!");
	}

	g_new_pico_data_received = false;
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "pico_data_processor");
	ros::NodeHandle nh;

	ROS_INFO("Initializing pico_data_processor node...");

	pub_serial_tx = nh.advertise<std_msgs::ByteMultiArray>("/pico/serial_tx", 10);

	ros::Subscriber sub_serial_rx = nh.subscribe("/pico/serial_rx", 10, serialRxCallback);
	ros::Subscriber sub_omnicamera_topic = nh.subscribe("/omnicamera_topic", 10, omnicamera_callback);
	ros::Subscriber sub_frontcamera_topic = nh.subscribe("/frontcamera_topic", 10, frontcamera_callback);


	{
		std::lock_guard<std::mutex> lock(g_data_mutex);
		data = {};
		g_new_pico_data_received = false;
	}

	ros::Duration(1.0).sleep();

	ROS_INFO("Sending initial 'empty' message to Pico...");
	BinarySerializationData init_message = {};
	std::vector<int8_t> init_bytes = Serializer::serialize(init_message);

	if (!init_bytes.empty()) {
   	     std_msgs::ByteMultiArray init_tx_msg;
		init_tx_msg.data = init_bytes;
		init_tx_msg.data.push_back('\n');
		pub_serial_tx.publish(init_tx_msg);
		ROS_INFO("Initial message sent.");
	} else {
		ROS_ERROR("Serialization of initial empty message failed!");
	}

	ros::Timer control_timer = nh.createTimer(ros::Duration(1.0 / CONTROL_LOOP_FREQUENCY), controlLoopCallback);

	int num_spinner_threads = 4;
	ros::AsyncSpinner spinner(num_spinner_threads);
	spinner.start();
	ROS_INFO("AsyncSpinner started with %d threads. Ready to process messages.", num_spinner_threads);

	ros::waitForShutdown();

	ROS_INFO("Shutting down pico_data_processor node.");

	return 0;
}
