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
#include <algorithm>

#include "binarySerializationData.h"
#include "serializer.h"

#define INVALID_VALUE 999.0
#define CONTROL_LOOP_FREQUENCY 30.0
#define INIT_MESSAGE_FREQUENCY 0.2

ros::Publisher pub_serial_tx;
BinarySerializationData data;
float omni_angle = INVALID_VALUE;
float front_angle = INVALID_VALUE;
float front_distance = INVALID_VALUE;
bool first_front_camera_detection = true;

std::mutex g_data_mutex;

bool g_new_pico_data_received = false;
std::atomic<bool> g_communication_has_started(false);

ros::Timer control_timer;
ros::Timer init_timer;

unsigned long long millis() {
	auto currentTimePoint = std::chrono::steady_clock::now();
	auto durationSinceEpoch = currentTimePoint.time_since_epoch();
	auto millisSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(durationSinceEpoch);
	return static_cast<unsigned long long>(millisSinceEpoch.count());
}


void serialRxCallback(const std_msgs::ByteMultiArray::ConstPtr& msg) {
    std::optional<BinarySerializationData> received_data_opt = Serializer::deserialize(msg->data);

	if (!received_data_opt) {
		if (g_communication_has_started.load()) {
			ROS_WARN("Deserialization failed (likely incorrect packet size: %zu bytes, expected %zu)",
				msg->data.size(), BINARY_SERIALIZATION_DATA_SIZE);
		}
		return;
	}

	if (!g_communication_has_started.load()) {
		g_communication_has_started.store(true);
		ROS_INFO("First valid message received from Pico! Communication established.");
	}

	{
		std::lock_guard<std::mutex> lock(g_data_mutex);
		data.compass_yaw = received_data_opt->compass_yaw;
		data.ldr_value = received_data_opt->ldr_value;
		g_new_pico_data_received = true;
	}
}

void omnicamera_callback (const std_msgs::Float32MultiArray::ConstPtr& msg) {
	std::lock_guard<std::mutex> lock(g_data_mutex);
	if (!msg->data.empty()) {
		omni_angle = msg->data[0];
	} else {
		omni_angle = INVALID_VALUE;
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
	BinarySerializationData data_to_send;
	float current_front_angle;
	float current_front_distance;
	float current_omni_angle;
	bool process_this_cycle = false;

	{
		std::lock_guard<std::mutex> lock(g_data_mutex);
		if (!g_new_pico_data_received) return;

		process_this_cycle = true;

		data_to_send = data;
		current_front_angle = front_angle;
		current_front_distance = front_distance;
		current_omni_angle = omni_angle;

		bool kick_requested = false;
		if (front_distance != INVALID_VALUE && front_distance < 2.5) {
			kick_requested = true;
		}

		if (kick_requested) {
			data.kicker_active = true;
		} else {
			data.kicker_active = false;
		}

		if (current_front_angle == INVALID_VALUE) first_front_camera_detection = true;

		if (current_front_angle != INVALID_VALUE) {
			if (first_front_camera_detection) {
				first_front_camera_detection = false;
				data_to_send.robot_stop = true;
				data_to_send.robot_direction = 0;
				data_to_send.robot_speed = 0;
				data_to_send.robot_facing = 0;
			} else {
				float speed_offset = (current_front_distance > 0.1) ? std::max(-2.0/3.0 * current_front_distance + 20.0, 0.0) : 20.0;

				data_to_send.robot_stop = false;
				data_to_send.robot_direction = static_cast<int16_t>(current_front_angle * 1.3);
				data_to_send.robot_speed = std::max(0, 40 - static_cast<int>(speed_offset));
				data_to_send.robot_facing = 0;
			}
		} else if (current_omni_angle != INVALID_VALUE) {
			int speed_offset = static_cast<int>(-1.0/18.0 * std::abs(current_omni_angle) + 15.0);
			speed_offset = std::max(0, std::min(15, speed_offset));

			data_to_send.robot_stop = false;
			data_to_send.robot_direction = static_cast<int16_t>(omni_angle + (60.0 * signOf(static_cast<float>(data.robot_direction))));
			data_to_send.robot_speed = 45 - speed_offset;
			data_to_send.robot_facing = 0;
		} else {
			data_to_send.robot_stop = false;
			data_to_send.robot_direction = 0;
			data_to_send.robot_speed = 0;
			data_to_send.robot_facing = 0;
		}

		g_new_pico_data_received = false;
		data = data_to_send;
	}


	std::vector<int8_t> bytes_to_send = Serializer::serialize(data_to_send);

	if (!bytes_to_send.empty()) {
		std_msgs::ByteMultiArray tx_msg;
		tx_msg.data = bytes_to_send;
		tx_msg.data.push_back('\n');

		pub_serial_tx.publish(tx_msg);
		ROS_INFO("Ctrl: (Dir=%d, Speed=%d, Stop=%d, Face=%d, Kick=%d) | Sens: (Yaw: %d, LDR: %d) | Cam: [f_a: %.1f, f_d: %.1f - o_a: %.1f]",
			data_to_send.robot_direction, data_to_send.robot_speed, data_to_send.robot_stop, data_to_send.robot_facing, data_to_send.kicker_active,
			data_to_send.compass_yaw, data_to_send.ldr_value,
			current_front_angle, current_front_distance, current_omni_angle
		);
	} else {
		ROS_ERROR("Serialization of control data failed (empty byte vector)!");
	}
}


void sendInitMessageCallback(const ros::TimerEvent& event) {
    if (g_communication_has_started.load()) {
        ROS_INFO("Communication detected, stopping init timer and starting control timer.");
        init_timer.stop();

        if (!control_timer) {
            ros::NodeHandle nh;
             control_timer = nh.createTimer(ros::Duration(1.0 / CONTROL_LOOP_FREQUENCY), controlLoopCallback);
             ROS_INFO("Control timer started.");
        } else {
            control_timer.start();
             ROS_INFO("Control timer (re)started.");
        }

        return;
    }

    ROS_INFO("Sending empty init message to Pico (waiting for response)...");
    BinarySerializationData init_message = {};
    std::vector<int8_t> init_bytes = Serializer::serialize(init_message);

    if (!init_bytes.empty()) {
        std_msgs::ByteMultiArray init_tx_msg;
        init_tx_msg.data = init_bytes;
        init_tx_msg.data.push_back('\n');

        pub_serial_tx.publish(init_tx_msg);
    } else {
        ROS_ERROR("Serialization of initial empty message failed!");
    }
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
		omni_angle = INVALID_VALUE;
		front_angle = INVALID_VALUE;
		front_distance = INVALID_VALUE;
		first_front_camera_detection = true;
	}

	ROS_INFO("Waiting for communication with Pico...");

	init_timer = nh.createTimer(ros::Duration(1.0 / INIT_MESSAGE_FREQUENCY), sendInitMessageCallback);


	int num_spinner_threads = 4;
	ros::AsyncSpinner spinner(num_spinner_threads);
	spinner.start();
	ROS_INFO("AsyncSpinner started with %d threads. Ready to process messages.", num_spinner_threads);

	ros::waitForShutdown();

	ROS_INFO("Shutting down pico_data_processor node.");

	return 0;
}
