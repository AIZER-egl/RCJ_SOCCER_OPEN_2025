#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/duration.h"
#include <vector>
#include <optional>
#include <thread>
#include <chrono>

#include "binarySerializationData.h"
#include "serializer.h"

#define INVALID_VALUE 999

ros::Publisher pub_serial_tx;
BinarySerializationData data;
float omni_angle = INVALID_VALUE;
float front_angle = INVALID_VALUE;
float front_distance = INVALID_VALUE;

void serialRxCallback(const std_msgs::ByteMultiArray::ConstPtr& msg) {
    ROS_INFO("Received %zu bytes on /pico/serial_rx", msg->data.size());

    std::optional<BinarySerializationData> received_data_opt = Serializer::deserialize(msg -> data);

    if (!received_data_opt) {
        ROS_WARN("Deserialization failed (likely incorrect packet size: %zu bytes, expected %zu)",
                 msg->data.size(), BINARY_SERIALIZATION_DATA_SIZE);
        return;
    }

    // Values that are being updated are handled by the PICO
    //    -> They include information of hardware is doing
    // Values not updated here are handled by the jetson
    //    -> They include information about what the hardware should do
    BinarySerializationData received_data = *received_data_opt;

    ROS_INFO("Deserialized Data: Robot Dir=%d, Robot Speed=%d Yaw=%d, Kicker=%d",
        received_data.robot_direction, received_data.robot_speed, received_data.compass_yaw, received_data.kicker_active);

    data.compass_yaw = received_data_opt -> compass_yaw;

    ROS_INFO("Local data pack values: Robot Dir=%d, Robot Speed=%d Yaw=%d, Kicker=%d",
        data.robot_direction, data.robot_speed, data.compass_yaw, data.kicker_active);

    std::vector<int8_t> bytes_to_send = Serializer::serialize(data);

    if (!bytes_to_send.empty()) {
        std_msgs::ByteMultiArray tx_msg;
        tx_msg.data = bytes_to_send;
        tx_msg.data.push_back('\n');
        pub_serial_tx.publish(tx_msg);
        ROS_INFO("Published %zu modified bytes to /pico/serial_tx", bytes_to_send.size());

        // Set kicker to false after message has been published
        data.kicker_active = false;
    } else {
        ROS_ERROR("Serialization of modified data failed (empty byte vector)!");
    }
}

void omnicamera_callback (const std_msgs::Float32MultiArray::ConstPtr& msg) {
	ROS_INFO("Received omnicamera data (size: %zu)", msg -> data.size());

	if (msg -> data.size() >= 1) {
		omni_angle = msg -> data[0];
		ROS_INFO("OmniCam Data: Angle=%.2f", omni_angle);
	} else {
		ROS_WARN("Received insufficient data on omnicamera_topic (size %zu, expected >= 1)", msg -> data.size());
	}
}

void frontcamera_callback (const std_msgs::Float32MultiArray::ConstPtr& msg) {
	ROS_INFO("Received frontcamera data (size: %zu)", msg -> data.size());

	if (msg -> data.size() >= 2) {
		front_angle = msg -> data[0];
		front_distance = msg -> data[1];

		ROS_INFO("FrontCam Data: Angle=%.2f, Distance=%.2f", front_angle, front_distance);
	} else {
		ROS_WARN("Received insufficient data on frontcamera_topic (size %zu, expected >= 2)", msg -> data.size());
	}
}

unsigned long long millis() {
    auto currentTimePoint = std::chrono::steady_clock::now();
    auto durationSinceEpoch = currentTimePoint.time_since_epoch();
    auto millisSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(durationSinceEpoch);
    return static_cast<unsigned long long>(millisSinceEpoch.count());
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "pico_data_processor");
    ros::NodeHandle nh;

    ROS_INFO("Listening for messages");

    pub_serial_tx = nh.advertise<std_msgs::ByteMultiArray>("/pico/serial_tx", 10);
    ros::Subscriber sub_serial_rx = nh.subscribe("/pico/serial_rx", 10, serialRxCallback);
    ros::Subscriber sub_frontcamera_topic = nh.subscribe("/omnicamera_topic", 10, omnicamera_callback);
    ros::Subscriber sub_omnicamera_topic = nh.subscribe("/frontcamera_topic", 10, frontcamera_callback);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_INFO("Sending initial 'empty' message to Pico...");
    BinarySerializationData init_message = {};
    std::vector<int8_t> init_bytes = Serializer::serialize(init_message);

    if (!init_bytes.empty()) {
        std_msgs::ByteMultiArray init_tx_msg;
        init_tx_msg.data = init_bytes;
        init_tx_msg.data.push_back('\n');
        pub_serial_tx.publish(init_tx_msg);
        ROS_INFO("Published initial message (%zu bytes) to /pico/serial_tx", init_bytes.size());
    } else {
        ROS_ERROR("Serialization of initial empty message failed!");
    }

    unsigned long long previous_time = millis();
    ros::Rate loop_rate(15);

    ROS_INFO("Ready to process incoming messages."); // Mover esto antes del bucle

    while (ros::ok()) {
        if (millis() - previous_time >= 10000) {
            ROS_INFO("Updating BinarySerializationData variable to activate kicker");
            data.kicker_active = true;
            previous_time = millis();
        }

	if (front_angle != INVALID_VALUE) {
        	data.robot_stop = false;
        	data.robot_direction = front_angle;
        	data.robot_speed = 30;
        	data.robot_facing = 0;
        } else {
		data.robot_direction = 0;
		data.robot_speed = 0;
		data.robot_facing = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Ready to process incoming messages.");

    return 0;
}
