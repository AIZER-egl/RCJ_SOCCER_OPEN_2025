#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"
#include "ros/duration.h"
#include <vector>
#include <optional>
#include <thread>
#include <chrono>

#include "binarySerializationData.h"
#include "serializer.h"

ros::Publisher pub_serial_tx;
BinarySerializationData data;

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

    ROS_INFO("Deserialized Data: Yaw=%d, Kicker=%d, LDR=%d",
        received_data.compass_yaw, received_data.kicker_active, received_data.ldr_value);

    data.compass_yaw = received_data_opt -> compass_yaw;

    data.ldr_value = received_data_opt -> ldr_value;

    ROS_INFO("Local data pack values: Yaw=%d, Kicker=%d, LDR=%d",
        data.compass_yaw, data.kicker_active, data.ldr_value);

    std::vector<int8_t> bytes_to_send = Serializer::serialize(data);

    if (!bytes_to_send.empty()) {
        std_msgs::ByteMultiArray tx_msg;
        tx_msg.data = bytes_to_send;
        tx_msg.data.push_back('\n');
        pub_serial_tx.publish(tx_msg);
        ROS_INFO("Published %zu modified bytes to /pico/serial_tx", bytes_to_send.size());
    } else {
        ROS_ERROR("Serialization of modified data failed (empty byte vector)!");
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
        if (millis() - previous_time >= 10000) { // 10 segundos
            ROS_INFO("Updating BinarySerializationData variable to activate kicker");
            data.kicker_active = true;
            previous_time = millis();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Ready to process incoming messages.");

    return 0;
}
