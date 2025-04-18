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

void serialRxCallback(const std_msgs::ByteMultiArray::ConstPtr& msg) {
    ROS_INFO("Received %zu bytes on /pico/serial_rx", msg->data.size());

    std::optional<BinarySerializationData> received_data_opt = Serializer::deserialize(msg -> data);

    if (!received_data_opt) {
        ROS_WARN("Deserialization failed (likely incorrect packet size: %zu bytes, expected %zu)",
                 msg->data.size(), BINARY_SERIALIZATION_DATA_SIZE);
        return;
    }

    BinarySerializationData received_data = *received_data_opt;
    ROS_INFO("Deserialized Data: Yaw=%d, Kicker=%d, LDR=%d",
         received_data.compass_yaw, received_data.kicker_active, received_data.ldr_value);

    RobotData data_to_send = received_data;

    data_to_send.motor_se_speed = 0;
    data_to_send.motor_sw_speed = 0;
    data_to_send.motor_ne_speed = 0;
    data_to_send.motor_nw_speed = 0;

    ROS_INFO("Modified Data: Setting motor speeds to 0 for TX.");

    std::vector<uint8_t> bytes_to_send = Serializer::serialize(data_to_send);

    if (!bytes_to_send.empty()) {
        std_msgs::ByteMultiArray tx_msg;
        tx_msg.data = bytes_to_send;
        pub_serial_tx.publish(tx_msg);
        ROS_INFO("Published %zu modified bytes to /pico/serial_tx", bytes_to_send.size());
    } else {
        ROS_ERROR("Serialization of modified data failed (empty byte vector)!");
    }

}

int main (int argc, char **argv) {
    ros::init(argc, argv, "pico_data_processor");
    ros::NodeHandle nh;

    ROS_INFO("Listening for messages");

    pub_serial_tx = nh.advertise<std_msgs::ByteMultiArray>("/pico/serial_tx", 10);
    ros::Subscriber sub_serial_rx = nh.subscribe("/pico/serial_rx", 10, serialRxCallback);

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second
    ROS_INFO("Sending initial 'empty' message to Pico...");
    BinarySerializationData init_message = {};
    std::vector<uint8_t> init_bytes = Serializer::serialize(init_message);

    if (!init_bytes.empty()) {
        std_msgs::ByteMultiArray init_tx_msg;
        init_tx_msg.data = init_bytes;
        pub_serial_tx.publish(init_tx_msg);
        ROS_INFO("Published initial message (%zu bytes) to /pico/serial_tx", init_bytes.size());
    } else {
        ROS_ERROR("Serialization of initial empty message failed!");
    }

    ROS_INFO("Ready to process incoming messages.");

    ros::spin();

    return 0;
}
