#ifndef BINARYSERIALIZATIONDATA_H
#define BINARYSERIALIZATIONDATA_H

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)

struct BinarySerializationData {
	int16_t compass_yaw;

	/*
		In order to save space bytes, robot_speed is expressed in uin16_t (2 bytes)
		Even though the robot accepts float numbers (such as 11.5)
		Because two decimal precision is not needed and the normal range for robot_speed is (5 to 17 RPS)
		I will receive values in this format
		RPS: 12   -> robot_speed: 120
		RPS: 11.5 -> robot_speed 115

		The reason for this logic instead of a floating point number, is that the raspberry pico and the ROS
		python node are not capable of processing a lot of bytes per second.
	 */
	uint16_t robot_speed;
	int16_t robot_direction;
	int16_t robot_facing;
	bool robot_stop;

	/**
	 * In order to reduce bytes, I will be using uint8_t (1 byte)
	 * The robot has 8 LDR sensors, (0 - 3 in one side), (4 - 7 in the other side)
	 * The ldr_channel has a value from 0 to 7 and contains which LDR the ldr_value is from.
	 * For example:
	 *     If reading LDR 0, the ldr_channel must be 0 and the ldr_value is the real value for LDR 0
	 *     If reading LDR 4, the ldr_channel must be 4 and the ldr_value is the real value for LDR 4
	 * The order cannot be changed on demand by the Jetson, ldr_channel is updated by the pico every time
	 * the message is sent.
	 * The reason to not make ldr_channel an on demmand is because coordination might break the communication
	 * and since time is key in this time of the development, I will not add this feature since it is only for
	 * calibration purposes.
	 */
	// uint8_t ldr_channel;
	uint16_t ldr_value;

	bool kicker_active;
};

#pragma pack(pop)

const size_t BINARY_SERIALIZATION_DATA_SIZE = sizeof(BinarySerializationData);

#endif //BINARYSERIALIZATIONDATA_H
