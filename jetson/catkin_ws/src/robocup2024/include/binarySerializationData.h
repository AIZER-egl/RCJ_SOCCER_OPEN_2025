#ifndef BINARYSERIALIZATIONDATA_H
#define BINARYSERIALIZATIONDATA_H

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)

struct BinarySerializationData {
	int16_t compass_yaw;

	float motor_se_rps;
	float motor_sw_rps;
	float motor_ne_rps;
	float motor_nw_rps;

	int16_t robot_direction;
	int16_t robot_speed;
	int16_t robot_facing;
	bool robot_stop;

	bool kicker_active;

	uint8_t ldr_0_value;
	uint8_t ldr_4_value;
};

#pragma pack(pop)

const size_t BINARY_SERIALIZATION_DATA_SIZE = sizeof(BinarySerializationData);

#endif //BINARYSERIALIZATIONDATA_H
