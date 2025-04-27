#ifndef BINARYSERIALIZATIONDATA_H
#define BINARYSERIALIZATIONDATA_H

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)

struct BinarySerializationData {
	int16_t compass_yaw;

	int16_t robot_direction;
	int16_t robot_speed;
	int16_t robot_facing;
	bool robot_stop;

	bool kicker_active;

	uint16_t ldr_0_value;
	uint16_t ldr_1_value;
	uint16_t ldr_2_value;
	uint16_t ldr_3_value;
	uint16_t ldr_4_value;
	uint16_t ldr_5_value;
	uint16_t ldr_6_value;
	uint16_t ldr_7_value;

	uint8_t setting_team_id;
	uint8_t setting_attack_goal;
};

#pragma pack(pop)

const size_t BINARY_SERIALIZATION_DATA_SIZE = sizeof(BinarySerializationData);

#endif //BINARYSERIALIZATIONDATA_H
