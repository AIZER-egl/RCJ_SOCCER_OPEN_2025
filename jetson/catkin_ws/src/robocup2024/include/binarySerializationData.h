#ifndef BINARYSERIALIZATIONDATA_H
#define BINARYSERIALIZATIONDATA_H

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)

struct BinarySerializationData {
	uint8_t motor_se_speed;
	uint8_t motor_sw_speed;
	uint8_t motor_ne_speed;
	uint8_t motor_nw_speed;

	float motor_se_rpm;
	float motor_sw_rpm;
	float motor_ne_rpm;
	float motor_nw_rpm;

	int8_t motor_se_direction;
	int8_t motor_sw_direction;
	int8_t motor_ne_direction;
	int8_t motor_nw_direction;

	int16_t compass_yaw;

	int16_t robot_direction;
	int16_t robot_speed;
	int16_t robot_facing;
	bool robot_stop;

	bool kicker_active;

	uint8_t ldr_value;

	uint8_t setting_team_id;
	uint8_t setting_attack_goal;
};

#pragma pack(pop)

const size_t BINARY_SERIALIZATION_DATA_SIZE = sizeof(BinarySerializationData);

#endif //BINARYSERIALIZATIONDATA_H