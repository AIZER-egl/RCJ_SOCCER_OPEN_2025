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

	uint8_t motor_se_rpm;
	uint8_t motor_sw_rpm;
	uint8_t motor_ne_rpm;
	uint8_t motor_nw_rpm;

	uint8_t motor_se_direction;
	uint8_t motor_sw_direction;
	uint8_t motor_ne_direction;
	uint8_t motor_nw_direction;

	int16_t compass_yaw;

	bool kicker_active;

	uint8_t ldr_value;

	uint8_t setting_team_id;
	uint8_t setting_attack_goal;
};

#pragma pack(pop)

const size_t BINARY_SERIALIZATION_DATA_SIZE = sizeof(BinarySerializationData);

#endif //BINARYSERIALIZATIONDATA_H
