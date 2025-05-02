#ifndef PICO_LIB_MOTOR_H
#define PICO_LIB_MOTOR_H

#include <algorithm>
#include <cstdint>

#include "../software/pid.h"
#include "../../pico-lib/time.h"
#include "../../pico-lib/usb.h"
#include "../../pico-lib/gpio.h"
#include "../software/binarySerializationData.h"

#define MOTOR_NE_DIR 12
#define MOTOR_NE_PWM 13
#define MOTOR_NE_ENC_A 6
#define MOTOR_NE_ENC_B 7

#define MOTOR_SE_DIR 8
#define MOTOR_SE_PWM 9
#define MOTOR_SE_ENC_A 21
#define MOTOR_SE_ENC_B 20

#define MOTOR_SW_DIR 2
#define MOTOR_SW_PWM 3
#define MOTOR_SW_ENC_A 11 // Encoder A and Encoder B are switched because Robot 1 Encoder B is failing
#define MOTOR_SW_ENC_B 10

#define MOTOR_NW_DIR 26
#define MOTOR_NW_PWM 22
#define MOTOR_NW_ENC_A 14
#define MOTOR_NW_ENC_B 15

#define MOTOR_SE 0b0000
#define MOTOR_SW 0b0001
#define MOTOR_NE 0b0010
#define MOTOR_NW 0b0011

#define TICKS_PER_REVOLUTION_ENCODER 12
#define GEAR_RATIO 9.68

#define MAX_SPEED 255
#define MIN_SPEED 15

#define MAX_RPS 18
#define MIN_RPS 0

#define PI 3.141592

class Motor {
public:
	Motor();

	struct individualMotor {
		std::string name;
		uint8_t id{};
		int8_t direction{};
		uint16_t speed{};
		double rps{};
		double previous_rps{};

		void setSpeed(int16_t new_speed, bool save_direction = true);
		void move(double new_rps);

		static void callback (unsigned int gpio, unsigned long events);

		void getRPS ();
		double getRPS_average ();

		PID::PidParameters rpsPID;

		int pwm_pin{};
		int dir_pin{};

		double rps_summatory = 0;
		unsigned long long rps_count = 0;
		unsigned long long lastIteration_ms = 0;
		double previousOut{};
	};

	individualMotor motorSE;
	individualMotor motorSW;
	individualMotor motorNE;
	individualMotor motorNW;

	PID::PidParameters rotationPID;

	void moveNW (int16_t rps);

	void begin (BinarySerializationData& data);

	void stop ();

	void tick ();

	void move (float rps, float direction, float facing_target, float facing_current, bool save_direction = true);

	void rotate (int16_t angle);

	float current_direction{};
private:
	bool robot_stopped = false;
	BinarySerializationData* dataPtr;
	unsigned long long previousTick{};
};

#endif //PICO_LIB_MOTOR_H
