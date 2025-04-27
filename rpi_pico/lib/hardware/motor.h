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
#define MOTOR_SW_ENC_A 10
#define MOTOR_SW_ENC_B 11

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
#define MIN_SPEED 25

#define MAX_RPS 80
#define MIN_RPS 0

#define PI 3.141592

class Motor {
public:
	Motor();

	struct individualMotor {
		uint8_t id{};
		int8_t direction{};
		uint16_t speed{};
		double rps{};

		unsigned long previousPulsesA{};
		unsigned long previousPulsesB{};

		unsigned long previousTimeA{};
		unsigned long previousTimeB{};

		static void callback (unsigned int gpio, unsigned long events);

		void getRPS ();

		PID::PidParameters rpsPID;
		int delay_ms = 20;
		unsigned long lastIteration_ms = 0;
		int previousOut{};
	};

	individualMotor motorSE;
	individualMotor motorSW;
	individualMotor motorNE;
	individualMotor motorNW;

	PID::PidParameters rotationPID;

	void setSpeedSE (int16_t speed);
	void setSpeedSW (int16_t speed);
	void setSpeedNE (int16_t speed);
	void setSpeedNW (int16_t speed);

	void moveSE (int16_t rps);
	void moveSW (int16_t rps);
	void moveNE (int16_t rps);
	void moveNW (int16_t rps);

	void begin (BinarySerializationData& data);

	void stop ();

	void tick ();

	void move (float rps, float direction, float facing_target, float facing_current);

	void rotate (int16_t angle);
private:
	BinarySerializationData* dataPtr;
	unsigned long long previousTick{};
};

#endif //PICO_LIB_MOTOR_H
