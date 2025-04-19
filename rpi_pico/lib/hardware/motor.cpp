#include "motor.h"

volatile unsigned long pulses_SE;
volatile unsigned long pulses_SW;
volatile unsigned long pulses_NE;
volatile unsigned long pulses_NW;

unsigned long long previous_pulses_SE_timestamp;
unsigned long long previous_pulses_SW_timestamp;
unsigned long long previous_pulses_NE_timestamp;
unsigned long long previous_pulses_NW_timestamp;

Motor::Motor() : dataPtr(nullptr) {};

void Motor::begin (BinarySerializationData& data) {
	dataPtr = &data;

	motorNE.id = MOTOR_NE;
	motorNW.id = MOTOR_NW;
	motorSE.id = MOTOR_SE;
	motorSW.id = MOTOR_SW;

	pinMode(MOTOR_NE_DIR, OUTPUT);
	pinMode(MOTOR_NE_PWM, OUTPUT_PWM);
	pinMode(MOTOR_NW_DIR, OUTPUT);
	pinMode(MOTOR_NW_PWM, OUTPUT_PWM);
	pinMode(MOTOR_SE_DIR, OUTPUT);
	pinMode(MOTOR_SE_PWM, OUTPUT_PWM);
	pinMode(MOTOR_SW_DIR, OUTPUT);
	pinMode(MOTOR_SW_PWM, OUTPUT_PWM);

	interrupts(
		std::vector<uint8_t> { MOTOR_NE_ENC_B, MOTOR_NW_ENC_B, MOTOR_SW_ENC_B, MOTOR_SE_ENC_B },
		[] (const unsigned int gpio, const unsigned long events) {
			Motor::individualMotor::callback(gpio, events);
		}
	);

	motorSE.rpmPID.maxOutput = MAX_SPEED;
	motorSE.rpmPID.maxError = 1000.0;
	motorSE.rpmPID.errorThreshold = 3.0;
	motorSE.rpmPID.kp = 0.1;
	motorSE.rpmPID.ki = 0.0001;
	motorSE.rpmPID.kd = 0.0;

	motorSW.rpmPID.maxOutput = MAX_SPEED;
	motorSW.rpmPID.maxError = 1000.0;
	motorSW.rpmPID.errorThreshold = 3.0;
	motorSW.rpmPID.kp = 0.1;
	motorSW.rpmPID.ki = 0.0001;
	motorSW.rpmPID.kd = 0.0;

	motorNW.rpmPID.maxOutput = MAX_SPEED;
	motorNW.rpmPID.maxError = 1000.0;
	motorNW.rpmPID.errorThreshold = 3.0;
	motorNW.rpmPID.kp = 0.1;
	motorNW.rpmPID.ki = 0.0001;
	motorNW.rpmPID.kd = 0.0;

	motorNE.rpmPID.maxOutput = MAX_SPEED;
	motorNE.rpmPID.maxError = 1000.0;
	motorNE.rpmPID.errorThreshold = 3.0;
	motorNE.rpmPID.kp = 0.1;
	motorNE.rpmPID.ki = 0.0001;
	motorNE.rpmPID.kd = 0.0;

	rotationPID.maxOutput = MAX_RPM;
	rotationPID.maxError = 1000.0;
	rotationPID.errorThreshold = 3.0;
	rotationPID.kp = 0.25;
	rotationPID.ki = 0.4;
	rotationPID.kd = 0.16;
}

void Motor::tick() {
	if (!dataPtr) return;

	if (millis() - previousTick >= 20) {
		motorSE.getRPM();
		motorSW.getRPM();
		motorNE.getRPM();
		motorNW.getRPM();

		dataPtr -> motor_se_speed = motorSE.speed;
		dataPtr -> motor_se_direction = motorSE.direction;
		dataPtr -> motor_se_rpm = static_cast<float>(motorSE.rpm);

		dataPtr -> motor_sw_speed = motorSW.speed;
		dataPtr -> motor_sw_direction = motorSW.direction;
		dataPtr -> motor_sw_rpm = static_cast<float>(motorSW.rpm);

		dataPtr -> motor_ne_speed = motorNE.speed;
		dataPtr -> motor_ne_direction = motorNE.direction;
		dataPtr -> motor_ne_rpm = static_cast<float>(motorNE.rpm);

		dataPtr -> motor_nw_speed = motorNW.speed;
		dataPtr -> motor_nw_direction = motorNW.direction;
		dataPtr -> motor_nw_rpm = static_cast<float>(motorNW.rpm);

		previousTick = millis();
	}
}

void Motor::setSpeedSE (int16_t speed) {
	speed = std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

	analogWrite(MOTOR_SE_PWM, std::abs(speed));
	digitalWrite(MOTOR_SE_DIR, speed > 0);
	motorSE.speed = std::abs(speed);
	motorSE.direction = speed > 0 ? 1 : -1;
}

void Motor::setSpeedSW (int16_t speed) {
	speed = std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

	analogWrite(MOTOR_SW_PWM, std::abs(speed));
	digitalWrite(MOTOR_SW_DIR, speed > 0);
	motorSW.speed = std::abs(speed);
	motorSW.direction = speed > 0 ? 1 : -1;
}

void Motor::setSpeedNE (int16_t speed) {
	speed = std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

	analogWrite(MOTOR_NE_PWM, std::abs(speed));
	digitalWrite(MOTOR_NE_DIR, speed > 0);
	motorNE.speed = std::abs(speed);
	motorNE.direction = speed > 0 ? 1 : -1;
}

void Motor::setSpeedNW (int16_t speed) {
	speed = std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

	analogWrite(MOTOR_NW_PWM, std::abs(speed));
	digitalWrite(MOTOR_NW_DIR, speed > 0);
	motorNW.speed = std::abs(speed);
	motorNW.direction = speed > 0 ? 1 : -1;
}

void Motor::individualMotor::callback(const unsigned int gpio, unsigned long events) {
	switch (gpio) {
		case MOTOR_SE_ENC_B: pulses_SE++; break;
		case MOTOR_SW_ENC_B: pulses_SW++; break;
		case MOTOR_NE_ENC_B: pulses_NE++; break;
		case MOTOR_NW_ENC_B: pulses_NW++; break;
		default: break;
	}
}

void Motor::individualMotor::getRPM() {
	unsigned long pulses;
	unsigned long long previous_pulses_timestamp;
	switch (id) {
		case MOTOR_SE: pulses = pulses_SE; previous_pulses_timestamp = previous_pulses_SE_timestamp; break;
		case MOTOR_SW: pulses = pulses_SW; previous_pulses_timestamp = previous_pulses_SW_timestamp; break;
		case MOTOR_NE: pulses = pulses_NE; previous_pulses_timestamp = previous_pulses_NE_timestamp; break;
		case MOTOR_NW: pulses = pulses_NW; previous_pulses_timestamp = previous_pulses_NW_timestamp; break;
		default: pulses = 0; previous_pulses_timestamp = 0; break;
	}

	const double elapsed_us = (micros() - previous_pulses_timestamp);
	rpm = (pulses * (SECOND * 60 * 1000)) / (GEAR_RATIO * TICKS_PER_REVOLUTION_ENCODER * elapsed_us);

	switch (id) {
		case MOTOR_SE: pulses_SE = 0; previous_pulses_SE_timestamp = micros(); break;
		case MOTOR_SW: pulses_SW = 0; previous_pulses_SW_timestamp = micros(); break;
		case MOTOR_NE: pulses_NE = 0; previous_pulses_NE_timestamp = micros(); break;
		case MOTOR_NW: pulses_NW = 0; previous_pulses_NW_timestamp = micros(); break;
		default: break;
	}
}

void Motor::stop() {
	if (motorSE.speed == 0 && motorSW.speed == 0 && motorNE.speed == 0 && motorNW.speed == 0) return;

	setSpeedSE(-MAX_SPEED * motorSE.speed / std::abs(motorSE.speed));
	setSpeedSW(-MAX_SPEED * motorSW.speed / std::abs(motorSW.speed));
	setSpeedNE(-MAX_SPEED * motorNE.speed / std::abs(motorNE.speed));
	setSpeedNW(-MAX_SPEED * motorNW.speed / std::abs(motorNW.speed));

	delay(50);

	setSpeedSE(0);
	setSpeedSW(0);
	setSpeedNE(0);
	setSpeedNW(0);

	dataPtr->robot_stop = false;
}

void Motor::moveSE(const int16_t rpm) {
	if (micros() - motorSE.lastIteration_ms < motorSE.delay_ms) return;

	motorSE.rpmPID.error = std::abs(rpm) - std::abs(motorSE.rpm);
	motorSE.rpmPID.target = std::abs(rpm);
	PID::compute(motorSE.rpmPID);

	if (rpm > 0) {
		if (motorSE.previousOut < 0) motorSE.previousOut *= -1;
		motorSE.previousOut = motorSE.previousOut + motorSE.rpmPID.output;
		motorSE.previousOut = std::clamp(motorSE.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorSE.previousOut > 0) motorSE.previousOut *= -1;
		motorSE.previousOut = motorSE.previousOut - motorSE.rpmPID.output;
		motorSE.previousOut = std::clamp(motorSE.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedSE(motorSE.previousOut);
}

void Motor::moveSW(const int16_t rpm) {
	if (micros() - motorSW.lastIteration_ms < motorSW.delay_ms) return;

	motorSW.rpmPID.error = std::abs(rpm) - std::abs(motorSW.rpm);
	motorSW.rpmPID.target = std::abs(rpm);
	PID::compute(motorSW.rpmPID);

	if (rpm > 0) {
		if (motorSW.previousOut < 0) motorSW.previousOut *= -1;
		motorSW.previousOut = motorSW.previousOut + motorSW.rpmPID.output;
		motorSW.previousOut = std::clamp(motorSW.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorSW.previousOut > 0) motorSW.previousOut *= -1;
		motorSW.previousOut = motorSW.previousOut - motorSW.rpmPID.output;
		motorSW.previousOut = std::clamp(motorSW.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedSW(motorSW.previousOut);
}

void Motor::moveNE(const int16_t rpm) {
	if (micros() - motorNE.lastIteration_ms < motorNE.delay_ms) return;

	motorNE.rpmPID.error = std::abs(rpm) - std::abs(motorNE.rpm);
	motorNE.rpmPID.target = std::abs(rpm);
	PID::compute(motorNE.rpmPID);

	if (rpm > 0) {
		if (motorNE.previousOut < 0) motorNE.previousOut *= -1;
		motorNE.previousOut = motorNE.previousOut + motorNE.rpmPID.output;
		motorNE.previousOut = std::clamp(motorNE.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorNE.previousOut > 0) motorNE.previousOut *= -1;
		motorNE.previousOut = motorNE.previousOut - motorNE.rpmPID.output;
		motorNE.previousOut = std::clamp(motorNE.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedNE(motorNE.previousOut);
}

void Motor::moveNW(const int16_t rpm) {
	if (micros() - motorNW.lastIteration_ms < motorNW.delay_ms) return;

	motorNW.rpmPID.error = std::abs(rpm) - std::abs(motorNW.rpm);
	motorNW.rpmPID.target = std::abs(rpm);
	PID::compute(motorNW.rpmPID);

	if (rpm > 0) {
		if (motorNW.previousOut < 0) motorNW.previousOut *= -1;
		motorNW.previousOut = motorNW.previousOut + motorNW.rpmPID.output;
		motorNW.previousOut = std::clamp(motorNW.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorNW.previousOut > 0) motorNW.previousOut *= -1;
		motorNW.previousOut = motorNW.previousOut - motorNW.rpmPID.output;
		motorNW.previousOut = std::clamp(motorNW.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedNW(motorNW.previousOut);
}

void Motor::move(const float rpm, const float direction, const float facing_target, const float facing_current) {
	double NWSpeed = rpm * cos((PI / 180) * (direction + 315));
	double SWSpeed = rpm * cos((PI / 180) * (direction + 225));
	double SESpeed = rpm * cos((PI / 180) * (direction + 135));
	double NESpeed = rpm * cos((PI / 180) * (direction + 45));

	double values[] = { NWSpeed, SWSpeed, SESpeed, NESpeed };
	const double max = *std::max_element(values, values + 4);
	const double ratio = max != 0 ? rpm / max : 0;

	NWSpeed *= ratio;
	SWSpeed *= ratio;
	SESpeed *= ratio;
	NESpeed *= ratio;

	rotationPID.target = facing_target;
	rotationPID.error = facing_target - facing_current;
	PID::compute(rotationPID);
	std::cout << facing_current << " " << rotationPID.output << std::endl;

	setSpeedNW(NWSpeed + rotationPID.output);
	setSpeedSW(SWSpeed + rotationPID.output);
	setSpeedSE(SESpeed - rotationPID.output);
	setSpeedNE(NESpeed - rotationPID.output);
}