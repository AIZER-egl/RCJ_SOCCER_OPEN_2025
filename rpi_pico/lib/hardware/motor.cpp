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

	motorSE.rpsPID.maxOutput = MAX_SPEED;
	motorSE.rpsPID.maxError = 1000.0;
	motorSE.rpsPID.errorThreshold = 3.0;
	motorSE.rpsPID.kp = 0.1;
	motorSE.rpsPID.ki = 0.0001;
	motorSE.rpsPID.kd = 0.0;

	motorSW.rpsPID.maxOutput = MAX_SPEED;
	motorSW.rpsPID.maxError = 1000.0;
	motorSW.rpsPID.errorThreshold = 3.0;
	motorSW.rpsPID.kp = 0.1;
	motorSW.rpsPID.ki = 0.0001;
	motorSW.rpsPID.kd = 0.0;

	motorNW.rpsPID.maxOutput = MAX_SPEED;
	motorNW.rpsPID.maxError = 1000.0;
	motorNW.rpsPID.errorThreshold = 3.0;
	motorNW.rpsPID.kp = 0.1;
	motorNW.rpsPID.ki = 0.0001;
	motorNW.rpsPID.kd = 0.0;

	motorNE.rpsPID.maxOutput = MAX_SPEED;
	motorNE.rpsPID.maxError = 1000.0;
	motorNE.rpsPID.errorThreshold = 3.0;
	motorNE.rpsPID.kp = 0.1;
	motorNE.rpsPID.ki = 0.0001;
	motorNE.rpsPID.kd = 0.0;

	rotationPID.maxOutput = MAX_RPS;
	rotationPID.maxError = 1000.0;
	rotationPID.errorThreshold = 3.0;
	rotationPID.kp = 0.25;
	rotationPID.ki = 0.4;
	rotationPID.kd = 0.16;
}

void Motor::tick() {
	if (!dataPtr) return;

	if (millis() - previousTick >= 10) {
		motorSE.getRPS();
		motorSW.getRPS();
		motorNE.getRPS();
		motorNW.getRPS();

		dataPtr -> motor_nw_rps = motorNW.rps;
		dataPtr -> motor_ne_rps = motorNE.rps;
		dataPtr -> motor_sw_rps = motorSW.rps;
		dataPtr -> motor_se_rps = motorSE.rps;

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
	// TODO: Hardcoded 10 offset
	speed = std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

	analogWrite(MOTOR_NE_PWM, std::abs(speed) + 10);
	digitalWrite(MOTOR_NE_DIR, speed > 0);
	motorNE.speed = std::abs(speed) + 10;
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

void Motor::individualMotor::getRPS() {
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
	rps = (pulses * (SECOND * 1000)) / (GEAR_RATIO * TICKS_PER_REVOLUTION_ENCODER * elapsed_us);

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

	if (motorSE.speed != 0) setSpeedSE(-MAX_SPEED * motorSE.speed / std::abs(motorSE.speed));
	if (motorSW.speed != 0) setSpeedSW(-MAX_SPEED * motorSW.speed / std::abs(motorSW.speed));
	if (motorNE.speed != 0) setSpeedNE(-MAX_SPEED * motorNE.speed / std::abs(motorNE.speed));
	if (motorNW.speed != 0) setSpeedNW(-MAX_SPEED * motorNW.speed / std::abs(motorNW.speed));

	delay(50);

	setSpeedSE(0);
	setSpeedSW(0);
	setSpeedNE(0);
	setSpeedNW(0);

	dataPtr->robot_stop = false;
}

void Motor::moveSE(const int16_t rps) {
	if (micros() - motorSE.lastIteration_ms < motorSE.delay_ms) return;

	motorSE.rpsPID.error = std::abs(rps) - std::abs(motorSE.rps);
	motorSE.rpsPID.target = std::abs(rps);
	PID::compute(motorSE.rpsPID);

	if (rps > 0) {
		if (motorSE.previousOut < 0) motorSE.previousOut *= -1;
		motorSE.previousOut = motorSE.previousOut + motorSE.rpsPID.output;
		motorSE.previousOut = std::clamp(motorSE.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorSE.previousOut > 0) motorSE.previousOut *= -1;
		motorSE.previousOut = motorSE.previousOut - motorSE.rpsPID.output;
		motorSE.previousOut = std::clamp(motorSE.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedSE(motorSE.previousOut);
}

void Motor::moveSW(const int16_t rps) {
	if (micros() - motorSW.lastIteration_ms < motorSW.delay_ms) return;

	motorSW.rpsPID.error = std::abs(rps) - std::abs(motorSW.rps);
	motorSW.rpsPID.target = std::abs(rps);
	PID::compute(motorSW.rpsPID);

	if (rps > 0) {
		if (motorSW.previousOut < 0) motorSW.previousOut *= -1;
		motorSW.previousOut = motorSW.previousOut + motorSW.rpsPID.output;
		motorSW.previousOut = std::clamp(motorSW.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorSW.previousOut > 0) motorSW.previousOut *= -1;
		motorSW.previousOut = motorSW.previousOut - motorSW.rpsPID.output;
		motorSW.previousOut = std::clamp(motorSW.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedSW(motorSW.previousOut);
}

void Motor::moveNE(const int16_t rps) {
	if (micros() - motorNE.lastIteration_ms < motorNE.delay_ms) return;

	motorNE.rpsPID.error = std::abs(rps) - std::abs(motorNE.rps);
	motorNE.rpsPID.target = std::abs(rps);
	PID::compute(motorNE.rpsPID);

	if (rps > 0) {
		if (motorNE.previousOut < 0) motorNE.previousOut *= -1;
		motorNE.previousOut = motorNE.previousOut + motorNE.rpsPID.output;
		motorNE.previousOut = std::clamp(motorNE.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorNE.previousOut > 0) motorNE.previousOut *= -1;
		motorNE.previousOut = motorNE.previousOut - motorNE.rpsPID.output;
		motorNE.previousOut = std::clamp(motorNE.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedNE(motorNE.previousOut);
}

void Motor::moveNW(const int16_t rps) {
	if (micros() - motorNW.lastIteration_ms < motorNW.delay_ms) return;

	motorNW.rpsPID.error = std::abs(rps) - std::abs(motorNW.rps);
	motorNW.rpsPID.target = std::abs(rps);
	PID::compute(motorNW.rpsPID);

	if (rps > 0) {
		if (motorNW.previousOut < 0) motorNW.previousOut *= -1;
		motorNW.previousOut = motorNW.previousOut + motorNW.rpsPID.output;
		motorNW.previousOut = std::clamp(motorNW.previousOut, MIN_SPEED, MAX_SPEED);
	} else {
		if (motorNW.previousOut > 0) motorNW.previousOut *= -1;
		motorNW.previousOut = motorNW.previousOut - motorNW.rpsPID.output;
		motorNW.previousOut = std::clamp(motorNW.previousOut, -MAX_SPEED, -MIN_SPEED);
	}

	setSpeedNW(motorNW.previousOut);
}

void Motor::move(const float rps, const float direction, const float facing_target, const float facing_current) {
	double NWSpeed = rps * cos((PI / 180) * (direction + 350));
	double SWSpeed = rps * cos((PI / 180) * (direction) + 190);
	double SESpeed = rps * cos((PI / 180) * (direction) + 170);
	double NESpeed = rps * cos((PI / 180) * (direction) + 10);

	double values[] = { NWSpeed, SWSpeed, SESpeed, NESpeed };
	const double max = *std::max_element(values, values + 4);
	const double ratio = max != 0 ? rps / max : 0;

	NWSpeed *= ratio;
	SWSpeed *= ratio;
	SESpeed *= ratio;
	NESpeed *= ratio;

	rotationPID.target = facing_target;
	rotationPID.error = facing_target - facing_current;
	PID::compute(rotationPID);

	setSpeedNW(NWSpeed + rotationPID.output);
	setSpeedSW(SWSpeed - rotationPID.output);
	setSpeedSE(SESpeed + rotationPID.output);
	setSpeedNE(NESpeed - rotationPID.output);
}