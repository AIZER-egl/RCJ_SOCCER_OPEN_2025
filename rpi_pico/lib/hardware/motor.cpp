#include "motor.h"

volatile unsigned long pulses_SE;
volatile unsigned long pulses_SW;
volatile unsigned long pulses_NE;
volatile unsigned long pulses_NW;

unsigned long long previous_pulses_SE_timestamp;
unsigned long long previous_pulses_SW_timestamp;
unsigned long long previous_pulses_NE_timestamp;
unsigned long long previous_pulses_NW_timestamp;

Motor::Motor() : dataPtr(nullptr) {
	motorSE.pwm_1_pin = MOTOR_SE_PWM_1;
	motorSE.pwm_2_pin = MOTOR_SE_PWM_2;

	motorSW.pwm_1_pin = MOTOR_SW_PWM_1;
	motorSW.pwm_2_pin = MOTOR_SW_PWM_2;

	motorNE.pwm_1_pin = MOTOR_NE_PWM_1;
	motorNE.pwm_2_pin = MOTOR_NE_PWM_2;

	motorNW.pwm_1_pin = MOTOR_NW_PWM_1;
	motorNW.pwm_2_pin = MOTOR_NW_PWM_2;
};

void Motor::begin (BinarySerializationData& data) {
	dataPtr = &data;

	motorNE.id = MOTOR_NE;
	motorNW.id = MOTOR_NW;
	motorSE.id = MOTOR_SE;
	motorSW.id = MOTOR_SW;

	pinMode(MOTOR_NE_PWM_1, OUTPUT_PWM);
	pinMode(MOTOR_NE_PWM_2, OUTPUT_PWM);
	pinMode(MOTOR_NW_PWM_1, OUTPUT_PWM);
	pinMode(MOTOR_NW_PWM_2, OUTPUT_PWM);
	pinMode(MOTOR_SE_PWM_1, OUTPUT_PWM);
	pinMode(MOTOR_SE_PWM_2, OUTPUT_PWM);
	pinMode(MOTOR_SW_PWM_1, OUTPUT_PWM);
	pinMode(MOTOR_SW_PWM_2, OUTPUT_PWM);

	interrupts(
		std::vector<uint8_t> { MOTOR_NE_ENC_B, MOTOR_NW_ENC_B, MOTOR_SW_ENC_B, MOTOR_SE_ENC_B },
		[] (const unsigned int gpio, const unsigned long events) {
			Motor::individualMotor::callback(gpio, events);
		}
	);

	motorSE.name = "SE";
	motorSW.name = "SW";
	motorNE.name = "NE";
	motorNW.name = "NW";

	motorSE.rpsPID.one_direction_only = true;
	motorSE.rpsPID.max_output = MAX_SPEED;
	motorSE.rpsPID.error_threshold = 0.0;
	motorSE.rpsPID.kp = 40;
	motorSE.rpsPID.ki = 30;
	motorSE.rpsPID.kd = 1;

	motorSW.rpsPID.one_direction_only = true;
	motorSW.rpsPID.max_output = MAX_SPEED;
	motorSW.rpsPID.error_threshold = 0.0;
	motorSW.rpsPID.kp = 40;
	motorSW.rpsPID.ki = 30;
	motorSW.rpsPID.kd = 1;

	motorNW.rpsPID.one_direction_only = true;
	motorNW.rpsPID.max_output = MAX_SPEED;
	motorNW.rpsPID.error_threshold = 0.0;
	motorNW.rpsPID.kp = 40;
	motorNW.rpsPID.ki = 30;
	motorNW.rpsPID.kd = 1;

	motorNE.rpsPID.one_direction_only = true;
	motorNE.rpsPID.max_output = MAX_SPEED;
	motorNE.rpsPID.error_threshold = 0.0;
	motorNE.rpsPID.kp = 51;
	motorNE.rpsPID.ki = 36;
	motorNE.rpsPID.kd = 1;

	rotationPID.one_direction_only = false;
	rotationPID.reset_within_threshold = true;
	rotationPID.max_output = 1.5;
	rotationPID.error_threshold = 8.0;
	rotationPID.kp = 0.01;
	rotationPID.ki = 0.01;
	rotationPID.kd = 0.0;
}

void Motor::tick() {
	if (!dataPtr) return;

	if (millis() - previousTick >= 10) {
		motorSE.getRPS();
		motorSW.getRPS();
		motorNE.getRPS();
		motorNW.getRPS();

		// dataPtr -> motor_nw_rps = static_cast<float>(motorNW.rps);
		// dataPtr -> motor_ne_rps = static_cast<float>(motorNE.rps);
		// dataPtr -> motor_sw_rps = static_cast<float>(motorSW.rps);
		// dataPtr -> motor_se_rps = static_cast<float>(motorSE.rps);

		previousTick = millis();

		if (robot_stopped) stop();
	}
}

void Motor::individualMotor::setSpeed(int16_t new_speed, bool save_direction) {
	new_speed = std::clamp(new_speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

	if (std::abs(new_speed) < MIN_SPEED) {
		if (std::abs(new_speed) < (MIN_SPEED / 2)) {
			new_speed = 0;
		} else {
			new_speed = MIN_SPEED * (new_speed > 0 ? 1 : -1);
		}
	}

	if (new_speed < 0) {
		analogWrite(pwm_1_pin, std::abs(new_speed));
		analogWrite(pwm_2_pin, 0);
	} else {
		analogWrite(pwm_1_pin, 0);
		analogWrite(pwm_2_pin, std::abs(new_speed));
	}

	if (save_direction) {
		speed = std::abs(new_speed);
		direction = new_speed > 0 ? 1 : -1;
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

	previous_rps = rps;
	const double elapsed_us = (micros() - previous_pulses_timestamp);
	if (elapsed_us > 0)
		rps = (pulses * (SECOND * 1000)) / (GEAR_RATIO * TICKS_PER_REVOLUTION_ENCODER * elapsed_us);
	else
		rps = 0;

	if (rps > 18) rps = previous_rps; // SW Motor, Robot 1 encoder is failing, using this guard to prevent making things worse

	rps_summatory += rps;
	rps_count += 1;

	switch (id) {
		case MOTOR_SE: pulses_SE = 0; previous_pulses_SE_timestamp = micros(); break;
		case MOTOR_SW: pulses_SW = 0; previous_pulses_SW_timestamp = micros(); break;
		case MOTOR_NE: pulses_NE = 0; previous_pulses_NE_timestamp = micros(); break;
		case MOTOR_NW: pulses_NW = 0; previous_pulses_NW_timestamp = micros(); break;
		default: break;
	}
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

double Motor::individualMotor::getRPS_average() {
	if (rps_count == 0) return 0;
	return rps_summatory / rps_count;
}

void Motor::stop() {
	if (!robot_stopped) robot_stopped = true;
	dataPtr -> robot_speed = 0;
	dataPtr -> robot_direction = 0;

	analogWrite(MOTOR_SW_PWM_1, MAX_SPEED);
	analogWrite(MOTOR_SW_PWM_2, MAX_SPEED);
	analogWrite(MOTOR_SE_PWM_1, MAX_SPEED);
	analogWrite(MOTOR_SE_PWM_2, MAX_SPEED);
	analogWrite(MOTOR_NW_PWM_1, MAX_SPEED);
	analogWrite(MOTOR_NW_PWM_2, MAX_SPEED);
	analogWrite(MOTOR_NE_PWM_1, MAX_SPEED);
	analogWrite(MOTOR_NE_PWM_2, MAX_SPEED);
	// robot_stopped flag is automatically set to false when move method is called
}

void Motor::individualMotor::move(const double new_rps) {
	if (new_rps == 0) {
		setSpeed(0);
		return;
	}

	if (new_rps > 0) rpsPID.accept_type = PID_ACCEPT_POSITIVES_ONLY;
	else rpsPID.accept_type = PID_ACCEPT_NEGATIVES_ONLY;

	rpsPID.error = (std::abs(new_rps) - rps) * (new_rps > 0 ? 1 : -1);
	rpsPID.target = new_rps;

	PID::compute(rpsPID);

	setSpeed(static_cast<int16_t>(rpsPID.output));
}

void Motor::move(const float rps, const float direction, const float facing_target, const float facing_current, const bool save_direction) {
	if (save_direction) current_direction = direction;
	// Robot stop is automatically turned of if move function is called
	if (robot_stopped) robot_stopped = false;


	double NWSpeed = rps * cos((PI / 180) * (direction + 315));
	double SWSpeed = rps * cos((PI / 180) * (direction + 225));
	double SESpeed = rps * cos((PI / 180) * (direction + 135));
	double NESpeed = rps * cos((PI / 180) * (direction + 45));

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

	motorNW.move(NWSpeed + 0);
	motorSW.move(SWSpeed - 0);
	motorSE.move(SESpeed + rotationPID.output);
	motorNE.move(NESpeed - rotationPID.output);
}