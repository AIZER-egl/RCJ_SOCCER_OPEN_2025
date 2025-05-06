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
#ifdef ROBOT_1
		motorSE.pwm_pin = R1_MOTOR_SE_PWM;
		motorSE.dir_pin = R1_MOTOR_SE_DIR;
		motorSE.encoder_b_pin = R1_MOTOR_SE_ENC_B;

		motorSW.pwm_pin = R1_MOTOR_SW_PWM;
		motorSW.dir_pin = R1_MOTOR_SW_DIR;
		motorSW.encoder_b_pin = R1_MOTOR_SW_ENC_B;

		motorNE.pwm_pin = R1_MOTOR_NE_PWM;
		motorNE.dir_pin = R1_MOTOR_NE_DIR;
		motorNE.encoder_b_pin = R1_MOTOR_NE_ENC_B;

		motorNW.pwm_pin = R1_MOTOR_NW_PWM;
		motorNW.dir_pin = R1_MOTOR_NW_DIR;
		motorNW.encoder_b_pin = R1_MOTOR_NW_ENC_B;
#endif
#ifdef ROBOT_2
		motorSE.pwm_pin = R2_MOTOR_SE_PWM;
		motorSE.dir_pin = R2_MOTOR_SE_DIR;
		motorSE.encoder_b_pin = R2_MOTOR_SE_ENC_B;

		motorSW.pwm_pin = R2_MOTOR_SW_PWM;
		motorSW.dir_pin = R2_MOTOR_SW_DIR;
		motorSW.encoder_b_pin = R2_MOTOR_SW_ENC_B;

		motorNE.pwm_pin = R2_MOTOR_NE_PWM;
		motorNE.dir_pin = R2_MOTOR_NE_DIR;
		motorNE.encoder_b_pin = R2_MOTOR_NE_ENC_B;

		motorNW.pwm_pin = R2_MOTOR_NW_PWM;
		motorNW.dir_pin = R2_MOTOR_NW_DIR;
		motorNW.encoder_b_pin = R2_MOTOR_NW_ENC_B;
#endif
};

void Motor::begin (BinarySerializationData& data) {
	dataPtr = &data;

	motorNE.id = MOTOR_NE;
	motorNW.id = MOTOR_NW;
	motorSE.id = MOTOR_SE;
	motorSW.id = MOTOR_SW;

	pinMode(motorNE.dir_pin, OUTPUT);
	pinMode(motorNE.pwm_pin, OUTPUT_PWM);
	pinMode(motorNW.dir_pin, OUTPUT);
	pinMode(motorNW.pwm_pin, OUTPUT_PWM);
	pinMode(motorSE.dir_pin, OUTPUT);
	pinMode(motorSE.pwm_pin, OUTPUT_PWM);
	pinMode(motorSW.dir_pin, OUTPUT);
	pinMode(motorSW.pwm_pin, OUTPUT_PWM);

	interrupts(
		std::vector<uint8_t> { motorNE.encoder_b_pin, motorNW.encoder_b_pin, motorSW.encoder_b_pin, motorSE.encoder_b_pin },
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
	motorSE.rpsPID.kp = 5;
	motorSE.rpsPID.ki = 50;
	motorSE.rpsPID.kd = 0;

	motorSW.rpsPID.one_direction_only = true;
	motorSW.rpsPID.max_output = MAX_SPEED;
	motorSW.rpsPID.error_threshold = 0.0;
	motorSW.rpsPID.kp = 5;
	motorSW.rpsPID.ki = 50;
	motorSW.rpsPID.kd = 0;

	motorNW.rpsPID.one_direction_only = true;
	motorNW.rpsPID.max_output = MAX_SPEED;
	motorNW.rpsPID.error_threshold = 0.0;
	motorNW.rpsPID.kp = 5;
	motorNW.rpsPID.ki = 50;
	motorNW.rpsPID.kd = 0;

	motorNE.rpsPID.one_direction_only = true;
	motorNE.rpsPID.max_output = MAX_SPEED;
	motorNE.rpsPID.error_threshold = 0.0;
	motorNE.rpsPID.kp = 5;
	motorNE.rpsPID.ki = 50;
	motorNE.rpsPID.kd = 0;

	rotationPID.one_direction_only = false;
	rotationPID.reset_within_threshold = true;
	rotationPID.max_output = 1.5;
	rotationPID.error_threshold = 8.0;
	rotationPID.kp = 0.015;
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

	analogWrite(pwm_pin, std::abs(new_speed));

#ifdef ROBOT_1
	digitalWrite(dir_pin, new_speed > 0);
#endif
#ifdef ROBOT_2
	// Because of an error when setting up the second robot
	// the motor cables were mixed and now the logic is inverted
	digitalWrite(dir_pin, new_speed < 0);
#endif

	if (save_direction) {
		speed = std::abs(new_speed);
		if (new_speed != 0) direction = new_speed > 0 ? 1 : -1;
		// If direction is equal to 0, direction will remain unchanged
		// (asumming a PID controller is probably doing some things with speed 0 to keep things working well)
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
#ifdef ROBOT_1
		case R1_MOTOR_SE_ENC_B: pulses_SE++; break;
		case R1_MOTOR_SW_ENC_B: pulses_SW++; break;
		case R1_MOTOR_NE_ENC_B: pulses_NE++; break;
		case R1_MOTOR_NW_ENC_B: pulses_NW++; break;
#endif
#ifdef ROBOT_2
		case R2_MOTOR_SE_ENC_B: pulses_SE++; break;
		case R2_MOTOR_SW_ENC_B: pulses_SW++; break;
		case R2_MOTOR_NE_ENC_B: pulses_NE++; break;
		case R2_MOTOR_NW_ENC_B: pulses_NW++; break;
#endif
		default: break;
	}
}

double Motor::individualMotor::getRPS_average() {
	if (rps_count == 0) return 0;
	return rps_summatory / rps_count;
}

void Motor::stop() {
	if (motorSE.rps == 0 && motorSW.rps == 0 && motorNE.rps == 0 && motorNW.rps == 0) return;

	if (!robot_stopped) robot_stopped = true;
	dataPtr -> robot_speed = 0;
	dataPtr -> robot_direction = 0;

	if (motorSE.speed != 0) motorSE.setSpeed(-MAX_SPEED * motorSE.speed / std::abs(motorSE.speed), false);
	if (motorSW.speed != 0) motorSW.setSpeed(-MAX_SPEED * motorSW.speed / std::abs(motorSW.speed), false);
	if (motorNE.speed != 0) motorNE.setSpeed(-MAX_SPEED * motorNE.speed / std::abs(motorNE.speed), false);
	if (motorNW.speed != 0) motorNW.setSpeed(-MAX_SPEED * motorNW.speed / std::abs(motorNW.speed), false);

	if (motorSE.rps < 1.5 || motorSE.rps > (motorSE.previous_rps + 0.2)) motorSE.setSpeed(0);
	if (motorSW.rps < 1.5 || motorSW.rps > (motorSW.previous_rps + 0.2)) motorSW.setSpeed(0);
	if (motorNE.rps < 1.5 || motorNE.rps > (motorNE.previous_rps + 0.2)) motorNE.setSpeed(0);
	if (motorNW.rps < 1.5 || motorNW.rps > (motorNW.previous_rps + 0.2)) motorNW.setSpeed(0);

	PID::reset(motorNW.rpsPID);
	PID::reset(motorSW.rpsPID);
	PID::reset(motorNE.rpsPID);
	PID::reset(motorSE.rpsPID);

	// robot_stopped flag is automatically set to false when move method is called
}

void Motor::individualMotor::move(const double new_rps) {
	if (std::abs(new_rps) < 0.1) {
		setSpeed(0);
		return;
	}

	const double current_signed_rps = rps * static_cast<double>(direction);
    rpsPID.error = new_rps - current_signed_rps;
	rpsPID.target = new_rps;

	if (new_rps > 0) rpsPID.accept_type = PID_ACCEPT_POSITIVES_ONLY;
	else rpsPID.accept_type = PID_ACCEPT_NEGATIVES_ONLY;

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

	motorNW.move(NWSpeed + rotationPID.output);
	motorSW.move(SWSpeed - rotationPID.output);
	motorSE.move(SESpeed + rotationPID.output);
	motorNE.move(NESpeed - rotationPID.output);
}
