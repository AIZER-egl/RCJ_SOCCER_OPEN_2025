/**
 * cd ~/Desktop/pico-lib/build
 * make
 *
 * cp ~/Documents/RoboticProjects/RCJ_SOCCER_OPEN_2025/rpi_pico/build/main.uf2 /media/iker/RPI-RP2/main.uf2
*/
#include <iostream>
#include <string>
#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico-lib/gpio.h"
#include "lib/software/serializer.h"
#include "lib/software/pid.h"
#include "lib/software/hex.h"
#include "lib/hardware/kicker.h"
#include "lib/hardware/motor.h"
#include "lib/hardware/compass_classes.h"
#include "lib/hardware/light_sensor.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define SDA 4
#define SCL 5
#define ARDUINO_I2C 8
#define ACTION_BUTTON 28

#define ROBOT_ON true
#define ROBOT_OFF false

bool disabled = false;
std::vector<uint8_t> message = {};
int ldr_3_threshold = 1024;

void sendData(BinarySerializationData& data) {
	std::vector<uint8_t> bytes = Serializer::serialize(data);

	if (!bytes.empty()) {
		// For whatever reason, the PICO is transforming the \n to a \r\n
		// Since pico is reliably (yet unknown) sending a \r the jetson will just accept this behaviour
		bytes.push_back('\n');
		size_t bytes_written = fwrite(
			bytes.data(),
			sizeof(uint8_t),
			bytes.size(),
			stdout
		);

		fflush(stdout);
	}
}

int main () {
    stdio_init_all();

	delay(2000);

	Motor motor;
	Kicker kicker;
	Light_Sensor light_sensor;
	Adafruit_BNO055 bno;

	unsigned long long last_kicker_time = millis();
	unsigned long long last_motor_time = millis();
	unsigned long long last_led_time = millis();
	unsigned long led_rate = 500;
    bool previous_action_button_read = false;
    bool action_button_read = false;
	bool robot_state = false;
	bool led_state = true;

	BinarySerializationData data{};
	data.compass_yaw = 0;
    data.robot_direction = 0;
    data.robot_speed = 0;
    data.robot_facing = 0;
    data.robot_stop = false;
	data.kicker_active = false;

    bno.begin(I2C_PORT_0, 100 * 1000, SDA, SCL, data);
    motor.begin(data);
    kicker.begin(data);
	light_sensor.begin(data);

	pinMode(KICKER, OUTPUT);
	pinMode(ACTION_BUTTON, INPUT);
	pinMode(BUILTIN_LED, OUTPUT);
	digitalWrite(BUILTIN_LED, HIGH);

	uint8_t i = 0;

	for (;;) {
		bno.tick();
		motor.tick();
		kicker.tick();
		light_sensor.tick();

		action_button_read = digitalRead(ACTION_BUTTON);

		if (ldr_3_threshold < light_sensor.ldr_3) {}

		int received_char;
		while ((received_char = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
			char byte = (char)received_char;
			if (byte == '\n') {
				std::optional<BinarySerializationData> receivedData = Serializer::deserialize(message);
				if (receivedData.has_value()) {
					data.robot_direction = receivedData.value().robot_direction;
					data.robot_speed = receivedData.value().robot_speed;
					data.robot_facing = receivedData.value().robot_facing;
					data.robot_stop = receivedData.value().robot_stop;
					data.kicker_active = receivedData.value().kicker_active;
				}

				sendData(data);

				message.clear();
			} else {
				message.push_back(byte);
			}
			if (message.size() > 100) {
				message.clear();
			}
		}

		if (action_button_read && !previous_action_button_read) {
			if (robot_state == ROBOT_OFF) {
				PID::reset(motor.motorSE.rpsPID);
				PID::reset(motor.motorSW.rpsPID);
				PID::reset(motor.motorNE.rpsPID);
				PID::reset(motor.motorNW.rpsPID);
				PID::reset(motor.rotationPID);
				robot_state = ROBOT_ON;
				led_rate = 100;
				// kicker.kick();
				bno.setYawOffset(bno.raw_yaw);
				motor.motorSE.setSpeed(0);
				motor.motorSW.setSpeed(0);
				motor.motorNW.setSpeed(0);
				motor.motorNE.setSpeed(0);
				motor.motorSE.rps_count = 0;
				motor.motorSE.rps_summatory = 0;
				motor.motorSW.rps_count = 0;
				motor.motorSW.rps_summatory = 0;
				motor.motorNE.rps_count = 0;
				motor.motorNE.rps_summatory = 0;
				motor.motorNW.rps_count = 0;
				motor.motorNW.rps_summatory = 0;
				ldr_3_threshold = light_sensor.ldr_3 + 40;
			} else {
				motor.stop();
				led_rate = 500;
				robot_state = ROBOT_OFF;
			}
		}

		if (millis() - last_led_time >= led_rate) {
			digitalWrite(BUILTIN_LED, led_state);
			led_state = !led_state;
			last_led_time = millis();
		}

		if (millis() - last_kicker_time >= 10000) {
			if (data.kicker_active) {
				kicker.kick();
			}

			last_kicker_time = millis();
		}

		if (millis() - last_motor_time >= 25) {
			if (data.robot_stop || robot_state == ROBOT_OFF) {
				motor.stop();
			} else {
				std::cout << "NE: " << motor.motorNE.getRPS_average() <<
					" NW: " << motor.motorNW.getRPS_average() <<
					" SW: " << motor.motorSW.getRPS_average() <<
					" SE: " << motor.motorSE.getRPS_average() << std::endl;

				motor.move(
					static_cast<float>(data.robot_speed) / 10,
					data.robot_direction,
					data.robot_facing,
					data.compass_yaw
				);
			}

			last_motor_time = millis();
		}

		previous_action_button_read = action_button_read;
	}
}


#pragma clang diagnostic pop
