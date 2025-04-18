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
#include "lib/hardware/USB_Serial.h"
#include "lib/hardware/compass_classes.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define SDA 4
#define SCL 5
#define ARDUINO_I2C 8
#define START_BUTTON 21

#define LDR 28

float yaw = 0;
float yaw_offset = 0;
bool disabled = false;

void sendData(BinarySerializationData& data) {
	std::vector<uint8_t> bytes = Serializer::serialize(data);

	if (!bytes.empty()) {
		fwrite(
			bytes.data(),
			1,
			bytes.size(),
			stdout
		);

		fflush(stdout);
	}
}

int main () {
    stdio_init_all();
    USB_Serial_Init();

	Motor motor;
	Kicker kicker;
	Adafruit_BNO055 bno;

	// bno.begin(I2C_PORT_0, 100 * 1000, SDA, SCL);
	motor.begin();
	kicker.begin();

	pinMode(KICKER, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);
	digitalWrite(BUILTIN_LED, HIGH);

	delay(2000);
	kicker.kick();

	unsigned long last_time = millis();
	bool state = true;

	BinarySerializationData data{};
	data.motor_se_speed = 255;
	data.motor_sw_speed = 255;
	data.motor_ne_speed = 255;
	data.motor_nw_speed = 255;
	data.motor_se_rpm = 255;
	data.motor_sw_rpm = 255;
	data.motor_ne_rpm = 255;
	data.motor_nw_rpm = 255;
	data.motor_se_direction = 1;
	data.motor_sw_direction = 1;
	data.motor_ne_direction = 1;
	data.motor_nw_direction = 1;
	data.compass_yaw = -179;
	data.kicker_active = true;
	data.ldr_value = 255;
	data.setting_team_id = 1;
	data.setting_attack_goal = 1;

	for (;;) {
		motor.tick();
		kicker.tick();

		if (USB_Serial_Available()) {
			char byte = USB_Serial_Get_Byte();
			if (byte == '\n') {
				sendData(data);
			}
		}
	}
}


#pragma clang diagnostic pop


// uint16_t raw_value = adc_read();
// const float conversion_factor = 3.3f / (1 << 12); // (1 << 12) is 4096
// float voltage = raw_value * conversion_factor;
//
// last_time = millis();
// std::cout << voltage << " " << raw_value << std::endl;
