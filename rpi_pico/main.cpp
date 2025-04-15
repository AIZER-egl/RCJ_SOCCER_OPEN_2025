/**
 * cd ~/Desktop/pico-lib/build
 * make
 *
 * cp ~/Documents/RoboticProjects/robocup2024/rpi_pico/build/main.uf2 /media/iker/RPI-RP2/main.uf2
*/
#include <cstdint>
#include <string>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico-lib/gpio.h"
#include "lib/software/bitmask.h"
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

int main () {
    stdio_init_all();
    USB_Serial_Init();

	Motor motor;
	Kicker kicker;
	Adafruit_BNO055 bno;

	bno.begin(I2C_PORT_0, 100 * 1000, SDA, SCL);
	motor.begin();
	// kicker.begin();

    pinMode(27, OUTPUT);
	digitalWrite(27, HIGH);

	// delay(2000);
	// kicker.kick();
 //
	// unsigned long last_time = millis();
 //    bool state = true;
 //
	// for (;;) {
	// 	motor.tick();
	// 	kicker.tick();
 //
	// 	if (millis() - last_time >= 2000) {
	// 		kicker.kick();
	// 		last_time = millis();
	// 	}
	// }
}


#pragma clang diagnostic pop


// uint16_t raw_value = adc_read();
// const float conversion_factor = 3.3f / (1 << 12); // (1 << 12) is 4096
// float voltage = raw_value * conversion_factor;
//
// last_time = millis();
// std::cout << voltage << " " << raw_value << std::endl;
