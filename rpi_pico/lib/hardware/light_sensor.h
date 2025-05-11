#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <iostream>
#include <cstdint>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "../software/binarySerializationData.h"

#define MCP_SPI_PORT spi0
#define MCP_SPI_SCK 18
#define MCP_SPI_RX 16
#define MCP_SPI_TX 19
#define MCP_SPI_CSN 17

#define NUM_LDR_SENSORS 8
#define CALIBRATION_OFFSET 30

class Light_Sensor {
public:
	Light_Sensor();
	void begin(BinarySerializationData& data);
	void tick();

	void calibrate();

	bool detection_active = false;

	uint16_t ldr_readings[NUM_LDR_SENSORS]{};
private:
	uint8_t current_channel = 0;
	uint16_t getValue(uint8_t channel);
	BinarySerializationData* dataPtr;
	double previous_read{};
	double previos_data_update{};

	uint16_t ldr_thresholds[NUM_LDR_SENSORS]{};
};

#endif //LIGHT_SENSOR_H