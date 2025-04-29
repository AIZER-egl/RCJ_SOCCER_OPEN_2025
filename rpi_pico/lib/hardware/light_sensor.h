#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

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

class Light_Sensor {
public:
	Light_Sensor();
	void begin(BinarySerializationData& data);
	void tick();

	uint16_t ldr_0{};
	uint16_t ldr_1{};
	uint16_t ldr_2{};
	uint16_t ldr_3{};
	uint16_t ldr_4{};
	uint16_t ldr_5{};
	uint16_t ldr_6{};
	uint16_t ldr_7{};
private:
	uint8_t current_channel = 0;
	uint16_t getValue(uint8_t channel);
	BinarySerializationData* dataPtr;
	double previous_read{};
	double previos_data_update{};
};

#endif //LIGHT_SENSOR_H