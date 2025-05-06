#include "light_sensor.h"

Light_Sensor::Light_Sensor() {};

void Light_Sensor::begin(BinarySerializationData& data) {
	dataPtr = &data;

	spi_init(MCP_SPI_PORT, 1000 * 1000);
	spi_set_format(MCP_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

	gpio_set_function(MCP_SPI_RX, GPIO_FUNC_SPI);
	gpio_set_function(MCP_SPI_SCK, GPIO_FUNC_SPI);
	gpio_set_function(MCP_SPI_TX, GPIO_FUNC_SPI);

	gpio_init(MCP_SPI_CSN);
	gpio_set_dir(MCP_SPI_CSN, GPIO_OUT);
	gpio_put(MCP_SPI_CSN, true);

	previous_read = time_us_64() / 1000;
}

uint16_t Light_Sensor::getValue(uint8_t channel) {
	if (channel > 7) return 0;

	uint8_t tx_buf[3];
	uint8_t rx_buf[3];

	tx_buf[0] = 0x01; // Start bit
	tx_buf[1] = (0x08 | channel) << 4; // Single-Ended + Channel Number
	tx_buf[2] = 0x00; // Dummy byte

	gpio_put(MCP_SPI_CSN, false);
	sleep_us(1);

	spi_write_read_blocking(MCP_SPI_PORT, tx_buf, rx_buf, 3);

	sleep_us(1);
	gpio_put(MCP_SPI_CSN, true);

	const uint16_t result = ((rx_buf[1] & 0x03) << 8) | rx_buf[2];
	return result;
}

void Light_Sensor::tick() {
	unsigned long long timestamp_us = time_us_64();
	if (timestamp_us - previous_read >= 1000) {
		uint8_t channel_just_read = current_channel;
		uint16_t value = getValue(current_channel);

		if (channel_just_read < NUM_LDR_SENSORS) {
			ldr_readings[channel_just_read] = value;

			if (value > ldr_thresholds[channel_just_read]) {
				detection_active = true;
			}
		}

		current_channel++;
		if (current_channel >= NUM_LDR_SENSORS) {
			current_channel = 0;
		}

		previous_read = timestamp_us;
	}

	if (timestamp_us - previos_data_update >= 30000) {
		dataPtr -> ldr_value = static_cast<int16_t>(ldr_readings[0]);
		previos_data_update = time_us_64();
	}
}

void Light_Sensor::calibrate() {
	for (uint8_t i = 0; i < NUM_LDR_SENSORS; ++i) {
		uint16_t current_value = getValue(i);
		uint32_t threshold_temp = static_cast<uint32_t>(current_value) + CALIBRATION_OFFSET;
		ldr_thresholds[i] = (threshold_temp > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(threshold_temp);
	}

	detection_active = false;
}