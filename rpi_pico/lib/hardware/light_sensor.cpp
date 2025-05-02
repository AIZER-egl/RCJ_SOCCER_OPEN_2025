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
	unsigned long long timestamp_ms = time_us_64() / 1000;
	if (timestamp_ms - previous_read >= 1) {
		uint16_t value = getValue(current_channel);
		switch (current_channel) {
			case 0: ldr_0 = value; break;
			case 1: ldr_1 = value; break;
			case 2: ldr_2 = value; break;
			case 3: ldr_3 = value; break;
			case 4: ldr_4 = value; break;
			case 5: ldr_5 = value; break;
			case 6: ldr_6 = value; break;
			case 7: ldr_7 = value; break;
			default: break;
		}

		current_channel++;
		if (current_channel > 7) current_channel = 0;
		previous_read = time_us_64() / 1000;
	}

	if (timestamp_ms - previos_data_update >= 30) {
		dataPtr -> ldr_value = static_cast<int16_t>(ldr_0);
		previos_data_update = time_us_64() / 1000;
	}
	// 	switch (dataPtr->ldr_channel) {
	// 		case 0:	dataPtr->ldr_value = static_cast<int16_t>(ldr_0); break;
	// 		case 1:	dataPtr->ldr_value = static_cast<int16_t>(ldr_1); break;
	// 		case 2:	dataPtr->ldr_value = static_cast<int16_t>(ldr_2); break;
	// 		case 3:	dataPtr->ldr_value = static_cast<int16_t>(ldr_3); break;
	// 		case 4:	dataPtr->ldr_value = static_cast<int16_t>(ldr_4); break;
	// 		case 5:	dataPtr->ldr_value = static_cast<int16_t>(ldr_5); break;
	// 		case 6:	dataPtr->ldr_value = static_cast<int16_t>(ldr_6); break;
	// 		case 7:	dataPtr->ldr_value = static_cast<int16_t>(ldr_7); break;
	// 		default: dataPtr->ldr_value = 0; break;
	// 	}
	//
	// 	previos_data_update = time_us_64() / 1000;
	// }
}
