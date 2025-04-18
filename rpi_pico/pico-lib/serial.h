#ifndef SERIAL_H
#define SERIAL_H

#include <optional>
#include <iostream>
#include <string>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#define UART_0 0
#define UART_1 1
#define MAX_LOOP 10000

#define USB_TX 0
#define USB_RX 1
#define USB_PORT 0

class Serial {
public:
    Serial (uint8_t port);
    void attach (uint8_t TX, uint8_t RX);
    void begin (uint32_t baudrate);
    bool available ();
    std::optional<char> read();
    std::optional<std::string> readString();
    std::optional<std::string> readStringUntil(char terminator);
    std::optional<unsigned int> write(char data);

    Serial& operator << (const std::string& message) {
        for (int i = 0; i < message.length(); i++) {
            write(message[i]);
        }
        return *this;
    }
private:
    uint8_t port;
    uint8_t TX;
    uint8_t RX;
    uint32_t baudrate;
};

#endif
