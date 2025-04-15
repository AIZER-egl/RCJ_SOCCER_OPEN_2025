#include "../serial.h"

Serial::Serial(uint8_t port) {
    this -> port = port;
}

void Serial::attach(uint8_t TX, uint8_t RX) {
    this -> TX = TX;
    this -> RX = RX;
}

void Serial::begin(uint32_t baudrate) {
    this -> baudrate = baudrate;
    if ((this -> port) == 0) {
        uart_init(((uart_inst_t*)uart0_hw), baudrate);
        gpio_set_function(this->TX, GPIO_FUNC_UART);
        gpio_set_function(this->RX, GPIO_FUNC_UART);
    } else if ((this -> port) == 1) {
        uart_init(((uart_inst_t*)uart1_hw), baudrate);
        gpio_set_function(this->TX, GPIO_FUNC_UART);
        gpio_set_function(this->RX, GPIO_FUNC_UART);
    }
}

bool Serial::available() {
    if ((this -> port) == 0) {
        return uart_is_readable(((uart_inst_t*)uart0_hw));
    } else if ((this -> port) == 1) {
        return uart_is_readable(((uart_inst_t*)uart1_hw));
    }
    return false;
}
std::optional<char> Serial::read() {
    if ((this -> port) == 0) {
        if (uart_is_readable(((uart_inst_t*)uart0_hw))) {
            return uart_getc(((uart_inst_t*)uart0_hw));
        }
    } else if ((this -> port) == 1) {
        if (uart_is_readable(((uart_inst_t*)uart1_hw))) {
            return uart_getc(((uart_inst_t*)uart1_hw));
        }
    }
    return std::nullopt;
}

std::optional<std::string> Serial::readString() {
    std::string str = "";
    for (int i = 0;i < MAX_LOOP; i++) {
        if (this->available()) {
            char c = this->read().value();
            str += c;
        }
    }
    return str;
}

std::optional<std::string> Serial::readStringUntil(char terminator) {
    std::string str = "";
    for (int i = 0;i < MAX_LOOP; i++) {
        if (this->available()) {
            char c = this->read().value();
            if (c == terminator) {
                return str;
            } else {
                str += c;
            }
        }
    }
    return std::nullopt;
}


std::optional<unsigned int> Serial::write(char data) {
    if ((this -> port) == 0) {
        if (uart_is_writable(((uart_inst_t*)uart0_hw))) {
            uart_putc(((uart_inst_t*)uart0_hw), data);
            return 1;
        }
    } else if ((this -> port) == 1) {
        if (uart_is_writable(((uart_inst_t*)uart1_hw))) {
            uart_putc(((uart_inst_t*)uart1_hw), data);
            return 1;
        }
    }
    return std::nullopt;
}

