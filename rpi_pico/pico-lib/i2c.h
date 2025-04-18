#ifndef PICO_LIB_I2C_H
#define PICO_LIB_I2C_H

#include <vector>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C0 0
#define I2C1 1

class I2C {
public:
    I2C(uint8_t sda, uint8_t scl);
    void setBaudrate(unsigned long baudrate);
    void setPort(bool port);

    void begin ();
    void write(uint8_t addr, uint8_t * pointerSource, size_t size, bool stop);
    void read(uint8_t addr, uint8_t * pointerDest, size_t size, bool stop);

private:
    uint8_t sda;
    uint8_t scl;
    bool port = I2C0;
    unsigned long baudrate = 100 * 1000;

    int hardware_instance = 0;
    bool slave = false;

    bool reserved_addr(uint8_t addr) {
        return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
    }
};

#endif //PICO_LIB_I2C_H
