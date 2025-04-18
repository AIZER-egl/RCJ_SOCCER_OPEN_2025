#include "../i2c.h"

I2C::I2C(uint8_t sda, uint8_t scl) {
    this -> sda = sda;
    this -> scl = scl;
}

void I2C::setBaudrate(unsigned long baudrate) {
    this -> baudrate = baudrate;
}

void I2C::setPort(bool port) {
    this -> port = port;
}

void I2C::begin() {
    i2c_init(port ? i2c1 : i2c0, baudrate);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

void I2C::write(uint8_t addr, uint8_t *pointerSource, size_t size, bool stop) {
    i2c_write_blocking(port ? i2c1 : i2c0, addr, pointerSource, size, stop);
}

void I2C::read(uint8_t addr, uint8_t *pointerDest, size_t size, bool stop) {
    i2c_read_blocking(port ? i2c1 : i2c0, addr, pointerDest, size, stop);
}

