#include "../time.h"

void delay(unsigned long ms) {
    sleep_ms(ms);
}

unsigned long long millis() {
    return time_us_64() / 1000;
}

unsigned long long micros() {
    return time_us_64();
}

void delayMicroseconds(unsigned int us) {
    sleep_us(us);
}

