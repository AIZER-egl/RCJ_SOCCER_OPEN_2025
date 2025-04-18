#ifndef PICO_LIB_TIME_H
#define PICO_LIB_TIME_H

#define SECOND 1000.0

#include "pico/time.h"

void delay(unsigned long ms);
unsigned long long millis();
unsigned long long micros();
void delayMicroseconds(unsigned int us);

#endif //PICO_LIB_TIME_H
