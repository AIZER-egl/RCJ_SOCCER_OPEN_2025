#ifndef KICKER_H
#define KICKER_H

#include "../../pico-lib/time.h"
#include "../../pico-lib/gpio.h"

#define KICKER 27

class Kicker {
public:
	Kicker();
	void tick();
	void begin();
	void kick();

private:
	bool active = false;
	unsigned long long previousKick{};
};



#endif //KICKER_H
