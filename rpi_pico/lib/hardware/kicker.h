#ifndef KICKER_H
#define KICKER_H

#include "../../pico-lib/time.h"
#include "../../pico-lib/gpio.h"
#include "../software/binarySerializationData.h"

#define KICKER 27

class Kicker {
public:
	Kicker();
	void tick();
	void begin(BinarySerializationData& data);
	void kick();

private:
	bool kicker_status = false;
	BinarySerializationData* dataPtr;
	unsigned long long previous_kick{};
};



#endif //KICKER_H
