#include "kicker.h"

Kicker::Kicker() : dataPtr(nullptr) {};

void Kicker::begin(BinarySerializationData& data) {
	dataPtr = &data;
	pinMode(KICKER, OUTPUT);
	previous_kick = millis();
}

void Kicker::tick() {
	if (!dataPtr) return;

	if (kicker_status && millis() - previous_kick >= 500) {
		kicker_status = false;
		dataPtr -> kicker_active = false;
		digitalWrite(KICKER, LOW);
	}
}


void Kicker::kick() {
	if (!dataPtr) return;
	if (kicker_status) return;

	kicker_status = true;
	dataPtr -> kicker_active = true;
	previous_kick = millis();
	digitalWrite(KICKER, HIGH);
}