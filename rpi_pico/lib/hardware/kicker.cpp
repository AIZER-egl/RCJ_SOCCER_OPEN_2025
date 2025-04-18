#include "kicker.h"

Kicker::Kicker() = default;

void Kicker::begin() {
	pinMode(KICKER, OUTPUT);
	previousKick = millis();
}

void Kicker::tick() {
	if (active && millis() - previousKick >= 500) {
		active = false;
		digitalWrite(KICKER, LOW);
	}
}

void Kicker::kick() {
	if (active) return;

	active = true;
	previousKick = millis();
	digitalWrite(KICKER, HIGH);
}