#pragma once

#include <Arduino.h>

// Reads the digital pin that reports whether the inverter currently sees
// mains/grid voltage. Shared between InverterRelay (cycling logic) and
// EnergyLog (the raw log's "mains online" status bit) so the pin is only
// known in one place.
class MainsSensor {
public:
    explicit MainsSensor(uint8_t pin) : _pin(pin) {}

    void begin() { pinMode(_pin, INPUT); }
    bool isOnline() const { return digitalRead(_pin) != 0; }

private:
    uint8_t _pin;
};
