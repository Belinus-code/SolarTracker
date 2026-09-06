#pragma once

#include <Arduino.h>

#include "MainsSensor.h"
#include "TrackerConfig.h"

// Drives the relay that power-cycles the inverter when mains has been gone
// for a while, plus a manual momentary-toggle override for the web UI's
// "toggle" button. All durations come from the shared TrackerConfig, so a
// live config reload takes effect immediately.
class InverterRelay {
public:
    InverterRelay(uint8_t relayPin, const MainsSensor& mainsSensor, const TrackerConfig& config);

    void begin();  // configures the relay pin as an output
    void update(); // call every loop()

    // Starts a momentary relay pulse, overriding the normal cycling logic
    // for relayOverwriteDurationMs. Used by the web UI's manual toggle.
    void toggle();

private:
    uint8_t _relayPin;
    const MainsSensor& _mainsSensor;
    const TrackerConfig& _config;

    bool _isOffline = false;
    uint32_t _offlineSinceMs = 0;

    bool _overrideActive = false;
    uint32_t _overrideStartMs = 0;

    void setRelay(bool energized);
};
