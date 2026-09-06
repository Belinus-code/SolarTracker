#include "InverterRelay.h"

InverterRelay::InverterRelay(uint8_t relayPin, const MainsSensor& mainsSensor, const TrackerConfig& config)
    : _relayPin(relayPin), _mainsSensor(mainsSensor), _config(config) {}

void InverterRelay::begin() {
    pinMode(_relayPin, OUTPUT);
}

void InverterRelay::setRelay(bool energized) {
    digitalWrite(_relayPin, energized ? HIGH : LOW);
}

void InverterRelay::toggle() {
    _overrideActive = true;
    _overrideStartMs = millis();
}

void InverterRelay::update() {
    uint32_t now = millis();

    if (_overrideActive) {
        if (now - _overrideStartMs < _config.relayOverwriteDurationMs) {
            setRelay(true);
            return;
        }
        _overrideActive = false;
        setRelay(false);
        return;
    }

    if (!_mainsSensor.isOnline()) {
        if (!_isOffline) {
            _isOffline = true;
            _offlineSinceMs = now;
            Serial.println("I0");
            return;
        }

        uint32_t offDuration = now - _offlineSinceMs;
        if (offDuration < _config.inverterOffDurationMs) {
            setRelay(false);
        } else if (offDuration < _config.inverterOffDurationMs + _config.inverterRelayDurationMs) {
            setRelay(true);
        } else if (_config.cycleInverter) {
            uint32_t cyclePos = (offDuration - (_config.inverterOffDurationMs + _config.inverterRelayDurationMs)) % _config.inverterCycleDurationMs;
            setRelay(cyclePos >= _config.inverterCycleDurationMs - _config.inverterRelayDurationMs);
        } else {
            setRelay(false);
        }
    } else {
        setRelay(false);
        if (_isOffline) {
            _isOffline = false;
            Serial.println("I1");
        }
    }
}
