#include "BuzzerIgnoreControl.h"

void BuzzerIgnoreControl::update(InverterLink& inverter, uint32_t quietPeriodMs) {
    WarningStatusResponse warnings;
    if (!inverter.device().queryWarningStatus(warnings)) {
        // Keep the last known state rather than guessing on a failed poll -
        // matches how the rest of the codebase treats a timed-out query.
        return;
    }

    _batteryLowAlarmActive = warnings.batteryLowAlarm != 0;

    if (_batteryLowAlarmActive) {
        _lastWarningSeenMs = millis(); // still occurring - keep resetting the quiet timer
        return;
    }

    if (!_ignoreActive) return;

    if (millis() - _lastWarningSeenMs > quietPeriodMs) {
        SetFlagsRequest req{};
        req.silenceBuzzer = AxpertFlagState::Enabled; // beeps again on the next episode
        if (inverter.device().setFlags(req)) {
            _ignoreActive = false;
        }
        // If setFlags() failed, _ignoreActive stays true and this is retried
        // on the next update() tick.
    }
}

bool BuzzerIgnoreControl::activateIgnore(InverterLink& inverter) {
    SetFlagsRequest req{};
    req.silenceBuzzer = AxpertFlagState::Disabled; // silences the buzzer
    if (!inverter.device().setFlags(req)) return false;

    _ignoreActive = true;
    _lastWarningSeenMs = millis(); // baseline the quiet timer - overwritten by
                                    // update() on its next tick if the alarm
                                    // is in fact still active right now
    return true;
}
