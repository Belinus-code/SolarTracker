#pragma once

#include <stdint.h>

#include "InverterLink.h"

// The inverter beeps continuously once the battery voltage drops too low
// (QPIWS's "Battery Low Alarm" bit), well before it actually cuts power -
// that only happens later if the voltage keeps dropping. The web UI's
// "Ignore Warning this time" button lets the user silence that beep for the
// current low-voltage episode; this class re-arms the buzzer automatically
// once the alarm has been continuously absent for a full logging cycle, so
// a *future* low-voltage episode beeps again instead of staying silenced
// forever.
//
// Note: on this hardware, the "Silence Buzzer" flag is inverted from what
// its name suggests - Disabled (false) is what actually stops the beeping,
// Enabled (true) is the normal/beeping state. Confirmed against the real
// device, not going by the protocol docs' naming.
class BuzzerIgnoreControl {
public:
    // Call once every `sampleCycleMs` tick (i.e. the same cadence
    // SolarTracker.ino already polls QPIGS at), only while sampling is
    // enabled - issues one QPIWS query to check the battery-low alarm, and
    // re-enables the buzzer if the user had silenced it and the alarm has
    // now been away for longer than `quietPeriodMs`.
    void update(InverterLink& inverter, uint32_t quietPeriodMs);

    // "Ignore Warning this time" button: silences the buzzer right now.
    // Returns false if the command itself failed (timeout/CRC) - state is
    // left unchanged so the button can just be pressed again.
    bool activateIgnore(InverterLink& inverter);

    // Whether the last successful QPIWS poll saw the battery-low alarm
    // active - the web UI shows the "Ignore Warning this time" button only
    // while this is true.
    bool batteryLowAlarmActive() const { return _batteryLowAlarmActive; }

    // Whether the user has silenced the buzzer and it hasn't been
    // automatically re-armed yet.
    bool ignoreActive() const { return _ignoreActive; }

private:
    bool _batteryLowAlarmActive = false;
    bool _ignoreActive = false;
    uint32_t _lastWarningSeenMs = 0;
};
