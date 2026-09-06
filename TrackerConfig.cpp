#include "TrackerConfig.h"

#include <LittleFS.h>

bool TrackerConfig::load() {
    File file = LittleFS.open(kPath, FILE_READ);
    if (!file) return false;
    if (file.size() != sizeof(Layout)) {
        file.close();
        return false;
    }

    Layout layout;
    file.read(reinterpret_cast<uint8_t*>(&layout), sizeof(layout));
    file.close();

    savingCycleMs = layout.savingCycleMs;
    sampleCycleMs = layout.sampleCycleMs;
    inverterOffDurationMs = layout.inverterOffDurationMs;
    inverterCycleDurationMs = layout.inverterCycleDurationMs;
    inverterRelayDurationMs = layout.inverterRelayDurationMs;
    relayOverwriteDurationMs = layout.relayOverwriteDurationMs;
    cycleInverter = layout.cycleInverter;
    loggingEnabled = layout.loggingEnabled;
    samplingEnabled = layout.samplingEnabled;
    return true;
}

bool TrackerConfig::save() const {
    File file = LittleFS.open(kPath, FILE_WRITE);
    if (!file) return false;

    Layout layout;
    layout.savingCycleMs = savingCycleMs;
    layout.sampleCycleMs = sampleCycleMs;
    layout.inverterOffDurationMs = inverterOffDurationMs;
    layout.inverterCycleDurationMs = inverterCycleDurationMs;
    layout.inverterRelayDurationMs = inverterRelayDurationMs;
    layout.relayOverwriteDurationMs = relayOverwriteDurationMs;
    layout.cycleInverter = cycleInverter;
    layout.loggingEnabled = loggingEnabled;
    layout.samplingEnabled = samplingEnabled;

    file.write(reinterpret_cast<const uint8_t*>(&layout), sizeof(layout));
    file.close();
    return true;
}
