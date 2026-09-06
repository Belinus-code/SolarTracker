#pragma once

#include <stdint.h>

// All user-tunable settings, persisted as a fixed-size binary blob at
// /config.bin on LittleFS. Field order/size below is the on-disk layout -
// don't reorder or resize fields without a migration, existing devices have
// a config.bin in this exact layout already.
class TrackerConfig {
public:
    static constexpr const char* kPath = "/config.bin";

    uint32_t savingCycleMs = 600000;         // Cycle to save Samples
    uint32_t sampleCycleMs = 1000;
    uint32_t inverterOffDurationMs = 10000;  // Duration after last online before relay
    uint32_t inverterCycleDurationMs = 300000; // Cycle Duration after first relay
    uint32_t inverterRelayDurationMs = 5000; // Duration of Relay Toggle
    uint32_t relayOverwriteDurationMs = 5000;

    bool cycleInverter = true;  // Cycle Inverter during Night (Always Cycle on Solar Power)
    bool loggingEnabled = false;
    bool samplingEnabled = false;

    // Reads /config.bin into this instance. Leaves defaults untouched and
    // returns false if the file is missing or the wrong size.
    bool load();

    // Writes this instance to /config.bin. Returns false if the file
    // couldn't be opened.
    bool save() const;

private:
    // Binary on-disk layout - matches CONFIG_FILE_SIZE (27 bytes) from the
    // original hand-packed format.
    struct __attribute__((packed)) Layout {
        uint32_t savingCycleMs;
        uint32_t sampleCycleMs;
        uint32_t inverterOffDurationMs;
        uint32_t inverterCycleDurationMs;
        uint32_t inverterRelayDurationMs;
        uint32_t relayOverwriteDurationMs;
        uint8_t cycleInverter;
        uint8_t loggingEnabled;
        uint8_t samplingEnabled;
    };
    static_assert(sizeof(Layout) == 27, "on-disk config layout changed size");
};
