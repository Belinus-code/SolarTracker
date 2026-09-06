#pragma once

#include <stdint.h>

#include "InverterLink.h"

// Accumulates QPIGS samples and, once per savingCycleMs, flushes them into:
//  - a compressed 4-byte-per-period raw trace (/log.bin, append-only),
//  - a 144-slot ring buffer of 10-minute period summaries covering the last
//    24h (/ring.bin),
//  - running lifetime totals (/totals.bin).
class EnergyLog {
public:
    static constexpr uint16_t kSlotsPerDay = 144;
    static constexpr const char* kLogPath = "/log.bin";
    static constexpr const char* kRingPath = "/ring.bin";
    static constexpr const char* kTotalsPath = "/totals.bin";
    static constexpr const char* kTimestampsPath = "/timestamps.bin";

    // Creates the backing files on LittleFS if missing and loads any
    // existing ring buffer / totals into memory.
    void begin();

    // Feeds one QPIGS sample into the current 10-minute period's
    // accumulators. Call this every sampleCycleMs while sampling is enabled.
    void addSample(const InverterReading& reading, uint32_t sampleCycleMs);

    // Call every loop(); once savingCycleMs has elapsed since the last
    // flush, averages/compresses the accumulated period into the raw log,
    // rolls the ring buffer forward by one slot and updates totals.
    // `mainsOnline` is stamped into the raw log's status bit for this period.
    void update(bool mainsOnline, uint32_t savingCycleMs, bool loggingEnabled);

    // Wipes log.bin/totals.bin/ring.bin and all in-memory state. Used by the
    // web UI's "reset logs" action.
    void resetAll();

    // Minimum spacing between two timestamp checkpoints written into
    // timestamps.bin (see noteClientTime()).
    static constexpr uint32_t kTimeSyncIntervalMs = 24UL * 60 * 60 * 1000;

    // The device has neither an RTC nor internet access, so a connecting
    // browser's local clock is the only source of real time it ever gets.
    // Call this whenever one becomes known (see WebPortal's /time route,
    // fired once whenever a user opens the dashboard). If loggingEnabled is
    // false this is a no-op - nothing gets written, so nothing is tracked
    // either, meaning "when was it last saved" correctly stays "never"
    // until logging is actually on.
    //
    // Otherwise, at most once every kTimeSyncIntervalMs (or immediately if
    // this is the first call this boot), appends one (sampleIndex, epoch)
    // pair to timestamps.bin - see EnergyLog.cpp for how sampleIndex and
    // epoch are derived (backdated to the last written sample if one
    // exists this boot, otherwise predicted forward to the next one).
    void noteClientTime(uint32_t unixEpochSeconds, bool loggingEnabled);

    // "Wartung starten": backdates a checkpoint to the last sample actually
    // written this boot (same math as noteClientTime()'s case A), ignoring
    // the 24h cooldown - a no-op if nothing has been written yet this boot,
    // since there's nothing precise to anchor to. Caller is responsible for
    // actually disabling logging afterward (this only touches
    // timestamps.bin).
    void startMaintenance(uint32_t unixEpochSeconds);

    // "Wartung beenden": anchors the *next* sample (not written yet) to now
    // - same math as noteClientTime()'s case B - ignoring the 24h cooldown.
    // Caller is responsible for re-enabling logging (before or after this
    // call, both are fine - this only reads log.bin's current length).
    // Combined with startMaintenance(), this leaves two back-to-back
    // timestamps.bin entries bracketing the paused period.
    void endMaintenance(uint32_t unixEpochSeconds);

private:
    struct RingSlot {
        float solarWh;
        float batteryChargeWh;
        float batteryDischargeWh;
        float maxSolarAmp;
        float maxBatteryVolt;
        float batteryVoltAtSunrise;
    };

    struct TotalStats {
        double solarWh;
        double batteryChargeWh;
        double batteryDischargeWh;
        uint64_t uptimeMs;
    };

    // Current 10-minute period accumulators
    double _periodSolarAmpSum = 0;
    double _periodSolarVoltSum = 0;
    double _periodBatteryAmpSum = 0;
    double _periodBatteryVoltSum = 0;
    uint16_t _sampleCount = 0;

    float _periodSolarWh = 0;
    float _periodBatteryChargeWh = 0;
    float _periodBatteryDischargeWh = 0;
    float _periodMaxSolarAmp = 0;
    float _periodMaxBatteryVolt = 0;
    float _periodBatteryVoltAtSunrise = 35.0f;

    RingSlot _ring[kSlotsPerDay];
    uint8_t _ringIndex = 0;

    double _totalSolarWh = 0;
    double _totalBatteryChargeWh = 0;
    double _totalBatteryDischargeWh = 0;
    uint64_t _totalUptimeMs = 0;

    uint32_t _lastSaveMs = 0;

    // Set the moment a sample is actually appended to log.bin, so
    // noteClientTime() can tell "backdate to the last sample" (case A) from
    // "predict the next one" (case B) - see EnergyLog.cpp.
    bool _hasWrittenSampleThisBoot = false;
    uint32_t _lastWrittenSampleMs = 0;

    bool _hasTimeCheckpoint = false;
    uint32_t _lastTimeCheckpointMs = 0;

    void resetRingDefaults();
    void resetPeriodAccumulators();
    void ensureFilesExist();
    void loadRingBuffer();
    void loadTotals();
    void saveRingSlot(uint8_t index);
    void saveTotals();

    uint32_t currentLogRecordCount() const;
    void appendTimestampEntry(uint32_t sampleIndex, uint32_t epochSeconds);
    void purgeUnresolvedTimestamps();

    // Shared by noteClientTime()/startMaintenance()/endMaintenance() - see
    // EnergyLog.cpp for the case A/B writeup.
    bool backdateToLastSample(uint32_t unixEpochSeconds, uint32_t nowMs);
    void anchorNextSample(uint32_t unixEpochSeconds);
};
