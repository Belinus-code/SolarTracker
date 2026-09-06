#include "EnergyLog.h"

#include <LittleFS.h>
#include <math.h>
#include <string.h>

void EnergyLog::begin() {
    resetRingDefaults();
    ensureFilesExist();
    loadRingBuffer();
    loadTotals();
    purgeUnresolvedTimestamps();
}

void EnergyLog::resetRingDefaults() {
    for (uint16_t i = 0; i < kSlotsPerDay; i++) {
        _ring[i] = RingSlot{0, 0, 0, 0, 0, 35.0f};
    }
    _ringIndex = 0;
}

void EnergyLog::resetPeriodAccumulators() {
    _periodSolarAmpSum = 0;
    _periodSolarVoltSum = 0;
    _periodBatteryAmpSum = 0;
    _periodBatteryVoltSum = 0;
    _sampleCount = 0;

    _periodSolarWh = 0;
    _periodBatteryChargeWh = 0;
    _periodBatteryDischargeWh = 0;
    _periodMaxSolarAmp = 0;
    _periodMaxBatteryVolt = 0;
    _periodBatteryVoltAtSunrise = 35.0f;
}

void EnergyLog::ensureFilesExist() {
    if (!LittleFS.exists(kRingPath)) {
        File file = LittleFS.open(kRingPath, "w");
        file.write(0);
        RingSlot empty = {0, 0, 0, 0, 0, 35.0f};
        for (uint16_t i = 0; i < kSlotsPerDay; i++) file.write((uint8_t*)&empty, sizeof(RingSlot));
        file.close();
    }

    if (!LittleFS.exists(kTotalsPath)) {
        File file = LittleFS.open(kTotalsPath, "w");
        TotalStats empty_stats = {0, 0, 0, 0};
        file.write((uint8_t*)&empty_stats, sizeof(TotalStats));
        file.close();
        Serial.println("totals.bin neu erstellt");
    }

    if (!LittleFS.exists(kLogPath)) {
        File file = LittleFS.open(kLogPath, "w");
        file.close();
        Serial.println("log.bin neu erstellt (0 Bytes)");
    }

    if (!LittleFS.exists(kTimestampsPath)) {
        File file = LittleFS.open(kTimestampsPath, "w");
        file.close();
    }
}

void EnergyLog::loadRingBuffer() {
    if (!LittleFS.exists(kRingPath)) return;
    File file = LittleFS.open(kRingPath, "r");

    _ringIndex = file.read();

    for (uint16_t i = 0; i < kSlotsPerDay; i++) {
        RingSlot slot;
        if (file.read((uint8_t*)&slot, sizeof(RingSlot)) == sizeof(RingSlot)) {
            _ring[i] = slot;
        }
    }
    file.close();
}

void EnergyLog::loadTotals() {
    if (!LittleFS.exists(kTotalsPath)) return;
    File file = LittleFS.open(kTotalsPath, "r");

    TotalStats ts;
    if (file.read((uint8_t*)&ts, sizeof(TotalStats)) == sizeof(TotalStats)) {
        _totalSolarWh = ts.solarWh;
        _totalBatteryChargeWh = ts.batteryChargeWh;
        _totalBatteryDischargeWh = ts.batteryDischargeWh;
        _totalUptimeMs = ts.uptimeMs;
    }
    file.close();
}

void EnergyLog::saveRingSlot(uint8_t index) {
    File file = LittleFS.open(kRingPath, "r+");
    if (!file) return;

    file.seek(0);
    file.write(index);

    uint32_t offset = 1 + (index * sizeof(RingSlot));
    file.seek(offset);
    file.write((uint8_t*)&_ring[index], sizeof(RingSlot));
    file.close();
}

void EnergyLog::saveTotals() {
    File file = LittleFS.open(kTotalsPath, "w");
    if (!file) return;

    TotalStats ts = {_totalSolarWh, _totalBatteryChargeWh, _totalBatteryDischargeWh, _totalUptimeMs};
    file.write((uint8_t*)&ts, sizeof(TotalStats));
    file.close();
}

void EnergyLog::addSample(const InverterReading& reading, uint32_t sampleCycleMs) {
    _periodSolarAmpSum += reading.solarCurrent;
    _periodSolarVoltSum += reading.solarVoltage;
    _periodBatteryAmpSum += reading.batteryCurrent;
    _periodBatteryVoltSum += reading.batteryVoltage;
    _sampleCount++;

    const float hoursPerSample = (float)sampleCycleMs / 3600000.0f;
    _periodSolarWh += reading.solarCurrent * reading.solarVoltage * hoursPerSample;
    float batteryPowerNow = reading.batteryCurrent * reading.batteryVoltage * hoursPerSample;
    if (reading.batteryCurrent >= 0) _periodBatteryChargeWh += batteryPowerNow;
    else _periodBatteryDischargeWh += fabsf(batteryPowerNow);

    _periodMaxSolarAmp = max(_periodMaxSolarAmp, reading.solarCurrent);
    _periodMaxBatteryVolt = max(_periodMaxBatteryVolt, reading.batteryVoltage);
    if (reading.solarVoltage < 5.0f) _periodBatteryVoltAtSunrise = min(_periodBatteryVoltAtSunrise, reading.batteryVoltage);

    _totalUptimeMs += sampleCycleMs;
}

void EnergyLog::update(bool mainsOnline, uint32_t savingCycleMs, bool loggingEnabled) {
    if (millis() - _lastSaveMs < savingCycleMs) return;
    _lastSaveMs += savingCycleMs;

    if (_sampleCount == 0) {
        // No samples came in this period (sampling was disabled the whole
        // time) - nothing to average, so skip the flush instead of dividing
        // by zero into the log/ring/totals.
        resetPeriodAccumulators();
        return;
    }

    float batteryAmpAvg = _periodBatteryAmpSum / _sampleCount;
    float solarAmpAvg = _periodSolarAmpSum / _sampleCount;
    float batteryVoltAvg = _periodBatteryVoltSum / _sampleCount;
    float solarVoltAvg = _periodSolarVoltSum / _sampleCount;

    batteryAmpAvg = constrain(batteryAmpAvg, -130.0f, 40.0f);
    solarVoltAvg = constrain(solarVoltAvg, 0.0f, 40.0f);
    solarAmpAvg = constrain(solarAmpAvg, 0.0f, 25.0f);
    batteryVoltAvg = constrain(batteryVoltAvg, 19.0f, 30.0f);

    uint8_t buffer[4];
    buffer[0] = (uint8_t)((batteryAmpAvg + 130.0f) * (255.0f / 170.0f));
    buffer[1] = (uint8_t)(solarVoltAvg * (255.0f / 40.0f));
    buffer[2] = (uint8_t)(solarAmpAvg * (255.0f / 25.0f));
    buffer[3] = (uint8_t)((batteryVoltAvg - 19.0f) * (127.0f / 11.0f)) & 0x7F;
    if (mainsOnline) buffer[3] |= 0x80;

    if (loggingEnabled) {
        File file = LittleFS.open(kLogPath, FILE_APPEND);
        if (!file) {
            // Matches the original sketch: bail out of the whole flush on a
            // write failure, leaving the ring buffer, totals and this
            // period's accumulators untouched until log.bin is writable
            // again (rather than silently dropping the period's data).
            Serial.println("Fehler beim Öffnen der log.bin!");
            return;
        }
        file.write(buffer, 4);
        file.close();

        // Lets noteClientTime() tell "a sample already exists this boot -
        // backdate a timestamp to it" from "nothing written yet - predict
        // the next one" (see EnergyLog.cpp's noteClientTime()). _lastSaveMs
        // was just set at the top of this call, so it's exactly this
        // write's flush time, not stale from an earlier skipped/failed one.
        _hasWrittenSampleThisBoot = true;
        _lastWrittenSampleMs = _lastSaveMs;
    }

    _ring[_ringIndex] = RingSlot{
        _periodSolarWh, _periodBatteryChargeWh, _periodBatteryDischargeWh,
        _periodMaxSolarAmp, _periodMaxBatteryVolt, _periodBatteryVoltAtSunrise
    };
    _ringIndex++;
    if (_ringIndex >= kSlotsPerDay) _ringIndex = 0;

    _totalSolarWh += _periodSolarWh;
    _totalBatteryChargeWh += _periodBatteryChargeWh;
    _totalBatteryDischargeWh += _periodBatteryDischargeWh;

    if (loggingEnabled) {
        uint8_t lastIndex = (_ringIndex == 0) ? (kSlotsPerDay - 1) : (_ringIndex - 1);
        saveRingSlot(lastIndex);
        saveTotals();
    }

    resetPeriodAccumulators();
}

void EnergyLog::resetAll() {
    if (LittleFS.exists(kLogPath)) LittleFS.remove(kLogPath);
    if (LittleFS.exists(kTotalsPath)) LittleFS.remove(kTotalsPath);

    File ringFile = LittleFS.open(kRingPath, "w");
    if (ringFile) {
        ringFile.write(0);
        RingSlot empty = {0, 0, 0, 0, 0, 35.0f};
        for (uint16_t i = 0; i < kSlotsPerDay; i++) ringFile.write((uint8_t*)&empty, sizeof(RingSlot));
        ringFile.close();
    }

    if (!LittleFS.exists(kTotalsPath)) {
        File totalsFile = LittleFS.open(kTotalsPath, "w");
        TotalStats empty_stats = {0, 0, 0, 0};
        totalsFile.write((uint8_t*)&empty_stats, sizeof(TotalStats));
        totalsFile.close();

        File logFile = LittleFS.open(kLogPath, "w");
        logFile.close();
    }

    if (LittleFS.exists(kTimestampsPath)) LittleFS.remove(kTimestampsPath);

    _totalSolarWh = 0;
    _totalBatteryChargeWh = 0;
    _totalBatteryDischargeWh = 0;
    _totalUptimeMs = 0;

    resetRingDefaults();
    resetPeriodAccumulators();

    _hasWrittenSampleThisBoot = false;
    _lastWrittenSampleMs = 0;

    _hasTimeCheckpoint = false;
    _lastTimeCheckpointMs = 0;
}

uint32_t EnergyLog::currentLogRecordCount() const {
    File file = LittleFS.open(kLogPath, "r");
    if (!file) return 0;
    uint32_t count = file.size() / 4;
    file.close();
    return count;
}

void EnergyLog::appendTimestampEntry(uint32_t sampleIndex, uint32_t epochSeconds) {
    uint8_t entry[8];
    memcpy(&entry[0], &sampleIndex, sizeof(sampleIndex));
    memcpy(&entry[4], &epochSeconds, sizeof(epochSeconds));

    File file = LittleFS.open(kTimestampsPath, FILE_APPEND);
    if (!file) return;
    file.write(entry, sizeof(entry));
    file.close();
}

void EnergyLog::purgeUnresolvedTimestamps() {
    // A timestamps.bin entry can point at a sample that doesn't exist in
    // log.bin yet (see noteClientTime()'s "case B", predicting the *next*
    // sample before it's written). That prediction only holds if the device
    // keeps running uninterrupted until that sample is actually flushed -
    // across a reboot, the accumulation/flush timing it was based on is
    // gone, and whatever eventually lands at that index is no longer
    // guaranteed to match the recorded epoch. So on every boot, drop any
    // entry whose sampleIndex isn't already covered by log.bin's current
    // length - entries that already resolved before the reboot are
    // unaffected and stay trustworthy.
    if (!LittleFS.exists(kTimestampsPath)) return;

    uint32_t recordCount = currentLogRecordCount();

    File in = LittleFS.open(kTimestampsPath, "r");
    if (!in) return;

    const char* tmpPath = "/timestamps.tmp";
    File out = LittleFS.open(tmpPath, "w");
    if (!out) {
        in.close();
        return;
    }

    uint8_t entry[8];
    while (in.read(entry, sizeof(entry)) == sizeof(entry)) {
        uint32_t sampleIndex;
        memcpy(&sampleIndex, &entry[0], sizeof(sampleIndex));
        if (sampleIndex < recordCount) {
            out.write(entry, sizeof(entry));
        }
    }
    in.close();
    out.close();

    LittleFS.remove(kTimestampsPath);
    LittleFS.rename(tmpPath, kTimestampsPath);
}

bool EnergyLog::backdateToLastSample(uint32_t unixEpochSeconds, uint32_t nowMs) {
    // Case A: at least one sample already landed in log.bin this boot -
    // backdate to exactly when that write happened, by subtracting the
    // millis() elapsed since then from the client's current time. Returns
    // false (writes nothing) if there's no such sample this boot to anchor
    // to precisely.
    if (!_hasWrittenSampleThisBoot) return false;

    uint32_t sampleIndex = currentLogRecordCount() - 1;
    uint32_t elapsedMs = nowMs - _lastWrittenSampleMs; // unsigned subtraction - safe across a millis() wrap
    uint32_t epochSeconds = unixEpochSeconds - (elapsedMs / 1000);
    appendTimestampEntry(sampleIndex, epochSeconds);
    return true;
}

void EnergyLog::anchorNextSample(uint32_t unixEpochSeconds) {
    // Case B: anchor the next sample, which doesn't exist yet - see
    // purgeUnresolvedTimestamps() for the reboot cleanup if it never
    // actually gets written.
    appendTimestampEntry(currentLogRecordCount(), unixEpochSeconds);
}

void EnergyLog::noteClientTime(uint32_t unixEpochSeconds, bool loggingEnabled) {
    if (!loggingEnabled) return; // nothing to anchor if nothing is being written anyway

    uint32_t now = millis();
    if (_hasTimeCheckpoint && (now - _lastTimeCheckpointMs) < kTimeSyncIntervalMs) return;

    _hasTimeCheckpoint = true;
    _lastTimeCheckpointMs = now;

    if (!backdateToLastSample(unixEpochSeconds, now)) {
        anchorNextSample(unixEpochSeconds);
    }
}

void EnergyLog::startMaintenance(uint32_t unixEpochSeconds) {
    _lastTimeCheckpointMs = millis();
    _hasTimeCheckpoint = true;
    backdateToLastSample(unixEpochSeconds, _lastTimeCheckpointMs); // no-op if nothing written yet this boot
}

void EnergyLog::endMaintenance(uint32_t unixEpochSeconds) {
    _lastTimeCheckpointMs = millis();
    _hasTimeCheckpoint = true;
    anchorNextSample(unixEpochSeconds);
}
