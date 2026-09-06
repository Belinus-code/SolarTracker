#pragma once

#include <Arduino.h>
#include <AxpertProtocol.h>

// Latest QPIGS reading, trimmed down to the fields the web UI actually uses.
struct InverterReading {
    float solarVoltage = 0;
    float solarCurrent = 0;
    float batteryVoltage = 0;
    float batteryCurrent = 0; // charge current minus discharge current
    uint16_t outputPowerW = 0;
    uint8_t batteryPercent = 0;
};

// Talks to the Axpert inverter over a hardware UART using the AxpertProtocol
// library: queryGeneralStatus() for the periodic QPIGS poll, plus
// AxpertDevice's own sendRawCommand() escape hatch for the web UI's ad-hoc
// command console.
class InverterLink {
public:
    InverterLink(HardwareSerial& port, int8_t rxPin, int8_t txPin, uint16_t defaultTimeoutMs = 1000);

    void begin(); // configures the UART at the protocol's 2400 8N1

    // Queries QPIGS and updates lastReading()/lastStatus() on success.
    bool poll();

    const InverterReading& lastReading() const { return _last; }

    // The full QPIGS reply behind lastReading() - the Inverter tab's live
    // feed reads this instead of re-querying, reusing the same 2s poll()
    // cycle the dashboard already runs.
    const GeneralStatusResponse& lastStatus() const { return _lastStatus; }

    // Sends an arbitrary command (e.g. from the web console) and copies the
    // decoded reply text into outBuffer. Used for commands the typed
    // AxpertDevice API doesn't cover.
    bool sendRawCommand(const char* cmd, char* outBuffer, size_t outBufferCapacity, uint32_t timeoutMs);

    // Direct access to the underlying typed AxpertDevice API, for the
    // Inverter tab's on-demand queries/settings (QPI, QPIRI, PBEQE, ...)
    // that this class doesn't otherwise wrap one-by-one.
    AxpertDevice& device() { return _device; }

private:
    AxpertDevice _device;
    InverterReading _last;
    GeneralStatusResponse _lastStatus{};
};
