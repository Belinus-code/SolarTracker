#include "InverterLink.h"

InverterLink::InverterLink(HardwareSerial& port, int8_t rxPin, int8_t txPin, uint16_t defaultTimeoutMs)
    : _device(port, rxPin, txPin, defaultTimeoutMs) {}

void InverterLink::begin() {
    _device.begin();
}

bool InverterLink::poll() {
    GeneralStatusResponse status;
    if (!_device.queryGeneralStatus(status)) return false;

    _lastStatus = status;

    _last.solarVoltage = status.pvInputVoltage;
    _last.solarCurrent = status.pvInputCurrent;
    _last.batteryVoltage = status.batteryVoltage;
    _last.batteryCurrent = (float)status.batteryChargingCurrent - (float)status.batteryDischargeCurrent;
    _last.batteryPercent = status.batteryCapacityPercent;
    _last.outputPowerW = status.acOutputActivePower;
    return true;
}

bool InverterLink::sendRawCommand(const char* cmd, char* outBuffer, size_t outBufferCapacity, uint32_t timeoutMs) {
    return _device.sendRawCommand(cmd, outBuffer, outBufferCapacity, /*appendCRC=*/true, (int32_t)timeoutMs);
}
