#pragma once

#include <Arduino.h>
#include <AxpertProtocol.h>

// Builds the JSON strings the Inverter tab's read-only queries return. Kept
// in its own module so WebPortal.cpp stays about HTTP routing, not
// protocol-struct-to-JSON mapping.
namespace InverterJson {
    String toJson(const ProtocolIdResponse& r);
    String toJson(const SerialNumberResponse& r);
    String toJson(const FirmwareVersionResponse& r);
    String toJson(const RatingInfoResponse& r);
    String toJson(const FlagStatusResponse& r);
    String toJson(const DeviceModeResponse& r);
    String toJson(const WarningStatusResponse& r);
    String toJson(const DefaultSettingsResponse& r);
    String toJson(const ChargingCurrentOptionsResponse& r);
    String toJson(const ModelNameResponse& r);
    String toJson(const GeneralModelNameResponse& r);
    String toJson(const BatteryEqualizationStatusResponse& r);
    String toJson(const ParallelInfoResponse& r);

    // Extra fields the QPIGS live feed (the Inverter tab) shows beyond what
    // WebPortal's /live already exposes for the dashboard.
    String toJson(const GeneralStatusResponse& r);
}
