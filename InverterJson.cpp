#include "InverterJson.h"

namespace {

const char* outputSourcePriorityName(AxpertOutputSourcePriority v) {
    switch (v) {
        case AxpertOutputSourcePriority::UtilityFirst: return "UtilityFirst";
        case AxpertOutputSourcePriority::SolarFirst: return "SolarFirst";
        case AxpertOutputSourcePriority::SbuFirst: return "SbuFirst";
    }
    return "Unknown";
}

const char* chargerSourcePriorityName(AxpertChargerSourcePriority v) {
    switch (v) {
        case AxpertChargerSourcePriority::UtilityFirst: return "UtilityFirst";
        case AxpertChargerSourcePriority::SolarFirst: return "SolarFirst";
        case AxpertChargerSourcePriority::SolarAndUtility: return "SolarAndUtility";
        case AxpertChargerSourcePriority::SolarOnly: return "SolarOnly";
    }
    return "Unknown";
}

const char* batteryTypeName(AxpertBatteryType v) {
    switch (v) {
        case AxpertBatteryType::Agm: return "Agm";
        case AxpertBatteryType::Flooded: return "Flooded";
        case AxpertBatteryType::User: return "User";
        case AxpertBatteryType::Pylontech: return "Pylontech";
        case AxpertBatteryType::Shinheung: return "Shinheung";
        case AxpertBatteryType::Weco: return "Weco";
        case AxpertBatteryType::Soltaro: return "Soltaro";
    }
    return "Unknown";
}

const char* inputVoltageRangeName(AxpertInputVoltageRange v) {
    switch (v) {
        case AxpertInputVoltageRange::Appliance: return "Appliance";
        case AxpertInputVoltageRange::Ups: return "Ups";
    }
    return "Unknown";
}

const char* outputModeName(AxpertOutputMode v) {
    switch (v) {
        case AxpertOutputMode::SingleMachine: return "SingleMachine";
        case AxpertOutputMode::Parallel: return "Parallel";
        case AxpertOutputMode::Phase1Of3: return "Phase1Of3";
        case AxpertOutputMode::Phase2Of3: return "Phase2Of3";
        case AxpertOutputMode::Phase3Of3: return "Phase3Of3";
    }
    return "Unknown";
}

const char* machineTypeName(AxpertMachineType v) {
    switch (v) {
        case AxpertMachineType::GridTie: return "GridTie";
        case AxpertMachineType::OffGrid: return "OffGrid";
        case AxpertMachineType::Hybrid: return "Hybrid";
    }
    return "Unknown";
}

const char* topologyName(AxpertTopology v) {
    switch (v) {
        case AxpertTopology::Transformerless: return "Transformerless";
        case AxpertTopology::Transformer: return "Transformer";
    }
    return "Unknown";
}

const char* pvOkConditionName(AxpertPvOkConditionForParallel v) {
    switch (v) {
        case AxpertPvOkConditionForParallel::AnyUnitConnected: return "AnyUnitConnected";
        case AxpertPvOkConditionForParallel::AllUnitsConnected: return "AllUnitsConnected";
    }
    return "Unknown";
}

const char* pvPowerBalanceName(AxpertPvPowerBalance v) {
    switch (v) {
        case AxpertPvPowerBalance::MaxCurrentIsMaxChargedCurrent: return "MaxCurrentIsMaxChargedCurrent";
        case AxpertPvPowerBalance::MaxPowerIsChargedPlusLoadPower: return "MaxPowerIsChargedPlusLoadPower";
    }
    return "Unknown";
}

const char* deviceModeName(AxpertDeviceMode v) {
    switch (v) {
        case AxpertDeviceMode::PowerOn: return "PowerOn";
        case AxpertDeviceMode::Standby: return "Standby";
        case AxpertDeviceMode::Line: return "Line";
        case AxpertDeviceMode::Battery: return "Battery";
        case AxpertDeviceMode::Fault: return "Fault";
        case AxpertDeviceMode::Shutdown: return "Shutdown";
        case AxpertDeviceMode::Charge: return "Charge";
        case AxpertDeviceMode::Bypass: return "Bypass";
        case AxpertDeviceMode::Eco: return "Eco";
        case AxpertDeviceMode::Unknown: return "Unknown";
    }
    return "Unknown";
}

const char* parallelBatteryStatusName(AxpertParallelBatteryStatus v) {
    switch (v) {
        case AxpertParallelBatteryStatus::Normal: return "Normal";
        case AxpertParallelBatteryStatus::Under: return "Under";
        case AxpertParallelBatteryStatus::Open: return "Open";
    }
    return "Unknown";
}

// Escapes the only characters that can realistically show up in the
// alphanumeric protocol fields this project deals with (serial numbers,
// model names) - not a general-purpose JSON string escaper.
String jsonString(const char* s) {
    String out = "\"";
    for (const char* p = s; *p; p++) {
        if (*p == '"' || *p == '\\') out += '\\';
        out += *p;
    }
    out += "\"";
    return out;
}

String fmtFloat(float v, uint8_t decimals = 2) {
    char buf[24];
    dtostrf(v, 0, decimals, buf);
    return String(buf);
}

} // namespace

namespace InverterJson {

String toJson(const ProtocolIdResponse& r) {
    return String("{\"protocolId\":") + r.protocolId + "}";
}

String toJson(const SerialNumberResponse& r) {
    String out = "{\"serial\":";
    out += jsonString(r.serial);
    out += ",\"length\":" + String(r.length) + "}";
    return out;
}

String toJson(const FirmwareVersionResponse& r) {
    return String("{\"seriesNumber\":") + r.seriesNumber + ",\"version\":" + r.version + "}";
}

String toJson(const RatingInfoResponse& r) {
    String out = "{";
    out += "\"acOutputRatingVoltage\":" + fmtFloat(r.acOutputRatingVoltage, 1) + ",";
    out += "\"acOutputRatingCurrent\":" + fmtFloat(r.acOutputRatingCurrent, 1) + ",";
    out += "\"acOutputRatingFrequency\":" + fmtFloat(r.acOutputRatingFrequency, 1) + ",";
    out += "\"acOutputRatingApparentPower\":" + String(r.acOutputRatingApparentPower) + ",";
    out += "\"acOutputRatingActivePower\":" + String(r.acOutputRatingActivePower) + ",";
    out += "\"batteryRatingVoltage\":" + fmtFloat(r.batteryRatingVoltage, 1) + ",";
    out += "\"batteryRechargeVoltage\":" + fmtFloat(r.batteryRechargeVoltage, 1) + ",";
    out += "\"batteryUnderVoltage\":" + fmtFloat(r.batteryUnderVoltage, 1) + ",";
    out += "\"batteryBulkVoltage\":" + fmtFloat(r.batteryBulkVoltage, 1) + ",";
    out += "\"batteryFloatVoltage\":" + fmtFloat(r.batteryFloatVoltage, 1) + ",";
    out += "\"batteryType\":\"" + String(batteryTypeName(r.batteryType)) + "\",";
    out += "\"maxAcChargingCurrent\":" + String(r.maxAcChargingCurrent) + ",";
    out += "\"maxChargingCurrent\":" + String(r.maxChargingCurrent) + ",";
    out += "\"inputVoltageRange\":\"" + String(inputVoltageRangeName(r.inputVoltageRange)) + "\",";
    out += "\"outputSourcePriority\":\"" + String(outputSourcePriorityName(r.outputSourcePriority)) + "\",";
    out += "\"chargerSourcePriority\":\"" + String(chargerSourcePriorityName(r.chargerSourcePriority)) + "\",";
    out += "\"parallelMaxNumber\":" + String(r.parallelMaxNumber) + ",";
    out += "\"machineType\":\"" + String(machineTypeName(r.machineType)) + "\",";
    out += "\"topology\":\"" + String(topologyName(r.topology)) + "\",";
    out += "\"outputMode\":\"" + String(outputModeName(r.outputMode)) + "\",";
    out += "\"batteryRedischargeVoltage\":" + fmtFloat(r.batteryRedischargeVoltage, 1) + ",";
    out += "\"pvOkConditionForParallel\":\"" + String(pvOkConditionName(r.pvOkConditionForParallel)) + "\",";
    out += "\"pvPowerBalance\":\"" + String(pvPowerBalanceName(r.pvPowerBalance)) + "\",";
    out += "\"maxChargingTimeAtCvMinutes\":" + String(r.maxChargingTimeAtCvMinutes);
    out += "}";
    return out;
}

String toJson(const FlagStatusResponse& r) {
    String out = "{";
    out += "\"silenceBuzzerEnabled\":" + String(r.silenceBuzzerEnabled ? "true" : "false") + ",";
    out += "\"overloadBypassEnabled\":" + String(r.overloadBypassEnabled ? "true" : "false") + ",";
    out += "\"lcdEscapeToDefaultEnabled\":" + String(r.lcdEscapeToDefaultEnabled ? "true" : "false") + ",";
    out += "\"overloadRestartEnabled\":" + String(r.overloadRestartEnabled ? "true" : "false") + ",";
    out += "\"overTemperatureRestartEnabled\":" + String(r.overTemperatureRestartEnabled ? "true" : "false") + ",";
    out += "\"backlightOnEnabled\":" + String(r.backlightOnEnabled ? "true" : "false") + ",";
    out += "\"alarmOnPrimarySourceInterruptEnabled\":" + String(r.alarmOnPrimarySourceInterruptEnabled ? "true" : "false") + ",";
    out += "\"faultCodeRecordEnabled\":" + String(r.faultCodeRecordEnabled ? "true" : "false");
    out += "}";
    return out;
}

String toJson(const DeviceModeResponse& r) {
    return String("{\"mode\":\"") + deviceModeName(r.mode) + "\"}";
}

String toJson(const WarningStatusResponse& r) {
    String out = "{\"active\":[";
    bool first = true;
    auto add = [&](bool set, const char* name) {
        if (!set) return;
        if (!first) out += ",";
        out += "\"" + String(name) + "\"";
        first = false;
    };
    add(r.pvLoss, "PvLoss");
    add(r.inverterFault, "InverterFault");
    add(r.busOver, "BusOver");
    add(r.busUnder, "BusUnder");
    add(r.busSoftFail, "BusSoftFail");
    add(r.lineFail, "LineFail");
    add(r.outputShort, "OutputShort");
    add(r.inverterVoltageTooLow, "InverterVoltageTooLow");
    add(r.inverterVoltageTooHigh, "InverterVoltageTooHigh");
    add(r.overTemperature, "OverTemperature");
    add(r.fanLocked, "FanLocked");
    add(r.batteryVoltageHigh, "BatteryVoltageHigh");
    add(r.batteryLowAlarm, "BatteryLowAlarm");
    add(r.batteryUnderShutdown, "BatteryUnderShutdown");
    add(r.batteryDerating, "BatteryDerating");
    add(r.overLoad, "OverLoad");
    add(r.eepromFault, "EepromFault");
    add(r.inverterOverCurrent, "InverterOverCurrent");
    add(r.inverterSoftFail, "InverterSoftFail");
    add(r.selfTestFail, "SelfTestFail");
    add(r.outputDcVoltageOver, "OutputDcVoltageOver");
    add(r.batteryOpen, "BatteryOpen");
    add(r.currentSensorFail, "CurrentSensorFail");
    add(r.pvVoltageHigh, "PvVoltageHigh");
    add(r.pvOverCurrent, "PvOverCurrent");
    add(r.dcDcOverCurrent, "DcDcOverCurrent");
    out += "],\"hasAnyWarning\":" + String(r.hasAnyWarning() ? "true" : "false") + "}";
    return out;
}

String toJson(const DefaultSettingsResponse& r) {
    String out = "{";
    out += "\"acOutputVoltage\":" + fmtFloat(r.acOutputVoltage, 1) + ",";
    out += "\"acOutputFrequency\":" + fmtFloat(r.acOutputFrequency, 1) + ",";
    out += "\"maxAcChargingCurrent\":" + String(r.maxAcChargingCurrent) + ",";
    out += "\"batteryUnderVoltage\":" + fmtFloat(r.batteryUnderVoltage, 1) + ",";
    out += "\"chargingFloatVoltage\":" + fmtFloat(r.chargingFloatVoltage, 1) + ",";
    out += "\"chargingBulkVoltage\":" + fmtFloat(r.chargingBulkVoltage, 1) + ",";
    out += "\"batteryRechargeVoltage\":" + fmtFloat(r.batteryRechargeVoltage, 1) + ",";
    out += "\"maxChargingCurrent\":" + String(r.maxChargingCurrent) + ",";
    out += "\"inputVoltageRange\":\"" + String(inputVoltageRangeName(r.inputVoltageRange)) + "\",";
    out += "\"outputSourcePriority\":\"" + String(outputSourcePriorityName(r.outputSourcePriority)) + "\",";
    out += "\"chargerSourcePriority\":\"" + String(chargerSourcePriorityName(r.chargerSourcePriority)) + "\",";
    out += "\"batteryType\":\"" + String(batteryTypeName(r.batteryType)) + "\",";
    out += "\"buzzerEnabled\":" + String(r.buzzerEnabled ? "true" : "false") + ",";
    out += "\"powerSavingEnabled\":" + String(r.powerSavingEnabled ? "true" : "false") + ",";
    out += "\"overloadRestartEnabled\":" + String(r.overloadRestartEnabled ? "true" : "false") + ",";
    out += "\"overTemperatureRestartEnabled\":" + String(r.overTemperatureRestartEnabled ? "true" : "false") + ",";
    out += "\"lcdBacklightEnabled\":" + String(r.lcdBacklightEnabled ? "true" : "false") + ",";
    out += "\"alarmOnPrimarySourceInterruptEnabled\":" + String(r.alarmOnPrimarySourceInterruptEnabled ? "true" : "false") + ",";
    out += "\"faultCodeRecordEnabled\":" + String(r.faultCodeRecordEnabled ? "true" : "false") + ",";
    out += "\"overloadBypassEnabled\":" + String(r.overloadBypassEnabled ? "true" : "false") + ",";
    out += "\"lcdEscapeToDefaultEnabled\":" + String(r.lcdEscapeToDefaultEnabled ? "true" : "false") + ",";
    out += "\"outputMode\":\"" + String(outputModeName(r.outputMode)) + "\",";
    out += "\"batteryRedischargeVoltage\":" + fmtFloat(r.batteryRedischargeVoltage, 1) + ",";
    out += "\"pvOkConditionForParallel\":\"" + String(pvOkConditionName(r.pvOkConditionForParallel)) + "\",";
    out += "\"pvPowerBalance\":\"" + String(pvPowerBalanceName(r.pvPowerBalance)) + "\",";
    out += "\"maxChargingTimeAtCvMinutes\":" + String(r.maxChargingTimeAtCvMinutes);
    out += "}";
    return out;
}

String toJson(const ChargingCurrentOptionsResponse& r) {
    String out = "{\"values\":[";
    for (uint8_t i = 0; i < r.count; i++) {
        if (i > 0) out += ",";
        out += String(r.values[i]);
    }
    out += "]}";
    return out;
}

String toJson(const ModelNameResponse& r) {
    String out = "{\"name\":";
    out += jsonString(r.name);
    out += ",\"ratedOutputVa\":" + String(r.ratedOutputVa) + "}";
    return out;
}

String toJson(const GeneralModelNameResponse& r) {
    return String("{\"modelCode\":") + r.modelCode + "}";
}

String toJson(const BatteryEqualizationStatusResponse& r) {
    String out = "{";
    out += "\"enabled\":" + String(r.enabled ? "true" : "false") + ",";
    out += "\"equalizationTimeMinutes\":" + String(r.equalizationTimeMinutes) + ",";
    out += "\"equalizationPeriodDays\":" + String(r.equalizationPeriodDays) + ",";
    out += "\"equalizationMaxCurrent\":" + String(r.equalizationMaxCurrent) + ",";
    out += "\"equalizationVoltage\":" + fmtFloat(r.equalizationVoltage, 2) + ",";
    out += "\"equalizationOverTimeMinutes\":" + String(r.equalizationOverTimeMinutes) + ",";
    out += "\"active\":" + String(r.active ? "true" : "false") + ",";
    out += "\"equalizationElapseTimeHours\":" + String(r.equalizationElapseTimeHours);
    out += "}";
    return out;
}

String toJson(const ParallelInfoResponse& r) {
    if (!r.exists) return "{\"exists\":false}";

    String out = "{\"exists\":true,";
    out += "\"serialNumber\":";
    out += jsonString(r.serialNumber);
    out += ",\"workMode\":\"" + String(deviceModeName(r.workMode)) + "\",";
    out += "\"faultCode\":" + String((uint8_t)r.faultCode) + ",";
    out += "\"gridVoltage\":" + fmtFloat(r.gridVoltage, 1) + ",";
    out += "\"gridFrequency\":" + fmtFloat(r.gridFrequency, 2) + ",";
    out += "\"acOutputVoltage\":" + fmtFloat(r.acOutputVoltage, 1) + ",";
    out += "\"acOutputFrequency\":" + fmtFloat(r.acOutputFrequency, 2) + ",";
    out += "\"acOutputApparentPower\":" + String(r.acOutputApparentPower) + ",";
    out += "\"acOutputActivePower\":" + String(r.acOutputActivePower) + ",";
    out += "\"loadPercent\":" + String(r.loadPercent) + ",";
    out += "\"batteryVoltage\":" + fmtFloat(r.batteryVoltage, 1) + ",";
    out += "\"batteryChargingCurrent\":" + String(r.batteryChargingCurrent) + ",";
    out += "\"batteryCapacityPercent\":" + String(r.batteryCapacityPercent) + ",";
    out += "\"pvInputVoltage\":" + fmtFloat(r.pvInputVoltage, 1) + ",";
    out += "\"totalChargingCurrent\":" + String(r.totalChargingCurrent) + ",";
    out += "\"totalAcOutputApparentPower\":" + String(r.totalAcOutputApparentPower) + ",";
    out += "\"totalOutputActivePower\":" + String(r.totalOutputActivePower) + ",";
    out += "\"totalAcOutputPercent\":" + String(r.totalAcOutputPercent) + ",";
    out += "\"isSccOk\":" + String(r.isSccOk ? "true" : "false") + ",";
    out += "\"isAcCharging\":" + String(r.isAcCharging ? "true" : "false") + ",";
    out += "\"isSccCharging\":" + String(r.isSccCharging ? "true" : "false") + ",";
    out += "\"batteryStatus\":\"" + String(parallelBatteryStatusName(r.batteryStatus)) + "\",";
    out += "\"isLineLoss\":" + String(r.isLineLoss ? "true" : "false") + ",";
    out += "\"isLoadOn\":" + String(r.isLoadOn ? "true" : "false") + ",";
    out += "\"isConfigurationChanged\":" + String(r.isConfigurationChanged ? "true" : "false") + ",";
    out += "\"outputMode\":\"" + String(outputModeName(r.outputMode)) + "\",";
    out += "\"chargerSourcePriority\":\"" + String(chargerSourcePriorityName(r.chargerSourcePriority)) + "\",";
    out += "\"maxChargerCurrent\":" + String(r.maxChargerCurrent) + ",";
    out += "\"maxChargerRange\":" + String(r.maxChargerRange) + ",";
    out += "\"maxAcChargerCurrent\":" + String(r.maxAcChargerCurrent) + ",";
    out += "\"pvInputCurrent\":" + String(r.pvInputCurrent) + ",";
    out += "\"batteryDischargeCurrent\":" + String(r.batteryDischargeCurrent);
    out += "}";
    return out;
}

String toJson(const GeneralStatusResponse& r) {
    String out = "{";
    out += "\"gridVoltage\":" + fmtFloat(r.gridVoltage, 1) + ",";
    out += "\"gridFrequency\":" + fmtFloat(r.gridFrequency, 2) + ",";
    out += "\"acOutputVoltage\":" + fmtFloat(r.acOutputVoltage, 1) + ",";
    out += "\"acOutputFrequency\":" + fmtFloat(r.acOutputFrequency, 2) + ",";
    out += "\"acOutputApparentPower\":" + String(r.acOutputApparentPower) + ",";
    out += "\"acOutputActivePower\":" + String(r.acOutputActivePower) + ",";
    out += "\"outputLoadPercent\":" + String(r.outputLoadPercent) + ",";
    out += "\"busVoltage\":" + String(r.busVoltage) + ",";
    out += "\"batteryVoltage\":" + fmtFloat(r.batteryVoltage, 2) + ",";
    out += "\"batteryChargingCurrent\":" + String(r.batteryChargingCurrent) + ",";
    out += "\"batteryCapacityPercent\":" + String(r.batteryCapacityPercent) + ",";
    out += "\"inverterHeatSinkTemperature\":" + String(r.inverterHeatSinkTemperature) + ",";
    out += "\"pvInputCurrent\":" + fmtFloat(r.pvInputCurrent, 1) + ",";
    out += "\"pvInputVoltage\":" + fmtFloat(r.pvInputVoltage, 1) + ",";
    out += "\"batteryVoltageFromScc\":" + fmtFloat(r.batteryVoltageFromScc, 2) + ",";
    out += "\"batteryDischargeCurrent\":" + String(r.batteryDischargeCurrent) + ",";
    out += "\"isConfigurationChanged\":" + String(r.isConfigurationChanged ? "true" : "false") + ",";
    out += "\"isLoadOn\":" + String(r.isLoadOn ? "true" : "false") + ",";
    out += "\"isCharging\":" + String(r.isCharging ? "true" : "false") + ",";
    out += "\"isSCCCharging\":" + String(r.isSCCCharging ? "true" : "false") + ",";
    out += "\"isACCharging\":" + String(r.isACCharging ? "true" : "false") + ",";
    out += "\"pvChargingPower\":" + String(r.pvChargingPower);
    out += "}";
    return out;
}

} // namespace InverterJson
