#include <LittleFS.h>

#include "EnergyLog.h"
#include "InverterLink.h"
#include "InverterRelay.h"
#include "MainsSensor.h"
#include "TrackerConfig.h"
#include "WebPortal.h"

// ----- Pin Definitions -----

constexpr uint8_t kMainsSensePin = 19;
constexpr uint8_t kInverterRelayPin = 18;
constexpr int8_t kInverterRxPin = 16;
constexpr int8_t kInverterTxPin = 17;

// ----- Subsystems -----

TrackerConfig config;
MainsSensor mainsSensor(kMainsSensePin);
InverterLink inverter(Serial2, kInverterRxPin, kInverterTxPin);
EnergyLog energyLog;
InverterRelay relay(kInverterRelayPin, mainsSensor, config);
WebPortal webPortal(config, inverter, energyLog, relay);

uint32_t lastSampleMs = 0;

void setup() {
    Serial.begin(115200);

    mainsSensor.begin();
    relay.begin();
    inverter.begin();

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS konnte nicht gemountet werden");
        return;
    }

    if (config.load()) {
        Serial.println("Config Loaded!");
    } else if (config.save()) {
        Serial.println("Config Saved!");
    } else {
        Serial.println("SaveConfig Error!");
    }

    energyLog.begin();
    webPortal.begin();
}

void loop() {
    if (millis() - lastSampleMs >= config.sampleCycleMs) {
        lastSampleMs += config.sampleCycleMs;
        if (config.samplingEnabled) {
            if (inverter.poll()) {
                const InverterReading& reading = inverter.lastReading();
                Serial.printf("SA: %f, SV: %f, BA: %f, BV: %f\n",
                               reading.solarCurrent, reading.solarVoltage,
                               reading.batteryCurrent, reading.batteryVoltage);
            } else {
                Serial.println("Querry Failed!");
            }
            energyLog.addSample(inverter.lastReading(), config.sampleCycleMs);
        }
    }

    energyLog.update(mainsSensor.isOnline(), config.savingCycleMs, config.loggingEnabled);
    relay.update();
    webPortal.update();
}
