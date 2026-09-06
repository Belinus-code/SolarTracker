#pragma once

#include <DNSServer.h>
#include <LittleFS.h>
#include <WebServer.h>

#include "EnergyLog.h"
#include "InverterLink.h"
#include "InverterRelay.h"
#include "TrackerConfig.h"

// The captive-portal AP + HTTP API. Every route below matches the original
// sketch's paths and response shapes so the existing web UI (index.html)
// keeps working unmodified.
class WebPortal {
public:
    WebPortal(TrackerConfig& config, InverterLink& inverter, EnergyLog& energyLog, InverterRelay& relay);

    void begin(); // starts the AP, DNS, HTTP routes
    void update(); // call every loop()

private:
    TrackerConfig& _config;
    InverterLink& _inverter;
    EnergyLog& _energyLog;
    InverterRelay& _relay;

    DNSServer _dnsServer;
    WebServer _server;
    File _uploadFile;

    void setupRoutes();

    void handleUpdatePageGet();
    void handleUpdatePost();
    void handleUpdateUpload();
    void handleDeleteFileGet(); // ?path=/foo - removes a stray/superseded LittleFS file

    void handleRoot();
    void handleLive();
    void handleToggle();
    void handleRebootGet();
    void handleResetLogsGet();
    void handleApiCommandPost();
    void handleTimeSyncGet();
    void handleMaintenanceStartGet();
    void handleMaintenanceEndGet();
    void handleFsInfoGet();

    void handleConfigGet();
    void handleConfigUploadPost();
    void handleConfigUploadFile();

    void streamStoredFile(const char* path, const char* contentType);
    void serveStaticAsset(const String& path, const char* contentType);

    // Reads and plausibility-checks the "epoch" query arg shared by /time
    // and the /maintenance/* routes. Sends the error response itself and
    // returns false if missing or implausible.
    bool readClientEpoch(uint32_t& outEpoch);

    // ----- Inverter tab: read-only queries (/inverter/q...) -----
    // Each just runs the matching typed AxpertDevice query and returns
    // {"ok":true,"data":{...}} or {"ok":false,"error":"..."} - see
    // InverterJson.h for the "data" shape of each.
    void handleInvQpiGet();
    void handleInvQidGet();
    void handleInvQsidGet();
    void handleInvQvfwGet();
    void handleInvQvfw2Get();
    void handleInvQpiriGet();
    void handleInvQflagGet();
    void handleInvQmodGet();
    void handleInvQpiwsGet();
    void handleInvQdiGet();
    void handleInvQmchgcrGet();
    void handleInvQmuchgcrGet();
    void handleInvQmnGet();
    void handleInvQgmnGet();
    void handleInvQbeqiGet();
    void handleInvQpgsnGet();

    // ----- Inverter tab: settings (/inverter/p...) -----
    // Each returns {"ok":true} or {"ok":false,"error":"..."}.
    void handleInvSetFlagsGet();                     // PE/PD combined
    void handleInvResetDefaultsGet();                // PF - requires confirm=yes
    void handleInvSetMaxChargingCurrentGet();         // MNCHGC
    void handleInvSetMaxUtilityChargingCurrentGet();  // MUCHGC
    void handleInvSetOutputSourcePriorityGet();       // POP
    void handleInvSetBatteryVoltageBackToUtilityGet();// PBCV
    void handleInvSetChargerSourcePriorityGet();      // PCP
    void handleInvSetGridWorkingRangeGet();           // PGR
    void handleInvSetBatteryTypeGet();                // PBT
    void handleInvSetBatteryCutOffVoltageGet();       // PSDV
    void handleInvSetBatteryCvChargingVoltageGet();   // PCVV
    void handleInvSetBatteryFloatChargingVoltageGet();// PBFT
    void handleInvSetBatteryEqualizationEnabledGet(); // PBEQE
    void handleInvSetBatteryEqualizationTimeGet();    // PBEQT
    void handleInvSetBatteryEqualizationPeriodGet();  // PBEQP
    void handleInvSetBatteryEqualizationVoltageGet(); // PBEQV
    void handleInvSetBatteryEqualizationOverTimeGet();// PBEQOT
    void handleInvSetMaxChargingTimeAtCvGet();        // PCVT

    // Sends a 400 {"ok":false,...} and returns false if `name` isn't present
    // in the current request's query args.
    bool requireArg(const char* name);
};
