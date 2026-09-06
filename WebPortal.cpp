#include "WebPortal.h"

#include <WiFi.h>

#include "InverterJson.h"

namespace {

// Wraps every /inverter/q... query the same way: {"ok":true,"data":{...}}
// on success, {"ok":false,"error":"..."} (HTTP 500) on failure.
template <typename ResponseT>
void sendInverterQuery(WebServer& server, bool ok, const ResponseT& value) {
    if (!ok) {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"Query failed (timeout/CRC)\"}");
        return;
    }
    server.send(200, "application/json", "{\"ok\":true,\"data\":" + InverterJson::toJson(value) + "}");
}

// Wraps every /inverter/p... setting the same way: {"ok":true} on success,
// {"ok":false,"error":"..."} (HTTP 500) on failure/NAK.
void sendInverterSetResult(WebServer& server, bool ok) {
    if (ok) {
        server.send(200, "application/json", "{\"ok\":true}");
    } else {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"Command failed (timeout/CRC/NAK)\"}");
    }
}

} // namespace

WebPortal::WebPortal(TrackerConfig& config, InverterLink& inverter, EnergyLog& energyLog, InverterRelay& relay)
    : _config(config), _inverter(inverter), _energyLog(energyLog), _relay(relay), _server(80) {}

void WebPortal::begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Solar Tracker");
    IPAddress apIP = WiFi.softAPIP();
    // "*" (not a specific hostname) makes DNSServer answer every query with
    // our own IP, including a phone's connectivity-check domains
    // (connectivitycheck.gstatic.com, captive.apple.com, ...) - that's what
    // makes the OS recognize this as a captive portal instead of "no
    // internet", which is what was pushing phones onto mobile data instead
    // of actually using this WiFi to reach the dashboard.
    _dnsServer.start(53, "*", apIP);

    setupRoutes();
    _server.serveStatic("/", LittleFS, "/");
    _server.begin();
}

void WebPortal::update() {
    _dnsServer.processNextRequest();
    _server.handleClient();
}

void WebPortal::setupRoutes() {
    _server.on("/update", HTTP_GET, [this]() { handleUpdatePageGet(); });
    _server.on("/update", HTTP_POST, [this]() { handleUpdatePost(); }, [this]() { handleUpdateUpload(); });
    _server.on("/delete", HTTP_GET, [this]() { handleDeleteFileGet(); });

    _server.on("/", HTTP_GET, [this]() { handleRoot(); });
    _server.on("/log.bin", HTTP_GET, [this]() { streamStoredFile(EnergyLog::kLogPath, "application/octet-stream"); });
    _server.on("/ring.bin", HTTP_GET, [this]() { streamStoredFile(EnergyLog::kRingPath, "application/octet-stream"); });
    _server.on("/totals.bin", HTTP_GET, [this]() { streamStoredFile(EnergyLog::kTotalsPath, "application/octet-stream"); });
    _server.on("/timestamps.bin", HTTP_GET, [this]() { streamStoredFile(EnergyLog::kTimestampsPath, "application/octet-stream"); });

    _server.on("/live", HTTP_GET, [this]() { handleLive(); });
    _server.on("/toggle", HTTP_GET, [this]() { handleToggle(); });
    _server.on("/time", HTTP_GET, [this]() { handleTimeSyncGet(); });
    _server.on("/maintenance/start", HTTP_GET, [this]() { handleMaintenanceStartGet(); });
    _server.on("/maintenance/end", HTTP_GET, [this]() { handleMaintenanceEndGet(); });
    _server.on("/fs_info", HTTP_GET, [this]() { handleFsInfoGet(); });

    _server.on("/config.bin", HTTP_GET, [this]() { handleConfigGet(); });
    _server.on("/upload_config", HTTP_POST, [this]() { handleConfigUploadPost(); }, [this]() { handleConfigUploadFile(); });

    _server.on("/reset_logs", HTTP_GET, [this]() { handleResetLogsGet(); });
    _server.on("/reboot", HTTP_GET, [this]() { handleRebootGet(); });
    _server.on("/api/command", HTTP_POST, [this]() { handleApiCommandPost(); });

    // Served pre-gzipped (see WebServer::_streamFileCore()): opening a
    // ".gz"-suffixed file and passing the *uncompressed* content type here
    // makes streamFile() add the Content-Encoding: gzip header on its own,
    // so the URL/content-type the browser sees is unchanged - only the
    // bytes on LittleFS and on the wire are smaller.
    _server.on("/tailwind.js", HTTP_GET, [this]() { serveStaticAsset("/tailwind.js.gz", "application/javascript"); });
    _server.on("/lucide.js", HTTP_GET, [this]() { serveStaticAsset("/lucide.js.gz", "application/javascript"); });
    _server.on("/chart.js", HTTP_GET, [this]() { serveStaticAsset("/chart.js.gz", "application/javascript"); });

    _server.on("/inverter/qpi", HTTP_GET, [this]() { handleInvQpiGet(); });
    _server.on("/inverter/qid", HTTP_GET, [this]() { handleInvQidGet(); });
    _server.on("/inverter/qsid", HTTP_GET, [this]() { handleInvQsidGet(); });
    _server.on("/inverter/qvfw", HTTP_GET, [this]() { handleInvQvfwGet(); });
    _server.on("/inverter/qvfw2", HTTP_GET, [this]() { handleInvQvfw2Get(); });
    _server.on("/inverter/qpiri", HTTP_GET, [this]() { handleInvQpiriGet(); });
    _server.on("/inverter/qflag", HTTP_GET, [this]() { handleInvQflagGet(); });
    _server.on("/inverter/qmod", HTTP_GET, [this]() { handleInvQmodGet(); });
    _server.on("/inverter/qpiws", HTTP_GET, [this]() { handleInvQpiwsGet(); });
    _server.on("/inverter/qdi", HTTP_GET, [this]() { handleInvQdiGet(); });
    _server.on("/inverter/qmchgcr", HTTP_GET, [this]() { handleInvQmchgcrGet(); });
    _server.on("/inverter/qmuchgcr", HTTP_GET, [this]() { handleInvQmuchgcrGet(); });
    _server.on("/inverter/qmn", HTTP_GET, [this]() { handleInvQmnGet(); });
    _server.on("/inverter/qgmn", HTTP_GET, [this]() { handleInvQgmnGet(); });
    _server.on("/inverter/qbeqi", HTTP_GET, [this]() { handleInvQbeqiGet(); });
    _server.on("/inverter/qpgsn", HTTP_GET, [this]() { handleInvQpgsnGet(); });

    _server.on("/inverter/pe_pd", HTTP_GET, [this]() { handleInvSetFlagsGet(); });
    _server.on("/inverter/pf", HTTP_GET, [this]() { handleInvResetDefaultsGet(); });
    _server.on("/inverter/mnchgc", HTTP_GET, [this]() { handleInvSetMaxChargingCurrentGet(); });
    _server.on("/inverter/muchgc", HTTP_GET, [this]() { handleInvSetMaxUtilityChargingCurrentGet(); });
    _server.on("/inverter/pop", HTTP_GET, [this]() { handleInvSetOutputSourcePriorityGet(); });
    _server.on("/inverter/pbcv", HTTP_GET, [this]() { handleInvSetBatteryVoltageBackToUtilityGet(); });
    _server.on("/inverter/pcp", HTTP_GET, [this]() { handleInvSetChargerSourcePriorityGet(); });
    _server.on("/inverter/pgr", HTTP_GET, [this]() { handleInvSetGridWorkingRangeGet(); });
    _server.on("/inverter/pbt", HTTP_GET, [this]() { handleInvSetBatteryTypeGet(); });
    _server.on("/inverter/psdv", HTTP_GET, [this]() { handleInvSetBatteryCutOffVoltageGet(); });
    _server.on("/inverter/pcvv", HTTP_GET, [this]() { handleInvSetBatteryCvChargingVoltageGet(); });
    _server.on("/inverter/pbft", HTTP_GET, [this]() { handleInvSetBatteryFloatChargingVoltageGet(); });
    _server.on("/inverter/pbeqe", HTTP_GET, [this]() { handleInvSetBatteryEqualizationEnabledGet(); });
    _server.on("/inverter/pbeqt", HTTP_GET, [this]() { handleInvSetBatteryEqualizationTimeGet(); });
    _server.on("/inverter/pbeqp", HTTP_GET, [this]() { handleInvSetBatteryEqualizationPeriodGet(); });
    _server.on("/inverter/pbeqv", HTTP_GET, [this]() { handleInvSetBatteryEqualizationVoltageGet(); });
    _server.on("/inverter/pbeqot", HTTP_GET, [this]() { handleInvSetBatteryEqualizationOverTimeGet(); });
    _server.on("/inverter/pcvt", HTTP_GET, [this]() { handleInvSetMaxChargingTimeAtCvGet(); });
}

// ----- Generic LittleFS file upload (unrelated to config) -----

void WebPortal::handleUpdatePageGet() {
    // Two upload paths to the same /update endpoint (handleUpdateUpload()
    // doesn't care how the bytes were produced, so no backend change was
    // needed here): the "compressed" one gzips the file client-side via the
    // browser's own CompressionStream API and appends ".gz" to the name
    // before uploading; the plain one uploads whatever was selected
    // unmodified - for files you already compressed yourself, or that
    // shouldn't be compressed at all, or if the browser doesn't support
    // CompressionStream (Safari <16.4, older browsers).
    _server.send(200, "text/html", R"HTML(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LittleFS Upload</title>
<style>
body{font-family:sans-serif;padding:20px;max-width:480px;margin:0 auto;}
.box{border:1px solid #ccc;border-radius:8px;padding:16px;margin-bottom:20px;}
button,input[type=submit]{padding:8px 16px;margin-top:8px;}
#gzip-status{margin-top:10px;font-size:14px;}
#gzip-unsupported{color:#b00;display:none;}
</style>
</head>
<body>
<h2>LittleFS Datei Upload</h2>

<div class="box">
<h3>Komprimiert hochladen (.gz)</h3>
<p>Komprimiert die Datei im Browser (gzip) und h&auml;ngt .gz an den Dateinamen an, bevor sie hochgeladen wird.</p>
<input type="file" id="gzip-file">
<br><br>
<button id="gzip-btn" onclick="uploadCompressed()">Komprimiert hochladen</button>
<p id="gzip-unsupported">Dieser Browser unterst&uuml;tzt keine Client-Kompression. Bitte die Datei manuell komprimieren (z.B. mit gzip) und &uuml;ber "Unkomprimiert hochladen" mit .gz-Endung hochladen.</p>
<p id="gzip-status"></p>
</div>

<div class="box">
<h3>Unkomprimiert hochladen</h3>
<p>L&auml;dt die Datei 1:1 hoch, ohne &Auml;nderung - f&uuml;r bereits komprimierte Dateien oder Dateien, die nicht komprimiert werden sollen.</p>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="update"><br><br>
<input type="submit" value="Hochladen">
</form>
</div>

<script>
if (typeof CompressionStream === "undefined") {
    document.getElementById("gzip-unsupported").style.display = "block";
    document.getElementById("gzip-btn").disabled = true;
}

async function uploadCompressed() {
    const input = document.getElementById("gzip-file");
    const status = document.getElementById("gzip-status");
    if (!input.files.length) { status.textContent = "Bitte zuerst eine Datei auswaehlen."; return; }

    const file = input.files[0];
    status.textContent = "Komprimiere...";
    try {
        const compressedStream = file.stream().pipeThrough(new CompressionStream("gzip"));
        const compressedBlob = await new Response(compressedStream).blob();

        const formData = new FormData();
        formData.append("update", compressedBlob, file.name + ".gz");

        status.textContent = "Lade hoch...";
        const res = await fetch("/update", { method: "POST", body: formData });
        const text = await res.text();
        status.textContent = res.ok
            ? ("Erfolg: " + file.name + ".gz (" + compressedBlob.size + " von " + file.size + " Bytes)")
            : ("Fehler: " + text);
    } catch (err) {
        status.textContent = "Fehler: " + err;
    }
}
</script>
</body>
</html>
)HTML");
}

void WebPortal::handleUpdatePost() {
    _server.send(200, "text/plain", "Upload erfolgreich!");
}

void WebPortal::handleDeleteFileGet() {
    if (!_server.hasArg("path")) {
        _server.send(400, "text/plain", "path fehlt");
        return;
    }
    String path = _server.arg("path");
    if (!path.startsWith("/")) path = "/" + path;

    if (!LittleFS.exists(path)) {
        _server.send(404, "text/plain", path + " existiert nicht.");
        return;
    }
    if (LittleFS.remove(path)) {
        _server.send(200, "text/plain", path + " geloescht.");
    } else {
        _server.send(500, "text/plain", "Loeschen von " + path + " fehlgeschlagen.");
    }
}

void WebPortal::handleUpdateUpload() {
    HTTPUpload& upload = _server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        String filename = upload.filename;
        if (!filename.startsWith("/")) filename = "/" + filename;

        Serial.printf("Upload startet: %s\n", filename.c_str());
        _uploadFile = LittleFS.open(filename, "w");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (_uploadFile) _uploadFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (_uploadFile) {
            _uploadFile.close();
            Serial.printf("Upload beendet! Groesse: %u Bytes\n", upload.totalSize);
        }
    }
}

// ----- Dashboard / data endpoints -----

void WebPortal::handleRoot() {
    // Served pre-gzipped - see the /tailwind.js etc. routes below for why
    // this doesn't need its own Content-Encoding handling.
    File file = LittleFS.open("/index.html.gz", "r");
    if (!file) {
        _server.send(404, "text/plain", "index.html.gz nicht gefunden!");
        return;
    }
    _server.streamFile(file, "text/html");
    file.close();
}

void WebPortal::streamStoredFile(const char* path, const char* contentType) {
    if (!LittleFS.exists(path)) {
        Serial.printf("%s gibs nicht\n", path);
        _server.send(404, "text/plain", String(path) + " existiert noch nicht.");
        return;
    }
    File file = LittleFS.open(path, "r");
    _server.streamFile(file, contentType);
    file.close();
}

void WebPortal::handleLive() {
    const InverterReading& reading = _inverter.lastReading();
    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"sol_v\":%.2f,\"sol_a\":%.2f,\"bat_v\":%.2f,\"bat_a\":%.2f,\"out_pow\":%d,\"bat_per\":%d",
             reading.solarVoltage, reading.solarCurrent, reading.batteryVoltage, reading.batteryCurrent,
             reading.outputPowerW, reading.batteryPercent);

    // The full QPIGS reply behind the trimmed fields above - the Inverter
    // tab's live feed reads "full" instead of triggering its own QPIGS
    // query, reusing this same 2s poll() cycle.
    String out = buf;
    out += ",\"full\":" + InverterJson::toJson(_inverter.lastStatus()) + "}";
    _server.send(200, "application/json", out);
}

void WebPortal::handleToggle() {
    _relay.toggle();
    _server.send(200, "text/plain", "Inverter Toggle aktiviert!");
}

bool WebPortal::readClientEpoch(uint32_t& outEpoch) {
    // The device has neither an RTC nor internet access - a connecting
    // browser's local clock (which needs no internet either, it's just the
    // OS's own clock) is the only source of real time it ever gets.
    if (!_server.hasArg("epoch")) {
        _server.send(400, "text/plain", "epoch fehlt");
        return false;
    }

    long epoch = _server.arg("epoch").toInt();
    constexpr long kMinPlausibleEpoch = 1735689600L; // 2025-01-01, guards against an unset/wrong client clock
    if (epoch < kMinPlausibleEpoch) {
        _server.send(400, "text/plain", "Uhrzeit unplausibel");
        return false;
    }

    outEpoch = (uint32_t)epoch;
    return true;
}

void WebPortal::handleTimeSyncGet() {
    // See EnergyLog::noteClientTime() for the once-per-24h checkpoint logic.
    uint32_t epoch;
    if (!readClientEpoch(epoch)) return;

    _energyLog.noteClientTime(epoch, _config.loggingEnabled);
    _server.send(200, "text/plain", "OK");
}

void WebPortal::handleMaintenanceStartGet() {
    // Pauses log.bin persistence for a planned interruption (reflash,
    // power-off, ...): backdates a timestamp to the last sample already
    // written (see EnergyLog::startMaintenance()), then disables logging.
    // Pairs with /maintenance/end once the device is back.
    uint32_t epoch;
    if (!readClientEpoch(epoch)) return;

    _energyLog.startMaintenance(epoch);
    _config.loggingEnabled = false;
    _config.save();
    _server.send(200, "text/plain", "Wartungsmodus aktiv - Speichern pausiert.");
}

void WebPortal::handleMaintenanceEndGet() {
    // Re-enables logging and anchors the next sample to now (see
    // EnergyLog::endMaintenance()), so the gap since /maintenance/start is
    // bracketed by two back-to-back timestamps.bin entries instead of being
    // silently spread across it by the usual fixed-interval assumption.
    uint32_t epoch;
    if (!readClientEpoch(epoch)) return;

    _config.loggingEnabled = true;
    _config.save();
    _energyLog.endMaintenance(epoch);
    _server.send(200, "text/plain", "Wartungsmodus beendet - Speichern aktiv.");
}

void WebPortal::handleFsInfoGet() {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"total\":%lu,\"used\":%lu}",
             (unsigned long)LittleFS.totalBytes(), (unsigned long)LittleFS.usedBytes());
    _server.send(200, "application/json", buf);
}

void WebPortal::handleRebootGet() {
    _server.send(200, "text/plain", "ESP32 startet neu");
    delay(1000);
    ESP.restart();
}

void WebPortal::handleResetLogsGet() {
    _energyLog.resetAll();
    _server.send(200, "text/plain", "Alle Logs und Statistiken wurden erfolgreich gelöscht!");
}

void WebPortal::handleApiCommandPost() {
    if (!_server.hasArg("cmd")) {
        _server.send(400, "text/plain", "Kein Befehl angegeben!");
        return;
    }

    String cmd = _server.arg("cmd");
    unsigned long timeoutMs = 3000;
    if (_server.hasArg("timeout")) timeoutMs = _server.arg("timeout").toInt();

    char outBuffer[256];
    outBuffer[0] = '\0';
    bool success = _inverter.sendRawCommand(cmd.c_str(), outBuffer, sizeof(outBuffer), timeoutMs);

    if (success) {
        _server.send(200, "text/plain", String(outBuffer));
    } else {
        _server.send(500, "text/plain", "Fehler: Timeout, CRC-Fehler oder fehlerhafte Antwort!");
    }
}

// ----- Config download/upload -----

void WebPortal::handleConfigGet() {
    _config.save();
    if (!LittleFS.exists(TrackerConfig::kPath)) {
        _server.send(404, "text/plain", "config.bin existiert nicht.");
        return;
    }
    File file = LittleFS.open(TrackerConfig::kPath, "r");
    _server.streamFile(file, "application/octet-stream");
    file.close();
}

void WebPortal::handleConfigUploadPost() {
    if (!_config.load()) {
        _server.send(500, "text/plain", "Fehler beim Anwenden der Config!");
    } else {
        _server.send(200, "text/plain", "Config hochgeladen und angewendet!");
    }
}

void WebPortal::handleConfigUploadFile() {
    HTTPUpload& upload = _server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        _uploadFile = LittleFS.open(TrackerConfig::kPath, "w");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (_uploadFile) _uploadFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (_uploadFile) _uploadFile.close();
    }
}

// ----- Static JS assets bundled on LittleFS -----

void WebPortal::serveStaticAsset(const String& path, const char* contentType) {
    Serial.printf("\n[WEB] >>> Anfrage empfangen fuer: %s\n", path.c_str());

    if (!LittleFS.exists(path)) {
        Serial.printf("[WEB] !!! FEHLER: %s existiert nicht auf dem LittleFS!\n", path.c_str());
        _server.send(404, "text/plain", "File Not Found");
        return;
    }

    File file = LittleFS.open(path, "r");
    if (!file) {
        Serial.printf("[WEB] !!! FEHLER: Konnte %s nicht oeffnen (Speicherfehler?)\n", path.c_str());
        _server.send(500, "text/plain", "Internal Server Error");
        return;
    }

    size_t fileSize = file.size();
    Serial.printf("[WEB] --- Datei gefunden. Groesse: %u Bytes. Starte Transfer...\n", fileSize);

    _server.sendHeader("Cache-Control", "max-age=2592000, public");
    size_t sentBytes = _server.streamFile(file, contentType);
    file.close();

    if (sentBytes != fileSize) {
        Serial.printf("[WEB] !!! ABBRUCH: Nur %u von %u Bytes gesendet (Verbindung getrennt?)\n", sentBytes, fileSize);
    } else {
        Serial.printf("[WEB] <<< ERFOLG: %s komplett gesendet (%u Bytes)\n", path.c_str(), sentBytes);
    }
}

// ----- Inverter tab -----

bool WebPortal::requireArg(const char* name) {
    if (_server.hasArg(name)) return true;
    _server.send(400, "application/json", String("{\"ok\":false,\"error\":\"missing '") + name + "'\"}");
    return false;
}

void WebPortal::handleInvQpiGet() {
    ProtocolIdResponse r;
    sendInverterQuery(_server, _inverter.device().queryProtocolId(r), r);
}

void WebPortal::handleInvQidGet() {
    SerialNumberResponse r;
    sendInverterQuery(_server, _inverter.device().querySerialNumber(r), r);
}

void WebPortal::handleInvQsidGet() {
    SerialNumberResponse r;
    sendInverterQuery(_server, _inverter.device().querySerialNumberExtended(r), r);
}

void WebPortal::handleInvQvfwGet() {
    FirmwareVersionResponse r;
    sendInverterQuery(_server, _inverter.device().queryMainFirmwareVersion(r), r);
}

void WebPortal::handleInvQvfw2Get() {
    FirmwareVersionResponse r;
    sendInverterQuery(_server, _inverter.device().querySccFirmwareVersion(r), r);
}

void WebPortal::handleInvQpiriGet() {
    RatingInfoResponse r;
    sendInverterQuery(_server, _inverter.device().queryRatingInfo(r), r);
}

void WebPortal::handleInvQflagGet() {
    FlagStatusResponse r;
    sendInverterQuery(_server, _inverter.device().queryFlagStatus(r), r);
}

void WebPortal::handleInvQmodGet() {
    DeviceModeResponse r;
    sendInverterQuery(_server, _inverter.device().queryDeviceMode(r), r);
}

void WebPortal::handleInvQpiwsGet() {
    WarningStatusResponse r;
    sendInverterQuery(_server, _inverter.device().queryWarningStatus(r), r);
}

void WebPortal::handleInvQdiGet() {
    DefaultSettingsResponse r;
    sendInverterQuery(_server, _inverter.device().queryDefaultSettings(r), r);
}

void WebPortal::handleInvQmchgcrGet() {
    ChargingCurrentOptionsResponse r;
    sendInverterQuery(_server, _inverter.device().queryMaxChargingCurrentOptions(r), r);
}

void WebPortal::handleInvQmuchgcrGet() {
    ChargingCurrentOptionsResponse r;
    sendInverterQuery(_server, _inverter.device().queryMaxUtilityChargingCurrentOptions(r), r);
}

void WebPortal::handleInvQmnGet() {
    ModelNameResponse r;
    sendInverterQuery(_server, _inverter.device().queryModelName(r), r);
}

void WebPortal::handleInvQgmnGet() {
    GeneralModelNameResponse r;
    sendInverterQuery(_server, _inverter.device().queryGeneralModelName(r), r);
}

void WebPortal::handleInvQbeqiGet() {
    BatteryEqualizationStatusResponse r;
    sendInverterQuery(_server, _inverter.device().queryBatteryEqualizationStatus(r), r);
}

void WebPortal::handleInvQpgsnGet() {
    uint8_t unit = _server.hasArg("unit") ? (uint8_t)_server.arg("unit").toInt() : 1;
    ParallelInfoResponse r;
    sendInverterQuery(_server, _inverter.device().queryParallelInfo(unit, r), r);
}

void WebPortal::handleInvSetFlagsGet() {
    // Every flag defaults to Unchanged if its arg is missing, so the
    // frontend could send just the ones it wants to flip - it currently
    // always sends all 8, but SetFlagsRequest supports either.
    auto flagArg = [this](const char* name) -> AxpertFlagState {
        if (!_server.hasArg(name)) return AxpertFlagState::Unchanged;
        return _server.arg(name).toInt() != 0 ? AxpertFlagState::Enabled : AxpertFlagState::Disabled;
    };

    SetFlagsRequest req{};
    req.silenceBuzzer = flagArg("silenceBuzzer");
    req.overloadBypass = flagArg("overloadBypass");
    req.lcdEscapeToDefault = flagArg("lcdEscapeToDefault");
    req.overloadRestart = flagArg("overloadRestart");
    req.overTemperatureRestart = flagArg("overTemperatureRestart");
    req.backlightOn = flagArg("backlightOn");
    req.alarmOnPrimarySourceInterrupt = flagArg("alarmOnPrimarySourceInterrupt");
    req.faultCodeRecord = flagArg("faultCodeRecord");

    sendInverterSetResult(_server, _inverter.device().setFlags(req));
}

void WebPortal::handleInvResetDefaultsGet() {
    // The web UI's own confirmation modal is the primary safeguard - this
    // just makes sure PF can't be triggered by a bare/accidental GET (a
    // stray link, a browser prefetch, ...).
    if (_server.arg("confirm") != "yes") {
        _server.send(400, "application/json", "{\"ok\":false,\"error\":\"confirmation missing\"}");
        return;
    }
    sendInverterSetResult(_server, _inverter.device().resetToDefaults());
}

void WebPortal::handleInvSetMaxChargingCurrentGet() {
    if (!requireArg("amps")) return;
    SetMaxChargingCurrentRequest req{0, (uint16_t)_server.arg("amps").toInt()};
    sendInverterSetResult(_server, _inverter.device().setMaxChargingCurrent(req));
}

void WebPortal::handleInvSetMaxUtilityChargingCurrentGet() {
    if (!requireArg("amps")) return;
    SetMaxUtilityChargingCurrentRequest req{0, (uint16_t)_server.arg("amps").toInt()};
    sendInverterSetResult(_server, _inverter.device().setMaxUtilityChargingCurrent(req));
}

void WebPortal::handleInvSetOutputSourcePriorityGet() {
    if (!requireArg("value")) return;
    SetOutputSourcePriorityRequest req{(AxpertOutputSourcePriority)_server.arg("value").toInt()};
    sendInverterSetResult(_server, _inverter.device().setOutputSourcePriority(req));
}

void WebPortal::handleInvSetBatteryVoltageBackToUtilityGet() {
    if (!requireArg("voltage")) return;
    SetBatteryVoltageBackToUtilityRequest req{_server.arg("voltage").toFloat()};
    sendInverterSetResult(_server, _inverter.device().setBatteryVoltageBackToUtility(req));
}

void WebPortal::handleInvSetChargerSourcePriorityGet() {
    if (!requireArg("value")) return;
    SetChargerSourcePriorityRequest req{(AxpertChargerSourcePriority)_server.arg("value").toInt()};
    sendInverterSetResult(_server, _inverter.device().setChargerSourcePriority(req));
}

void WebPortal::handleInvSetGridWorkingRangeGet() {
    if (!requireArg("value")) return;
    SetGridWorkingRangeRequest req{(AxpertInputVoltageRange)_server.arg("value").toInt()};
    sendInverterSetResult(_server, _inverter.device().setGridWorkingRange(req));
}

void WebPortal::handleInvSetBatteryTypeGet() {
    if (!requireArg("value")) return;
    SetBatteryTypeRequest req{(AxpertBatteryType)_server.arg("value").toInt()};
    sendInverterSetResult(_server, _inverter.device().setBatteryType(req));
}

void WebPortal::handleInvSetBatteryCutOffVoltageGet() {
    if (!requireArg("voltage")) return;
    SetBatteryCutOffVoltageRequest req{_server.arg("voltage").toFloat()};
    sendInverterSetResult(_server, _inverter.device().setBatteryCutOffVoltage(req));
}

void WebPortal::handleInvSetBatteryCvChargingVoltageGet() {
    if (!requireArg("voltage")) return;
    SetBatteryCvChargingVoltageRequest req{_server.arg("voltage").toFloat()};
    sendInverterSetResult(_server, _inverter.device().setBatteryCvChargingVoltage(req));
}

void WebPortal::handleInvSetBatteryFloatChargingVoltageGet() {
    if (!requireArg("voltage")) return;
    SetBatteryFloatChargingVoltageRequest req{_server.arg("voltage").toFloat()};
    sendInverterSetResult(_server, _inverter.device().setBatteryFloatChargingVoltage(req));
}

void WebPortal::handleInvSetBatteryEqualizationEnabledGet() {
    if (!requireArg("enabled")) return;
    SetBatteryEqualizationEnabledRequest req{_server.arg("enabled").toInt() != 0};
    sendInverterSetResult(_server, _inverter.device().setBatteryEqualizationEnabled(req));
}

void WebPortal::handleInvSetBatteryEqualizationTimeGet() {
    if (!requireArg("minutes")) return;
    SetBatteryEqualizationTimeRequest req{(uint16_t)_server.arg("minutes").toInt()};
    sendInverterSetResult(_server, _inverter.device().setBatteryEqualizationTime(req));
}

void WebPortal::handleInvSetBatteryEqualizationPeriodGet() {
    if (!requireArg("days")) return;
    SetBatteryEqualizationPeriodRequest req{(uint16_t)_server.arg("days").toInt()};
    sendInverterSetResult(_server, _inverter.device().setBatteryEqualizationPeriod(req));
}

void WebPortal::handleInvSetBatteryEqualizationVoltageGet() {
    if (!requireArg("voltage")) return;
    SetBatteryEqualizationVoltageRequest req{_server.arg("voltage").toFloat()};
    sendInverterSetResult(_server, _inverter.device().setBatteryEqualizationVoltage(req));
}

void WebPortal::handleInvSetBatteryEqualizationOverTimeGet() {
    if (!requireArg("minutes")) return;
    SetBatteryEqualizationOverTimeRequest req{(uint16_t)_server.arg("minutes").toInt()};
    sendInverterSetResult(_server, _inverter.device().setBatteryEqualizationOverTime(req));
}

void WebPortal::handleInvSetMaxChargingTimeAtCvGet() {
    if (!requireArg("minutes")) return;
    SetMaxChargingTimeAtCvRequest req{(uint16_t)_server.arg("minutes").toInt()};
    sendInverterSetResult(_server, _inverter.device().setMaxChargingTimeAtCv(req));
}
