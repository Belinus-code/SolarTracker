#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

#define CONFIG_FILE_SIZE 26
// ----- Pin Definitions -----

int inverter_voltage_pin = 19;
int inverter_relay_pin = 18;

#define RXD2 16
#define TXD2 17


// ----- Timing Variables -----

uint32_t last_save = 0;
uint32_t saving_cycle = 600000;  // Cycle to save Samples

uint32_t last_sample = 0;
uint32_t sample_cycle = 1000;

uint32_t last_inverter_online = 0;
uint32_t inverter_off_duration = 10000;      // Duration after last online before relay
uint32_t inverter_cycle_duration = 300000;   // Cycle Duration after first relay
uint32_t inverter_relay_duration = 5000;     // Duration of Relay Toggle

uint32_t relay_overwrite_start = 0;
uint32_t relay_overwrite_duration = 5000; 

uint32_t last_unstable_bat = 0;


// ----- Sample Buffers -----

float solar_voltage = 0;
float solar_ampere = 0;
float battery_voltage = 0;
float battery_ampere = 0;
int output_power = 0;
int battery_percent = 0;

struct QPIGS_Data {
    // AC Netz-Eingang
    float gridVoltage;           // AC-Eingangsspannung in V
    float gridFrequency;         // AC-Eingangsfrequenz in Hz
    
    // AC Ausgang
    float outputVoltage;         // AC-Ausgangsspannung in V
    float outputFrequency;       // AC-Ausgangsfrequenz in Hz
    uint16_t outputApparentPower;// AC-Ausgangsscheinleistung in VA
    uint16_t outputActivePower;  // AC-Ausgangswirkleistung in W
    uint8_t outputLoadPercent;   // Ausgangslast in Prozent
    
    // Interne Werte & Batterie
    uint16_t busVoltage;         // Interne BUS-Spannung in V
    float batteryVoltage;        // Batteriespannung in V
    uint16_t batteryChargeCurrent; // Batterieladestrom in A
    uint8_t batteryCapacity;     // Geschätzte Batteriekapazität in %
    uint16_t inverterTemp;       // Temperatur des Kühlkörpers in °C
    
    // Solar (PV) Eingang
    float pvInputCurrent;        // PV-Eingangsstrom in A
    float pvInputVoltage;        // PV-Eingangsspannung in V
    float sccBatteryVoltage;     // Batteriespannung vom SCC in V
    uint16_t batteryDischargeCurrent; // Batterie-Entladestrom in A
    
    // Status Flags (aus Feld 17 extrahiert)
    bool isSbuPriorityActive;    // SBU-Priorität aktiv
    bool isConfigurationChanged; // Konfigurationsstatus geändert
    bool isBusVoltageStable;     // BUS-Spannung stabil
    bool isPvVoltageSufficient;  // PV-Spannung ausreichend für Ladung
    bool isLineMode;             // Inverter-Status (Netzbetrieb = true)
    bool isChargingOn;           // Lade-Status
    bool isLoadOn;               // Last angeschlossen
    bool hasWarning;             // Warnindikator
    
    uint16_t pvChargingPower;    // PV Ladeleistung in W
};

// ----- 10min Buffer -----

double period_solar_amp = 0;
double period_solar_volt = 0;
double period_battery_amp = 0;
double period_battery_volt = 0;

uint16_t sample_counter = 0;

float period_solar_wh = 0;
float period_battery_charge_wh = 0;
float period_battery_discharge_wh = 0;
float period_max_sol_amp = 0;
float period_max_bat_volt = 0;
float period_bat_volt_sunrise = 35;

// ----- Day Buffers -----

float ring_solar_wh[144];
float ring_battery_charge_wh[144];
float ring_battery_discharge_wh[144];
float ring_max_sol_amp[144];
float ring_max_bat_volt[144];
float ring_bat_volt_sunrise[144];
uint8_t ring_buffer_counter = 0;

struct RingSlot {
    float solar_wh;
    float bat_charge_wh;
    float bat_discharge_wh;
    float max_sol_amp;
    float max_bat_volt;
    float bat_volt_sunrise;
};

// ----- Total Stats -----

double total_solar_wh = 0;
double total_battery_charge = 0;
double total_battery_discharge = 0;
uint64_t total_uptime = 0;

struct TotalStats {
    double solar_wh;
    double bat_charge;
    double bat_discharge;
    uint64_t uptime;
};

// ----- Settings -----

bool cycle_inverter = true;           // Cycle Inverter during Night (Always Cycle on Solar Power)
bool relay_overwrite_active = false;

bool enable_logging = false;

static const uint16_t crc_table[16] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};
char responseBuffer[256];

// ----- Objects -----

DNSServer dnsServer;
WebServer server(80);
File uploadFile;

void setup() {
  Serial.begin(115200);
  Serial2.begin(2400, SERIAL_8N1, RXD2, TXD2);

  pinMode(inverter_voltage_pin, INPUT);
  pinMode(inverter_relay_pin, OUTPUT);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS konnte nicht gemountet werden");
    return;
  }

  int err = LoadConfig();
  if (err < 0)
  {
    Serial.print("LoadConfing Error: ");
    Serial.println(err);
    err = SaveConfig();
    if (err < 0)
    {
      Serial.print("SaveConfing Error: ");
      Serial.println(err); 
    }
    else Serial.println("Config Saved!");
  }
  else Serial.println("Config Loaded!");

  for(int i = 0; i < 144; i++)
  {
    ring_solar_wh[i] = 0.0;
    ring_battery_charge_wh[i] = 0.0;
    ring_battery_discharge_wh[i] = 0.0;
    ring_max_sol_amp[i] = 0.0;
    ring_max_bat_volt[i] = 0.0;
    ring_bat_volt_sunrise[i] = 35.0f;
  }

  if (!LittleFS.exists("/ring.bin"))
  {
    File file = LittleFS.open("/ring.bin", "w");
    file.write(0);
    RingSlot empty = {0, 0, 0, 0, 0, 35.0f};
    for(int i=0; i<144; i++) file.write((uint8_t*)&empty, sizeof(RingSlot));
    file.close();
  }
  if (!LittleFS.exists("/totals.bin"))
  {
    File file = LittleFS.open("/totals.bin", "w");
    TotalStats empty_stats = {0, 0, 0, 0};
    file.write((uint8_t*)&empty_stats, sizeof(TotalStats));
    file.close();
    Serial.println("totals.bin neu erstellt");
  }

  if (!LittleFS.exists("/log.bin"))
  {
    File file = LittleFS.open("/log.bin", "w");
    file.close();
    Serial.println("log.bin neu erstellt (0 Bytes)");
  }
  LoadRingBuffer();
  LoadTotals();

  WiFi.mode(WIFI_AP);
  WiFi.softAP("Solar Tracker");
  IPAddress apIP = WiFi.softAPIP();
  dnsServer.start(53, "solar.tracker", apIP);

  server.on("/update", HTTP_GET, []() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head>";
    html += "<body style='font-family:sans-serif; padding:20px;'><h2>LittleFS Datei Upload</h2>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='update'><br><br>";
    html += "<input type='submit' value='Hochladen'>";
    html += "</form></body></html>";
    
    server.send(200, "text/html", html);
  });

  server.on("/update", HTTP_POST, 
    []() { 
      server.send(200, "text/plain", "Upload erfolgreich!");
    },
    []() {
      HTTPUpload& upload = server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        String filename = upload.filename;
        if (!filename.startsWith("/")) {
          filename = "/" + filename;
        }
        
        Serial.printf("Upload startet: %s\n", filename.c_str());
        uploadFile = LittleFS.open(filename, "w");
      } 
      
      else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) {
          uploadFile.write(upload.buf, upload.currentSize);
        }
      } 
      
      else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
          uploadFile.close();
          Serial.printf("Upload beendet! Groesse: %u Bytes\n", upload.totalSize);
        }
      }
    }
  );

  server.on("/", HTTP_GET, []() {
    File file = LittleFS.open("/index.html", "r");
    if(!file) {
      server.send(404, "text/plain", "index.html nicht gefunden!");
      return;
    }
    server.streamFile(file, "text/html");
    file.close();
  });

  server.on("/log.bin", HTTP_GET, []() {
    if (!LittleFS.exists("/log.bin")) {
      Serial.println("log.bin gibs nicht");
      server.send(404, "text/plain", "log.bin existiert noch nicht.");
      return;
    }
    File file = LittleFS.open("/log.bin", "r");
    server.streamFile(file, "application/octet-stream");
    file.close();
  });

  server.on("/ring.bin", HTTP_GET, []() {
    if (!LittleFS.exists("/ring.bin")) {
      Serial.println("ring.bin gibs nicht");
      server.send(404, "text/plain", "ring.bin existiert noch nicht.");
      return;
    }
    File file = LittleFS.open("/ring.bin", "r");
    server.streamFile(file, "application/octet-stream");
    file.close();
  });

  server.on("/totals.bin", HTTP_GET, []() {
    if (!LittleFS.exists("/totals.bin")) {
      Serial.println("totals.bin gibs nicht");
      server.send(404, "text/plain", "totals.bin existiert noch nicht.");
      return;
    }
    File file = LittleFS.open("/totals.bin", "r");
    server.streamFile(file, "application/octet-stream");
    file.close();
  });

  server.on("/live", HTTP_GET, []() {
    char buf[200];
    snprintf(buf, sizeof(buf), 
             "{\"sol_v\":%.2f,\"sol_a\":%.2f,\"bat_v\":%.2f,\"bat_a\":%.2f,\"out_pow\":%d,\"bat_per\":%d}",
             solar_voltage, solar_ampere, battery_voltage, battery_ampere, 
             output_power, battery_percent);
    server.send(200, "application/json", buf);
  });

  server.on("/toggle", HTTP_GET, []() {
    ToggleInverter();
    server.send(200, "text/plain", "Inverter Toggle aktiviert!");
  });

  server.on("/config.bin", HTTP_GET, []() {
    SaveConfig();
    if (!LittleFS.exists("/config.bin")) {
      server.send(404, "text/plain", "config.bin existiert nicht.");
      return;
    }
    File file = LittleFS.open("/config.bin", "r");
    server.streamFile(file, "application/octet-stream");
    file.close();
  });

  server.on("/upload_config", HTTP_POST, 
    []() { 
      int err = LoadConfig();
      if(err < 0) {
        server.send(500, "text/plain", "Fehler beim Anwenden der Config!");
      } else {
        server.send(200, "text/plain", "Config hochgeladen und angewendet!");
      }
    },
    []() {
      HTTPUpload& upload = server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        uploadFile = LittleFS.open("/config.bin", "w");
      } 
      else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) {
          uploadFile.write(upload.buf, upload.currentSize);
        }
      } 
      else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
          uploadFile.close();
        }
      }
    }
  );

  server.on("/reset_logs", HTTP_GET, [](){

    if (LittleFS.exists("/log.bin")) LittleFS.remove("/log.bin");
    if (LittleFS.exists("/totals.bin")) LittleFS.remove("/totals.bin");

    File file = LittleFS.open("/ring.bin", "w");
    if (file) {
      file.write(0);
      RingSlot empty = {0, 0, 0, 0, 0, 35.0f};
      for(int i = 0; i < 144; i++) {
        file.write((uint8_t*)&empty, sizeof(RingSlot));
      }
      file.close();
    }
    if (!LittleFS.exists("/totals.bin"))
    {
      file = LittleFS.open("/totals.bin", "w");
      TotalStats empty_stats = {0, 0, 0, 0};
      file.write((uint8_t*)&empty_stats, sizeof(TotalStats));
      file.close();

      file = LittleFS.open("/log.bin", "w");
      file.close();
    }

    total_solar_wh = 0;
    total_battery_charge = 0;
    total_battery_discharge = 0;
    total_uptime = 0;

    ring_buffer_counter = 0;
    
    for(int i = 0; i < 144; i++) {
      ring_solar_wh[i] = 0.0f;
      ring_battery_charge_wh[i] = 0.0f;
      ring_battery_discharge_wh[i] = 0.0f;
      ring_max_sol_amp[i] = 0.0f;
      ring_max_bat_volt[i] = 0.0f;
      ring_bat_volt_sunrise[i] = 35.0f;
    }

    server.send(200, "text/plain", "Alle Logs und Statistiken wurden erfolgreich gelöscht!");
  });

  server.on("/reboot", HTTP_GET, []() {
    server.send(200, "text/plain", "ESP32 startet neu");
    delay(1000);
    ESP.restart();
  });

  auto serveAndLogFile = [](String path) {
    Serial.printf("\n[WEB] >>> Anfrage empfangen für: %s\n", path.c_str());
    
    if (!LittleFS.exists(path)) {
      Serial.printf("[WEB] !!! FEHLER: %s existiert nicht auf dem LittleFS!\n", path.c_str());
      server.send(404, "text/plain", "File Not Found");
      return;
    }

    File file = LittleFS.open(path, "r");
    if (!file) {
      Serial.printf("[WEB] !!! FEHLER: Konnte %s nicht oeffnen (Speicherfehler?)\n", path.c_str());
      server.send(500, "text/plain", "Internal Server Error");
      return;
    }

    size_t fileSize = file.size();
    Serial.printf("[WEB] --- Datei gefunden. Groesse: %u Bytes. Starte Transfer...\n", fileSize);
    
    server.sendHeader("Cache-Control", "max-age=2592000, public");
    // Datei senden (streamFile ist normalerweise blockierend, bis alles gesendet ist)
    size_t sentBytes = server.streamFile(file, "application/javascript");
    file.close();

    if (sentBytes != fileSize) {
      Serial.printf("[WEB] !!! ABBRUCH: Nur %u von %u Bytes gesendet (Verbindung getrennt?)\n", sentBytes, fileSize);
    } else {
      Serial.printf("[WEB] <<< ERFOLG: %s komplett gesendet (%u Bytes)\n", path.c_str(), sentBytes);
    }
  };

  server.on("/tailwind.js", HTTP_GET, [serveAndLogFile]() { serveAndLogFile("/tailwind.js"); });
  server.on("/lucide.js",   HTTP_GET, [serveAndLogFile]() { serveAndLogFile("/lucide.js"); });
  server.on("/chart.js",    HTTP_GET, [serveAndLogFile]() { serveAndLogFile("/chart.js"); });

  server.serveStatic("/", LittleFS, "/");
  server.begin();
}

void loop() {
  Sample();
  SaveSample();
  InverterControl();

  dnsServer.processNextRequest();
  server.handleClient();
}

void ToggleInverter()
{
  relay_overwrite_active = true;
  relay_overwrite_start = millis();
}

void Sample()
{
  if(millis() - last_sample >= sample_cycle)
  {
    last_sample += sample_cycle;
    // Replace with Serial Reading soon
    if(!sendCommandAndWait("QPIGS", responseBuffer, 1000))
    {
      return;
    }
    QPIGS_Data response;
    parseQPIGS(responseBuffer, response);
    float batteryVoltage;        // Batteriespannung in V
    uint16_t batteryChargeCurrent; // Batterieladestrom in A
    uint8_t batteryCapacity;
    float sol_amp_buf = response.pvInputCurrent;
    float sol_volt_buf = response.pvInputVoltage;
    float bat_amp_buf = (float)(response.batteryChargeCurrent) - (float)(response.batteryDischargeCurrent);
    float bat_volt_buf = response.batteryVoltage;

    Serial.printf("SA: %f, SV: %f, BA: %f, BV: %f\n",
              sol_amp_buf, sol_volt_buf,
              bat_amp_buf, bat_volt_buf);

    // Accumulating Logging Buffers

    period_solar_amp += sol_amp_buf;
    period_solar_volt += sol_volt_buf;
    period_battery_amp += bat_amp_buf;
    period_battery_volt += bat_volt_buf;
    sample_counter ++;

    // Conversion of Raw-Data

    solar_voltage = sol_volt_buf;
    battery_voltage = bat_volt_buf;

    solar_ampere = sol_amp_buf;
    battery_ampere = bat_amp_buf;
    battery_percent = response.batteryCapacity;
    output_power = response.outputActivePower;


    // Analysis for Period Logging

    const float hours_per_sample = (float)sample_cycle / 3600000.0f;
    period_solar_wh += solar_ampere * solar_voltage * hours_per_sample;
    float batt_p_now = battery_ampere * battery_voltage * hours_per_sample;
    if(battery_ampere >= 0)period_battery_charge_wh += batt_p_now;
    else period_battery_discharge_wh += abs(batt_p_now);

    period_max_sol_amp = max(period_max_sol_amp, solar_ampere);
    period_max_bat_volt = max(period_max_bat_volt, battery_voltage);
    if(solar_voltage < 5.0f)period_bat_volt_sunrise = min(period_bat_volt_sunrise, battery_voltage);
    total_uptime += sample_cycle;
  }
}

void SaveSample()
{
  if(millis() - last_save >= saving_cycle)
  {
    // ----- Calculate Average -----
    last_save += saving_cycle;
    float bat_amp = period_battery_amp / sample_counter;
    float sol_amp = period_solar_amp / sample_counter;
    float bat_volt = period_battery_volt / sample_counter;
    float sol_volt = period_solar_volt / sample_counter;

    bat_amp = constrain(bat_amp, -130.0f, 40.0f);
    sol_volt = constrain(sol_volt, 0.0f, 40.0f);
    sol_amp = constrain(sol_amp, 0.0f, 25.0f);
    bat_volt = constrain(bat_volt, 19.0f, 30.0f);
    
    // ----- Compression for Raw-Logging -----

    uint8_t buffer[4];
    buffer[0] = (uint8_t)((bat_amp + 130.0f) * (255.0f / 170.0f)); 
    buffer[1] = (uint8_t)(sol_volt * (255.0f / 40.0f));
    buffer[2] = (uint8_t)(sol_amp * (255.0f / 25.0f));
    buffer[3] = (uint8_t)((bat_volt - 19.0f) * (127.0f / 11.0f)) & 0x7F;
    if(digitalRead(inverter_voltage_pin))buffer[3] |= 0x80;

    if(enable_logging)
    { 
      File file = LittleFS.open("/log.bin", FILE_APPEND);
      if(!file)
      {
        Serial.println("Fehler beim Öffnen der log.bin!");
        return;
      }
      
      file.write(buffer, 4);
      file.close();
    }

    sample_counter = 0;
    period_battery_amp = 0;
    period_solar_amp = 0;
    period_battery_volt = 0;
    period_solar_volt = 0;

    // Analysis for 24-Hour Stats

    ring_solar_wh[ring_buffer_counter] = period_solar_wh;
    ring_battery_charge_wh[ring_buffer_counter] = period_battery_charge_wh;
    ring_battery_discharge_wh[ring_buffer_counter] = period_battery_discharge_wh;
    ring_max_sol_amp[ring_buffer_counter] = period_max_sol_amp;
    ring_max_bat_volt[ring_buffer_counter] = period_max_bat_volt;
    ring_bat_volt_sunrise[ring_buffer_counter] = period_bat_volt_sunrise;
    ring_buffer_counter++;
    if(ring_buffer_counter >= 144)ring_buffer_counter = 0;

    total_solar_wh += period_solar_wh;
    total_battery_charge += period_battery_charge_wh;
    total_battery_discharge += period_battery_discharge_wh;

    if(enable_logging)
    {
      uint8_t last_index = (ring_buffer_counter == 0) ? 143 : ring_buffer_counter - 1;
      SaveRingSlot(last_index);
      SaveTotals();
    }

    period_solar_wh = 0;
    period_battery_charge_wh = 0;
    period_battery_discharge_wh = 0;
    period_max_sol_amp = 0;
    period_max_bat_volt = 0;
    period_bat_volt_sunrise = 35;
  }
}

void InverterControl()
{
  static bool is_offline = false;
  unsigned long now = millis();

  if(relay_overwrite_active)
  {
    if (now - relay_overwrite_start < relay_overwrite_duration) 
    {
      digitalWrite(inverter_relay_pin, HIGH);
      return;
    }
    else 
    {
      relay_overwrite_active = false;
      digitalWrite(inverter_relay_pin, LOW);
    }
  }
  else if(!digitalRead(inverter_voltage_pin))
  {
    if(is_offline)
    {
      unsigned long off_time = now - last_inverter_online;
      if (off_time < inverter_off_duration)digitalWrite(inverter_relay_pin, LOW);
      else if(off_time < inverter_off_duration + inverter_relay_duration)digitalWrite(inverter_relay_pin, HIGH);
      else if(cycle_inverter)
      {
        if((off_time - (inverter_off_duration + inverter_relay_duration)) % inverter_cycle_duration < inverter_cycle_duration - inverter_relay_duration)digitalWrite(inverter_relay_pin, LOW);
        else digitalWrite(inverter_relay_pin, HIGH);
      }
      else digitalWrite(inverter_relay_pin, LOW);
    }
    else
    {
      is_offline = true;
      last_inverter_online = millis();
      Serial.println("I0");
    }
  }
  else
  {
    digitalWrite(inverter_relay_pin, LOW);
    if(is_offline)
    {
      is_offline = false;
      Serial.println("I1");
    }
  }
}

int SaveConfig()
{
  File config_file = LittleFS.open("/config.bin", FILE_WRITE);
  if(!config_file)return -1;
  int counter = 0;
  uint8_t buffer[CONFIG_FILE_SIZE];

  memcpy(&buffer[RunningIndex(sizeof(saving_cycle), counter)], &saving_cycle, sizeof(saving_cycle));
  memcpy(&buffer[RunningIndex(sizeof(sample_cycle), counter)], &sample_cycle, sizeof(sample_cycle));
  memcpy(&buffer[RunningIndex(sizeof(inverter_off_duration), counter)], &inverter_off_duration, sizeof(inverter_off_duration));
  memcpy(&buffer[RunningIndex(sizeof(inverter_cycle_duration), counter)], &inverter_cycle_duration, sizeof(inverter_cycle_duration));
  memcpy(&buffer[RunningIndex(sizeof(inverter_relay_duration), counter)], &inverter_relay_duration, sizeof(inverter_relay_duration));
  memcpy(&buffer[RunningIndex(sizeof(relay_overwrite_duration), counter)], &relay_overwrite_duration, sizeof(relay_overwrite_duration));

  memcpy(&buffer[RunningIndex(sizeof(cycle_inverter), counter)], &cycle_inverter, sizeof(cycle_inverter));
  memcpy(&buffer[RunningIndex(sizeof(enable_logging), counter)], &enable_logging, sizeof(enable_logging));

  config_file.write(buffer, CONFIG_FILE_SIZE);
  config_file.close();
  return 0;
}

int LoadConfig()
{
  File config_file = LittleFS.open("/config.bin", FILE_READ);
  if(!config_file)return -1;
  if(config_file.size() != CONFIG_FILE_SIZE)return -2;
  int counter = 0;
  uint8_t buffer[CONFIG_FILE_SIZE];
  config_file.read(buffer, CONFIG_FILE_SIZE);
  config_file.close();

  memcpy(&saving_cycle, &buffer[RunningIndex(sizeof(saving_cycle), counter)], sizeof(saving_cycle));
  memcpy(&sample_cycle, &buffer[RunningIndex(sizeof(sample_cycle), counter)], sizeof(sample_cycle));
  memcpy(&inverter_off_duration, &buffer[RunningIndex(sizeof(inverter_off_duration), counter)], sizeof(inverter_off_duration));
  memcpy(&inverter_cycle_duration, &buffer[RunningIndex(sizeof(inverter_cycle_duration), counter)], sizeof(inverter_cycle_duration));
  memcpy(&inverter_relay_duration, &buffer[RunningIndex(sizeof(inverter_relay_duration), counter)], sizeof(inverter_relay_duration));
  memcpy(&relay_overwrite_duration, &buffer[RunningIndex(sizeof(relay_overwrite_duration), counter)], sizeof(relay_overwrite_duration));

  memcpy(&cycle_inverter, &buffer[RunningIndex(sizeof(cycle_inverter), counter)], sizeof(cycle_inverter));
  memcpy(&enable_logging, &buffer[RunningIndex(sizeof(enable_logging), counter)], sizeof(enable_logging));

  return 0;
}

int RunningIndex(int size, int &counter)
{
  int old_index = counter;
  counter += size;
  return old_index;
}

void SaveRingSlot(uint8_t index)
{
  File file = LittleFS.open("/ring.bin", "r+");
  if (!file) return;

  file.seek(0);
  file.write(index);

  uint32_t offset = 1 + (index * sizeof(RingSlot));
  file.seek(offset);

  RingSlot slot =
  {
    ring_solar_wh[index], ring_battery_charge_wh[index], ring_battery_discharge_wh[index],
    ring_max_sol_amp[index], ring_max_bat_volt[index],
    ring_bat_volt_sunrise[index]
  };

  file.write((uint8_t*)&slot, sizeof(RingSlot));
  file.close();
}

void LoadRingBuffer()
{
  if (!LittleFS.exists("/ring.bin")) return;
  File file = LittleFS.open("/ring.bin", "r");
  
  ring_buffer_counter = file.read();
  
  for (int i = 0; i < 144; i++) {
    RingSlot slot;
    if (file.read((uint8_t*)&slot, sizeof(RingSlot)) == sizeof(RingSlot))
    {
      ring_solar_wh[i] = slot.solar_wh;
      ring_battery_charge_wh[i] = slot.bat_charge_wh;
      ring_battery_discharge_wh[i] = slot.bat_discharge_wh;
      ring_max_sol_amp[i] = slot.max_sol_amp;
      ring_max_bat_volt[i] = slot.max_bat_volt;
      ring_bat_volt_sunrise[i] = slot.bat_volt_sunrise;
    }
  }
  file.close();
}

void SaveTotals() {
    File file = LittleFS.open("/totals.bin", "w");
    if (!file) return;

    TotalStats ts = { total_solar_wh, total_battery_charge, total_battery_discharge, total_uptime };
    file.write((uint8_t*)&ts, sizeof(TotalStats));
    file.close();
}

void LoadTotals() {
    if (!LittleFS.exists("/totals.bin")) return;
    File file = LittleFS.open("/totals.bin", "r");
    
    TotalStats ts;
    if (file.read((uint8_t*)&ts, sizeof(TotalStats)) == sizeof(TotalStats)) {
        total_solar_wh = ts.solar_wh;
        total_battery_charge = ts.bat_charge;
        total_battery_discharge = ts.bat_discharge;
        total_uptime = ts.uptime;
    }
    file.close();
}

uint16_t calculate_inverter_crc(const char* data, int length) {
    uint16_t crc = 0x0000;
    for (int i = 0; i < length; i++) {
        uint8_t da = ((crc >> 8) >> 4);
        crc <<= 4;
        crc ^= crc_table[da ^ (data[i] >> 4)];
        da = ((crc >> 8) >> 4);
        crc <<= 4;
        crc ^= crc_table[da ^ (data[i] & 0x0F)];
    }
    
    uint8_t crc_low = crc & 0xFF;
    uint8_t crc_high = (crc >> 8) & 0xFF;
    
    if (crc_low == 0x28 || crc_low == 0x0D || crc_low == 0x0A) crc_low++;
    if (crc_high == 0x28 || crc_high == 0x0D || crc_high == 0x0A) crc_high++;
    
    return (uint16_t)((crc_high << 8) | crc_low);
}

bool sendCommandAndWait(const char* cmd, char* outBuffer, unsigned long timeoutMs) {
    uint16_t sendCrc = calculate_inverter_crc(cmd, strlen(cmd));
    Serial2.print(cmd);
    Serial2.write((sendCrc >> 8) & 0xFF);
    Serial2.write(sendCrc & 0xFF);
    Serial2.write('\r');

    unsigned long startTime = millis();
    char tempBuffer[256];
    int rxIndex = 0;
    outBuffer[0] = '\0';

    while ((millis() - startTime) < timeoutMs) {
        
        while (Serial2.available() > 0) {
            char c = Serial2.read();
            if (c == '\r') {
                if (rxIndex < 2) {
                    Serial.println("Fehler: Antwort zu kurz!");
                    return false;
                }

                uint8_t crcHigh = tempBuffer[rxIndex - 2];
                uint8_t crcLow  = tempBuffer[rxIndex - 1];
                uint16_t receivedCrc = (crcHigh << 8) | crcLow;

                tempBuffer[rxIndex - 2] = '\0';
                int payloadLength = rxIndex - 2;
                uint16_t calculatedCrc = calculate_inverter_crc(tempBuffer, payloadLength);

                if (calculatedCrc == receivedCrc)
                {
                    strcpy(outBuffer, tempBuffer);
                    return true; 
                } else {
                    Serial.println("Fehler: CRC der Antwort stimmt nicht!");
                    return false;
                }
            }
            else if (rxIndex < (sizeof(tempBuffer) - 1))tempBuffer[rxIndex++] = c;
        }
        delay(1); 
    }
    Serial.println("Fehler: Timeout beim Warten auf Inverter-Antwort!");
    return false;
}

bool parseQPIGS(char* payload, QPIGS_Data& data) {
  if (payload[0] == '(') {
      payload++; 
  }

  int index = 0;
  char* token = strtok(payload, " ");

  while (token != NULL) {
    switch (index) {
      case 0:  data.gridVoltage = atof(token); break;           // Feld 1 
      case 1:  data.gridFrequency = atof(token); break;         // Feld 2 
      case 2:  data.outputVoltage = atof(token); break;         // Feld 3 
      case 3:  data.outputFrequency = atof(token); break;       // Feld 4 
      case 4:  data.outputApparentPower = atoi(token); break;   // Feld 5 
      case 5:  data.outputActivePower = atoi(token); break;     // Feld 6 
      case 6:  data.outputLoadPercent = atoi(token); break;     // Feld 7 
      case 7:  data.busVoltage = atoi(token); break;            // Feld 8 
      case 8:  data.batteryVoltage = atof(token); break;        // Feld 9 
      case 9:  data.batteryChargeCurrent = atoi(token); break;  // Feld 10 
      case 10: data.batteryCapacity = atoi(token); break;       // Feld 11 
      case 11: data.inverterTemp = atoi(token); break;          // Feld 12 
      case 12: data.pvInputCurrent = atof(token); break;        // Feld 13 
      case 13: data.pvInputVoltage = atof(token); break;        // Feld 14 
      case 14: data.sccBatteryVoltage = atof(token); break;     // Feld 15 
      case 15: data.batteryDischargeCurrent = atoi(token); break;// Feld 16 
      
      case 16: // Feld 17: Die Status-Flags als Binär-String 
        if (strlen(token) >= 8)
        {
          data.isSbuPriorityActive    = (token[0] == '1'); // Bit 7
          data.isConfigurationChanged = (token[1] == '1'); // Bit 6
          data.isBusVoltageStable     = (token[2] == '1'); // Bit 5
          data.isPvVoltageSufficient  = (token[3] == '1'); // Bit 4
          data.isLineMode             = (token[4] == '1'); // Bit 3
          data.isChargingOn           = (token[5] == '1'); // Bit 2
          data.isLoadOn               = (token[6] == '1'); // Bit 1
          data.hasWarning             = (token[7] == '1'); // Bit 0
        }
        break;
      case 19: data.pvChargingPower = atoi(token); break; // PV Ladeleistung in Watt
    }
      
    token = strtok(NULL, " ");
    index++;
  }
  return (index > 16); 
}