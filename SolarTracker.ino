#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

#define CONFIG_FILE_SIZE 57
// ----- Pin Definitions -----

int solar_ampere_pin = 32;
int solar_voltage_pin = 33;
int battery_ampere_pin = 34;
int battery_voltage_pin = 35;
int inverter_voltage_pin = 21;
int inverter_relay_pin = 23;


// ----- Timing Variables -----

uint32_t last_save = 0;
uint32_t saving_cycle = 600000;  // Cycle to save Samples

uint32_t last_sample = 0;
uint32_t sample_cycle = 500;

uint32_t last_inverter_online = 0;
uint32_t inverter_off_duration = 10000;      // Duration after last online before relay
uint32_t inverter_cycle_duration = 300000;   // Cycle Duration after first relay
uint32_t inverter_relay_duration = 5000;     // Duration of Relay Toggle

uint32_t relay_overwrite_start = 0;
uint32_t relay_overwrite_duration = 5000; 

uint32_t last_unstable_bat = 0;


// ----- Sample Thresholds -----

float solar_night_voltage = 5.0;
float battery_under_voltage = 20;

uint16_t solar_amp_offset = 1780;
uint16_t battery_amp_offset = 1780;
int16_t battery_amp_offset_offset = 0;            // Offset to solar_amp_offset

const float amp_step = 0.0138718;
float solar_amp_corr = 1.0; 
float battery_amp_corr = 1.0;
float ignore_amp = 1.0;


// ----- Sample Buffers -----

float solar_voltage = 0;
float solar_ampere = 0;
float battery_voltage = 0;
float battery_ampere = 0;

// ----- 10min Buffer -----

int64_t acc_solar_amp_raw = 0;
int64_t acc_solar_volt = 0;
int64_t acc_battery_amp_raw = 0;
int64_t acc_battery_volt = 0;

uint16_t sample_counter = 0;

float period_solar_wh = 0;
float period_battery_charge_wh = 0;
float period_battery_discharge_wh = 0;
float period_max_sol_amp = 0;
float period_max_bat_discharge_amp = 0;
float period_max_bat_volt = 0;
float period_min_bat_volt = 35;
float period_min_bat_volt_sunrise = 35;

// ----- Day Buffers -----

float ring_solar_wh[144];
float ring_battery_charge_wh[144];
float ring_battery_discharge_wh[144];
float ring_max_sol_amp[144];
float ring_max_bat_discharge_amp[144];
float ring_max_bat_volt[144];
float ring_min_bat_volt[144];
float ring_min_bat_volt_sunrise[144];
uint8_t ring_buffer_counter = 0;

struct RingSlot {
    float solar_wh;
    float bat_charge_wh;
    float bat_discharge_wh;
    float max_sol_amp;
    float max_bat_dis_amp;
    float max_bat_volt;
    float min_bat_volt;
    float min_bat_volt_sunrise;
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

// ----- Objects -----

DNSServer dnsServer;
WebServer server(80);
File uploadFile;

void setup() {
  Serial.begin(115200);

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
    ring_max_bat_discharge_amp[i] = 0.0;
    ring_max_bat_volt[i] = 0.0;
    ring_min_bat_volt[i] = 35.0f;
    ring_min_bat_volt_sunrise[i] = 35.0f;
  }

  if (!LittleFS.exists("/ring.bin"))
  {
    File file = LittleFS.open("/ring.bin", "w");
    file.write(0);
    RingSlot empty = {0, 0, 0, 0, 0, 0, 35.0f, 35.0f};
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
             "{\"sol_v\":%.2f,\"sol_a\":%.2f,\"bat_v\":%.2f,\"bat_a\":%.2f,\"inv_relay\":%d,\"inv_volt\":%d}",
             solar_voltage, solar_ampere, battery_voltage, battery_ampere, 
             digitalRead(inverter_relay_pin), digitalRead(inverter_voltage_pin));
    server.send(200, "application/json", buf);
  });

  server.on("/toggle", HTTP_GET, []() {
    ToggleInverter();
    server.send(200, "text/plain", "Inverter Toggle aktiviert!");
  });

  server.on("/config.bin", HTTP_GET, []() {
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
      RingSlot empty = {0, 0, 0, 0, 0, 0, 35.0f, 35.0f};
      for(int i = 0; i < 144; i++) {
        file.write((uint8_t*)&empty, sizeof(RingSlot));
      }
      file.close();
    }
    if (!LittleFS.exists("/ring.bin"))
    {
      File file = LittleFS.open("/ring.bin", "w");
      file.write(0);
      RingSlot empty = {0, 0, 0, 0, 0, 0, 35.0f, 35.0f};
      for(int i=0; i<144; i++) file.write((uint8_t*)&empty, sizeof(RingSlot));
      file.close();

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
      ring_max_bat_discharge_amp[i] = 0.0f;
      ring_max_bat_volt[i] = 0.0f;
      ring_min_bat_volt[i] = 35.0f;
      ring_min_bat_volt_sunrise[i] = 35.0f;
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
    // Replace with I2C Reading soon
    int16_t sol_amp_buf = analogRead(solar_ampere_pin) * 8;
    int16_t sol_volt_buf = analogRead(solar_voltage_pin) * 8;
    int16_t bat_amp_buf = analogRead(battery_ampere_pin) * 8;
    int16_t bat_volt_buf = analogRead(battery_voltage_pin) * 8;

    int16_t zero = 0;

    sol_amp_buf = max(zero, sol_amp_buf);
    sol_volt_buf = max(zero, sol_volt_buf);
    bat_amp_buf = max(zero, bat_amp_buf);
    bat_volt_buf = max(zero, bat_volt_buf);

    // Accumulating RAW Logging Buffers

    acc_solar_amp_raw += sol_amp_buf;
    acc_solar_volt += sol_volt_buf;
    acc_battery_amp_raw += bat_amp_buf;
    acc_battery_volt += bat_volt_buf;
    sample_counter ++;

    // Conversion of Raw-Data

    solar_voltage = sol_volt_buf * (40 / 32767.0f);
    battery_voltage = bat_volt_buf * (30 / 32767.0f);

    if(solar_voltage < 5.0)
    {
      solar_amp_offset = (solar_amp_offset * 0.99f) + (sol_amp_buf * 0.01f);
      battery_amp_offset = solar_amp_offset + battery_amp_offset_offset;
    }

    float solar_ampere_ = (float)(sol_amp_buf - solar_amp_offset) * amp_step * solar_amp_corr;
    float battery_ampere_ = (float)(bat_amp_buf - battery_amp_offset) * amp_step * battery_amp_corr;
    if(solar_ampere_ < ignore_amp) solar_ampere_ = 0.0f;
    if(abs(battery_ampere_) < ignore_amp) battery_ampere_ = 0.0f;

    solar_ampere = (solar_ampere * 0.8f) + (solar_ampere_ * 0.2f);
    battery_ampere = (battery_ampere * 0.8f) + (battery_ampere_ * 0.2f);

    // Analysis for Period Logging

    const float hours_per_sample = (float)sample_cycle / 3600000.0f;
    period_solar_wh += solar_ampere_ * solar_voltage * hours_per_sample;
    float batt_p_now = battery_ampere_ * battery_voltage * hours_per_sample;
    if(battery_ampere_ >= 0)period_battery_charge_wh += batt_p_now;
    else period_battery_discharge_wh += abs(batt_p_now);


    if(battery_ampere < -5.0f)
    {
      last_unstable_bat = millis();
    }
    else if(millis()-last_unstable_bat > 600000 && battery_voltage < period_min_bat_volt)
    {
      period_min_bat_volt = battery_voltage;
    }
    period_max_sol_amp = max(period_max_sol_amp, solar_ampere_);
    period_max_bat_discharge_amp = max(period_max_bat_discharge_amp, abs(min(0.0f, battery_ampere_)));
    period_max_bat_volt = max(period_max_bat_volt, battery_voltage);
    if(solar_voltage < 5.0f)period_min_bat_volt_sunrise = min(period_min_bat_volt_sunrise, battery_voltage);
    total_uptime += sample_cycle;
  }
}

void SaveSample()
{
  if(millis() - last_save >= saving_cycle)
  {
    // ----- Calculate Average -----
    last_save += saving_cycle;
    uint16_t bat_amp = acc_battery_amp_raw / sample_counter;
    uint16_t sol_amp = acc_solar_amp_raw / sample_counter;
    uint16_t bat_volt = acc_battery_volt / sample_counter;
    uint16_t sol_volt = acc_solar_volt / sample_counter;
    
    // ----- Compression for Raw-Logging -----

    bat_amp = bat_amp >> 3;     // Shift for 12Bit Conversion, 15th bit is signed bit, can be ignored

    if(sol_amp < 15000)sol_amp = 15000;
    if(sol_amp > 23190)sol_amp = 23190;
    sol_amp -= 15000;
    sol_amp = sol_amp >> 1;

    if(bat_volt < 20753) bat_volt = 20753;
    bat_volt -= 20753;          // 19/30 * 2^15
    bat_volt = bat_volt / 48;

    sol_volt = sol_volt >> 8;
    sol_volt = sol_volt & 0x7F;
    sol_volt = sol_volt | (digitalRead(inverter_voltage_pin) << 7);

    uint8_t buffer[5];
    buffer[0] = (sol_amp >> 4) & 0xFF; 
    buffer[1] = (sol_amp << 4) & 0xF0;
    buffer[1] |= (bat_amp >> 8) & 0x0F;
    buffer[2] = bat_amp & 0xFF;
    buffer[3] = sol_volt & 0xFF;
    buffer[4] = bat_volt & 0xFF;

    if(enable_logging)
    { 
      File file = LittleFS.open("/log.bin", FILE_APPEND);
      if(!file)
      {
        Serial.println("Fehler beim Öffnen der log.bin!");
        return;
      }
      
      file.write(buffer, 5);
      file.close();
    }

    sample_counter = 0;
    acc_battery_amp_raw = 0;
    acc_solar_amp_raw = 0;
    acc_battery_volt = 0;
    acc_solar_volt = 0;

    // Analysis for 24-Hour Stats

    ring_solar_wh[ring_buffer_counter] = period_solar_wh;
    ring_battery_charge_wh[ring_buffer_counter] = period_battery_charge_wh;
    ring_battery_discharge_wh[ring_buffer_counter] = period_battery_discharge_wh;
    ring_max_sol_amp[ring_buffer_counter] = period_max_sol_amp;
    ring_max_bat_discharge_amp[ring_buffer_counter] = period_max_bat_discharge_amp;
    ring_max_bat_volt[ring_buffer_counter] = period_max_bat_volt;
    ring_min_bat_volt[ring_buffer_counter] = period_min_bat_volt;
    ring_min_bat_volt_sunrise[ring_buffer_counter] = period_min_bat_volt_sunrise;
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
    period_max_bat_discharge_amp = 0;
    period_max_bat_volt = 0;
    period_min_bat_volt = 35;
    period_min_bat_volt_sunrise = 35;
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
      else if(cycle_inverter || solar_voltage > 30)
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
  memcpy(&buffer[RunningIndex(sizeof(relay_overwrite_start), counter)], &relay_overwrite_start, sizeof(relay_overwrite_start));
  memcpy(&buffer[RunningIndex(sizeof(relay_overwrite_duration), counter)], &relay_overwrite_duration, sizeof(relay_overwrite_duration));

  memcpy(&buffer[RunningIndex(sizeof(solar_night_voltage), counter)], &solar_night_voltage, sizeof(solar_night_voltage));
  memcpy(&buffer[RunningIndex(sizeof(battery_under_voltage), counter)], &battery_under_voltage, sizeof(battery_under_voltage));

  memcpy(&buffer[RunningIndex(sizeof(solar_amp_offset), counter)], &solar_amp_offset, sizeof(solar_amp_offset));
  memcpy(&buffer[RunningIndex(sizeof(battery_amp_offset), counter)], &battery_amp_offset, sizeof(battery_amp_offset));
  memcpy(&buffer[RunningIndex(sizeof(battery_amp_offset_offset), counter)], &battery_amp_offset_offset, sizeof(battery_amp_offset_offset));

  memcpy(&buffer[RunningIndex(sizeof(solar_amp_corr), counter)], &solar_amp_corr, sizeof(solar_amp_corr));
  memcpy(&buffer[RunningIndex(sizeof(battery_amp_corr), counter)], &battery_amp_corr, sizeof(battery_amp_corr));
  memcpy(&buffer[RunningIndex(sizeof(ignore_amp), counter)], &ignore_amp, sizeof(ignore_amp));

  memcpy(&buffer[RunningIndex(sizeof(cycle_inverter), counter)], &cycle_inverter, sizeof(cycle_inverter));
  memcpy(&buffer[RunningIndex(sizeof(relay_overwrite_active), counter)], &relay_overwrite_active, sizeof(relay_overwrite_active));
  memcpy(&buffer[RunningIndex(sizeof(enable_logging), counter)], &enable_logging, sizeof(enable_logging));

  config_file.write(buffer, 57);
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
  memcpy(&relay_overwrite_start, &buffer[RunningIndex(sizeof(relay_overwrite_start), counter)], sizeof(relay_overwrite_start));
  memcpy(&relay_overwrite_duration, &buffer[RunningIndex(sizeof(relay_overwrite_duration), counter)], sizeof(relay_overwrite_duration));

  memcpy(&solar_night_voltage, &buffer[RunningIndex(sizeof(solar_night_voltage), counter)], sizeof(solar_night_voltage));
  memcpy(&battery_under_voltage, &buffer[RunningIndex(sizeof(battery_under_voltage), counter)], sizeof(battery_under_voltage));

  memcpy(&solar_amp_offset, &buffer[RunningIndex(sizeof(solar_amp_offset), counter)], sizeof(solar_amp_offset));
  memcpy(&battery_amp_offset, &buffer[RunningIndex(sizeof(battery_amp_offset), counter)], sizeof(battery_amp_offset));
  memcpy(&battery_amp_offset_offset, &buffer[RunningIndex(sizeof(battery_amp_offset_offset), counter)], sizeof(battery_amp_offset_offset));

  memcpy(&solar_amp_corr, &buffer[RunningIndex(sizeof(solar_amp_corr), counter)], sizeof(solar_amp_corr));
  memcpy(&battery_amp_corr, &buffer[RunningIndex(sizeof(battery_amp_corr), counter)], sizeof(battery_amp_corr));
  memcpy(&ignore_amp, &buffer[RunningIndex(sizeof(ignore_amp), counter)], sizeof(ignore_amp));

  memcpy(&cycle_inverter, &buffer[RunningIndex(sizeof(cycle_inverter), counter)], sizeof(cycle_inverter));
  memcpy(&relay_overwrite_active, &buffer[RunningIndex(sizeof(relay_overwrite_active), counter)], sizeof(relay_overwrite_active));
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
    ring_max_sol_amp[index], ring_max_bat_discharge_amp[index], ring_max_bat_volt[index],
    ring_min_bat_volt[index], ring_min_bat_volt_sunrise[index]
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
      ring_max_bat_discharge_amp[i] = slot.max_bat_dis_amp;
      ring_max_bat_volt[i] = slot.max_bat_volt;
      ring_min_bat_volt[i] = slot.min_bat_volt;
      ring_min_bat_volt_sunrise[i] = slot.min_bat_volt_sunrise;
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
