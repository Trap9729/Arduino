/*
 * WEATHER STATION - HOME ASSISTANT 2025 VERSION
 * ==============================================
 *
 * Sensors:
 *   - SHT30 : Temperature & Humidity (I2C)
 *   - Rain Sensor Module : Analog rain / precipitation detection
 *   - SPS30 : Particulate Matter / Air Quality (I2C)
 *
 * WIRING (ESP8266-D1 / WeMos D1):
 * ┌──────────────┬────────────┬──────────┐
 * │ Sensor       │ Sensor Pin │ D1 Board │
 * ├──────────────┼────────────┼──────────┤
 * │ SHT30        │ VCC        │ 3.3V     │
 * │              │ GND        │ GND      │
 * │              │ SDA        │ D2/GPIO4 │
 * │              │ SCL        │ D1/GPIO5 │
 * ├──────────────┼────────────┼──────────┤
 * │ Rain Sensor  │ VCC        │ 3.3V     │
 * │              │ GND        │ GND      │
 * │              │ AO (analog)│ A0       │
 * ├──────────────┼────────────┼──────────┤
 * │ SPS30        │ VDD (pin1) │ 5V       │  ← needs 5 V, not 3.3 V
 * │              │ SDA (pin2) │ D2/GPIO4 │  ← shared with SHT30
 * │              │ SCL (pin3) │ D1/GPIO5 │  ← shared with SHT30
 * │              │ SEL (pin4) │ GND      │  ← pull to GND = I2C mode
 * │              │ GND (pin5) │ GND      │
 * └──────────────┴────────────┴──────────┘
 *
 * NOTE: SPS30 I2C lines are 3.3 V tolerant — no level shifting needed.
 *       Add 10 kΩ pull-ups on SDA/SCL if signal quality is poor.
 *
 * SETUP STEPS:
 *
 * 1. Create MQTT user in Home Assistant:
 *    Settings → People → Add Person
 *    Create user (avoid "homeassistant" or "addons" as names)
 *
 * 2. Fill in your WiFi and MQTT credentials below.
 *
 * 3. Find Home Assistant IP:
 *    Settings → System → Network → IPv4 address
 *
 * 4. Upload and open Serial Monitor at 115200 baud.
 *
 * 5. Home Assistant will auto-discover the device and show:
 *    - Outside Temperature  (°C)
 *    - Outside Humidity     (%)
 *    - Precipitation        (None / Light Rain / Moderate Rain / Heavy Rain
 *                            / Light Snow / Moderate Snow / Heavy Snow)
 *    - Rain Sensor Raw      (0–1023, diagnostic)
 *    - Air Quality PM1.0    (µg/m³)
 *    - Air Quality PM2.5    (µg/m³)
 *    - Air Quality PM4.0    (µg/m³)
 *    - Air Quality PM10.0   (µg/m³)
 *
 * CALIBRATION:
 *   Open Serial Monitor and observe "Rain Sensor Raw" values.
 *   Adjust RAIN_DRY / RAIN_LIGHT / RAIN_MODERATE thresholds below to match
 *   your specific sensor. Higher raw value = drier.
 *
 * SPS30 WARMUP:
 *   The SPS30 fan/laser needs ~10 s to stabilise after power-on.
 *   PM readings are withheld from the JSON payload until SPS30_WARMUP_MS
 *   has elapsed so that Home Assistant never sees spurious spikes.
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// ============================================================================
// CONFIGURATION — UPDATE THESE VALUES
// ============================================================================

// WiFi (2.4 GHz only)
const char* ssid          = "YOUR_WIFI_NAME";      // ← your WiFi SSID
const char* password      = "YOUR_WIFI_PASSWORD";  // ← your WiFi password

// MQTT / Home Assistant
const char* mqtt_server   = "192.168.1.XXX";       // ← Home Assistant IP
const int   mqtt_port     = 1883;
const char* mqtt_user     = "mqtt_user";            // ← MQTT user created in HA
const char* mqtt_password = "mqtt_password";        // ← password for that user

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// SHT30 I2C address: 0x44 (ADDR pin low/floating) or 0x45 (ADDR pin high)
#define SHT30_ADDR 0x44

// SPS30 I2C address (fixed, cannot be changed)
#define SPS30_ADDR 0x69

// SPS30 minimum warmup before PM readings are published (10 s = safe minimum;
// increase to 30000UL for maximum accuracy after cold start)
#define SPS30_WARMUP_MS 10000UL

// Rain sensor analog thresholds — raw ADC range is 0 (soaked) to 1023 (dry)
// Calibrate by watching "Rain Sensor Raw" in Serial Monitor while wetting the
// sensor with different amounts of water.
#define RAIN_DRY      800   // ≥ this → no precipitation
#define RAIN_LIGHT    550   // ≥ this → light precipitation
#define RAIN_MODERATE 280   // ≥ this → moderate precipitation
                            //  < this → heavy precipitation

// How often to read sensors and publish to MQTT (milliseconds)
#define PUBLISH_INTERVAL_MS 30000UL   // 30 seconds

// Firmware version — bump this string each time new firmware is flashed
#define FW_VERSION "1.0.1"

// ============================================================================
// MQTT TOPICS
// ============================================================================

// All sensor values are published as a single JSON object to STATE_TOPIC.
// Each HA entity uses a value_template to extract its field.
#define STATE_TOPIC  "homeassistant/sensor/weather_station/state"
#define AVAIL_TOPIC  "homeassistant/sensor/weather_station/availability"

// Discovery topics (one per entity)
#define DISC_TEMP    "homeassistant/sensor/ws_temperature/config"
#define DISC_HUMID   "homeassistant/sensor/ws_humidity/config"
#define DISC_PRECIP  "homeassistant/sensor/ws_precipitation/config"
#define DISC_RAW     "homeassistant/sensor/ws_rain_raw/config"
#define DISC_PM1     "homeassistant/sensor/ws_pm1/config"
#define DISC_PM25    "homeassistant/sensor/ws_pm25/config"
#define DISC_PM4     "homeassistant/sensor/ws_pm4/config"
#define DISC_PM10    "homeassistant/sensor/ws_pm10/config"

// ============================================================================
// GLOBALS
// ============================================================================

WiFiClient   espClient;
PubSubClient client(espClient);
unsigned long lastPublish          = 0;
unsigned long lastReconnectAttempt = 0;  // millis-based reconnect back-off
unsigned long sps30StartedAt = 0;   // millis() when measurement was started
bool          sps30Ready     = false; // true once warmup has elapsed
bool          sps30Present   = false; // true only if sps30Begin() succeeded
int8_t        discoveryStep  = -1;   // -1 = idle; 0-7 = step to publish next

struct Sps30Data {
  float pm1;    // PM1.0  µg/m³
  float pm2_5;  // PM2.5  µg/m³
  float pm4;    // PM4.0  µg/m³
  float pm10;   // PM10.0 µg/m³
};

// ============================================================================
// SHT30 — READ TEMPERATURE & HUMIDITY
// ============================================================================
// Protocol: send 2-byte command 0x2C06 (high repeatability, clock stretching),
// wait ~15 ms, then read 6 bytes:
//   [0..1] raw temperature  [2] CRC
//   [3..4] raw humidity     [5] CRC
// Conversion formulas from SHT3x datasheet.

bool sht30Read(float &temperature, float &humidity) {
  Wire.beginTransmission(SHT30_ADDR);
  Wire.write(0x2C);  // MSB of command: high repeatability + clock stretching
  Wire.write(0x06);  // LSB
  if (Wire.endTransmission() != 0) {
    Serial.println("✗ SHT30: I2C write failed");
    return false;
  }

  delay(20);  // Sensor needs ~15 ms; 20 ms gives margin

  if (Wire.requestFrom((uint8_t)SHT30_ADDR, (uint8_t)6) < 6) {
    Serial.println("✗ SHT30: not enough bytes");
    return false;
  }

  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();

  uint16_t rawT = ((uint16_t)d[0] << 8) | d[1];
  uint16_t rawH = ((uint16_t)d[3] << 8) | d[4];

  temperature = -45.0f + 175.0f * (rawT / 65535.0f);
  humidity    = 100.0f         * (rawH / 65535.0f);

  return true;
}

// ============================================================================
// SPS30 — PARTICULATE MATTER SENSOR
// ============================================================================
// Communication: I2C at 0x69 (SEL pin must be tied to GND).
// Supply: 5 V (I2C lines are 3.3 V tolerant).
//
// Protocol summary:
//   Start measurement : write [0x00,0x10, 0x03,0x00,CRC(0x03,0x00)]
//   Read measurement  : write pointer [0x03,0x00], read 24 bytes (first 4 floats — PM1.0 … PM10.0)
//
// Each of the 10 IEEE-754 float values is encoded as:
//   byte0, byte1, CRC(byte0,byte1), byte2, byte3, CRC(byte2,byte3)  = 6 bytes
// Reconstruct float from [byte0, byte1, byte2, byte3] (big-endian).
//
// CRC-8 details: polynomial 0x31, initialisation 0xFF, no reflection.

// CRC-8 over two bytes — used for both command parameters and data validation.
static uint8_t sps30Crc(uint8_t b0, uint8_t b1) {
  uint8_t crc = 0xFF;
  uint8_t buf[2] = {b0, b1};
  for (int i = 0; i < 2; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Send "Start Measurement" command (transitions sensor to measurement mode).
// Call once from setup(); fan and laser turn on and warm up.
bool sps30Begin() {
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(0x00); Wire.write(0x10);        // command: Start Measurement
  Wire.write(0x03); Wire.write(0x00);        // sub-command: IEEE-754 float output
  Wire.write(sps30Crc(0x03, 0x00));          // CRC of the 2 parameter bytes = 0xAC
  if (Wire.endTransmission() != 0) {
    Serial.println("✗ SPS30: start measurement command failed");
    return false;
  }
  delay(20);  // Execution time < 20 ms per datasheet
  return true;
}

// Read a fresh measurement from the SPS30.
// Sensor must be in measurement mode (sps30Begin() called) and warmed up.
bool sps30Read(Sps30Data &d) {
  // Point to the measurement data register
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(0x03); Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("✗ SPS30: read pointer failed");
    return false;
  }

  // Read only the first 4 floats (PM1.0, PM2.5, PM4.0, PM10.0 = 24 bytes).
  // The SPS30 sends 60 bytes total, but the I2C master controls how many it
  // reads; the bus issues NACK+STOP after byte 24, which the sensor accepts.
  // Requesting 24 bytes (< 32) is safe on all ESP8266 Arduino Core versions.
  if (Wire.requestFrom((uint8_t)SPS30_ADDR, (uint8_t)24) < 24) {
    Serial.println("✗ SPS30: short read");
    return false;
  }

  uint8_t raw[24];
  for (int i = 0; i < 24; i++) raw[i] = Wire.read();

  // Parse the first four floats (PM1.0, PM2.5, PM4.0, PM10.0).
  // Each float occupies 6 bytes: b0 b1 CRC01 b2 b3 CRC23.
  // Reassemble as big-endian 32-bit word, then reinterpret as float.
  float *out[4] = {&d.pm1, &d.pm2_5, &d.pm4, &d.pm10};
  for (int i = 0; i < 4; i++) {
    const uint8_t *p = raw + i * 6;
    uint32_t u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
               | ((uint32_t)p[3] <<  8) |  (uint32_t)p[4];
    memcpy(out[i], &u, sizeof(float));
  }
  return true;
}

// ============================================================================
// PRECIPITATION LOGIC
// ============================================================================
// Below 0 °C the water on the sensor is assumed to be snow.

// Returns a short machine-readable type string for automation triggers etc.
const char* precipType(int raw, float temp) {
  if (raw >= RAIN_DRY)      return "none";
  bool snow = (temp < 0.0f);
  if (raw >= RAIN_LIGHT)    return snow ? "light_snow"    : "light_rain";
  if (raw >= RAIN_MODERATE) return snow ? "moderate_snow" : "moderate_rain";
  return                           snow ? "heavy_snow"    : "heavy_rain";
}

// Returns a human-readable label shown in the HA frontend card.
const char* precipLabel(int raw, float temp) {
  if (raw >= RAIN_DRY)      return "None";
  bool snow = (temp < 0.0f);
  if (raw >= RAIN_LIGHT)    return snow ? "Light Snow"    : "Light Rain";
  if (raw >= RAIN_MODERATE) return snow ? "Moderate Snow" : "Moderate Rain";
  return                           snow ? "Heavy Snow"    : "Heavy Rain";
}

// ============================================================================
// MQTT DISCOVERY — one message per loop() iteration
// ============================================================================
// Publishing all 8 payloads in a tight loop overwhelms the ESP8266 TCP send
// buffer and starves the hardware watchdog (rst cause:4).  Instead, each call
// publishes exactly ONE message; loop() calls this once per iteration so that
// client.loop() and the SDK background tasks run between every publish,
// keeping the WDT alive and draining the TCP send buffer.
//
// discoveryStep (global) tracks progress:
//   -1 → idle (no discovery needed)
//    0–7 → index of the message to publish on the next call
// When loop() sees discoveryStep == 8 it prints "Discovery complete." and
// resets discoveryStep to -1.

// No-op callback — registered so PubSubClient never calls a NULL function
// pointer if the broker unexpectedly sends us a PUBLISH packet.
void mqttCallback(char* /*topic*/, byte* /*payload*/, unsigned int /*len*/) {}

void publishDiscoveryStep(int8_t step) {
  // BSS-resident buffer — NOT on the stack (a local char[512] consumes ~92 %
  // of the ESP8266 user stack and causes an Exception(0) crash).
  static char p[512];

  const char* devFrag =
    "\"dev\":{\"ids\":[\"weather_station_001\"],"
    "\"name\":\"Weather Station\",\"mf\":\"DIY\"}";

  const char* topic = nullptr;
  const char* label = nullptr;

  switch (step) {
    case 0:
      snprintf(p, sizeof(p),
        "{\"name\":\"Outside Temperature\","
        "\"uniq_id\":\"ws001_temp\","
        "\"dev_cla\":\"temperature\","
        "\"unit_of_meas\":\"\xc2\xb0""C\","   // °C (UTF-8)
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.temperature }}\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_TEMP;  label = "temp  "; break;

    case 1:
      snprintf(p, sizeof(p),
        "{\"name\":\"Outside Humidity\","
        "\"uniq_id\":\"ws001_humid\","
        "\"dev_cla\":\"humidity\","
        "\"unit_of_meas\":\"%%\","
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.humidity }}\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_HUMID;  label = "humid "; break;

    case 2:
      snprintf(p, sizeof(p),
        "{\"name\":\"Outside Precipitation\","
        "\"uniq_id\":\"ws001_precip\","
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.precip_label }}\","
        "\"icon\":\"mdi:weather-rainy\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_PRECIP;  label = "precip"; break;

    case 3:
      snprintf(p, sizeof(p),
        "{\"name\":\"Rain Sensor Raw\","
        "\"uniq_id\":\"ws001_rain_raw\","
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.rain_raw }}\","
        "\"entity_cat\":\"diagnostic\","
        "\"icon\":\"mdi:water-percent\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_RAW;  label = "raw   "; break;

    case 4:
      snprintf(p, sizeof(p),
        "{\"name\":\"Air Quality PM1.0\","
        "\"uniq_id\":\"ws001_pm1\","
        "\"dev_cla\":\"pm1\","
        "\"unit_of_meas\":\"\xc2\xb5g/m\xc2\xb3\","  // µg/m³ (UTF-8)
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.pm1 }}\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_PM1;  label = "pm1   "; break;

    case 5:
      snprintf(p, sizeof(p),
        "{\"name\":\"Air Quality PM2.5\","
        "\"uniq_id\":\"ws001_pm25\","
        "\"dev_cla\":\"pm25\","
        "\"unit_of_meas\":\"\xc2\xb5g/m\xc2\xb3\","
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.pm25 }}\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_PM25;  label = "pm25  "; break;

    case 6:
      // No standard HA device class for PM4; use air-filter icon instead.
      snprintf(p, sizeof(p),
        "{\"name\":\"Air Quality PM4.0\","
        "\"uniq_id\":\"ws001_pm4\","
        "\"unit_of_meas\":\"\xc2\xb5g/m\xc2\xb3\","
        "\"icon\":\"mdi:air-filter\","
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.pm4 }}\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_PM4;  label = "pm4   "; break;

    case 7:
      snprintf(p, sizeof(p),
        "{\"name\":\"Air Quality PM10.0\","
        "\"uniq_id\":\"ws001_pm10\","
        "\"dev_cla\":\"pm10\","
        "\"unit_of_meas\":\"\xc2\xb5g/m\xc2\xb3\","
        "\"stat_t\":\"" STATE_TOPIC "\","
        "\"val_tpl\":\"{{ value_json.pm10 }}\","
        "\"avty_t\":\"" AVAIL_TOPIC "\","
        "\"pl_avail\":\"online\","
        "\"pl_not_avail\":\"offline\",%s}", devFrag);
      topic = DISC_PM10;  label = "pm10  "; break;

    default: return;  // out of range — ignore
  }

  bool ok = client.publish(topic, p, true);
  Serial.print(ok ? "  ✓ " : "  ✗ ");
  Serial.print(label);
  Serial.print(strlen(p)); Serial.println(" bytes");
}

// ============================================================================
// PUBLISH SENSOR DATA
// ============================================================================

void publishSensorData() {
  float temp = 0.0f, humid = 0.0f;
  if (!sht30Read(temp, humid)) {
    Serial.println("✗ SHT30 read failed — skipping publish");
    return;
  }

  int         raw   = analogRead(A0);
  const char* type  = precipType(raw, temp);
  const char* label = precipLabel(raw, temp);

  // ESP8266 newlib-nano does not enable %f/%g in snprintf by default.
  // dtostrf() is always available on Arduino and sidesteps the issue.
  char sTemp[8], sHumid[8];
  dtostrf(temp,  1, 1, sTemp);
  dtostrf(humid, 1, 1, sHumid);

  // Build JSON payload into a fixed stack buffer — no heap allocation.
  char payload[256];
  int  len = snprintf(payload, sizeof(payload),
               "{\"temperature\":%s,\"humidity\":%s,"
               "\"precip_type\":\"%s\",\"precip_label\":\"%s\","
               "\"rain_raw\":%d",
               sTemp, sHumid, type, label, raw);

  // SPS30 PM readings — only attempted if sps30Begin() succeeded
  unsigned long now = millis();
  if (sps30Present) {
    if (!sps30Ready && (now - sps30StartedAt) >= SPS30_WARMUP_MS) {
      sps30Ready = true;
      Serial.println("✓ SPS30 warmup complete");
    }
    if (sps30Ready) {
      Sps30Data pm;
      if (sps30Read(pm)) {
        char sPm1[8], sPm25[8], sPm4[8], sPm10[8];
        dtostrf(pm.pm1,   1, 1, sPm1);
        dtostrf(pm.pm2_5, 1, 1, sPm25);
        dtostrf(pm.pm4,   1, 1, sPm4);
        dtostrf(pm.pm10,  1, 1, sPm10);
        len += snprintf(payload + len, sizeof(payload) - len,
                 ",\"pm1\":%s,\"pm25\":%s,\"pm4\":%s,\"pm10\":%s",
                 sPm1, sPm25, sPm4, sPm10);
      } else {
        Serial.println("✗ SPS30 read failed — PM values omitted this cycle");
      }
    } else {
      unsigned long remaining = (SPS30_WARMUP_MS - (now - sps30StartedAt)) / 1000UL;
      Serial.print("  SPS30 warming up... ");
      Serial.print(remaining);
      Serial.println("s remaining");
    }
  }

  // Close the JSON object
  payload[len++] = '}';
  payload[len]   = '\0';

  bool ok = client.publish(STATE_TOPIC, payload, true);
  Serial.print(ok ? "✓ Published: " : "✗ Failed:   ");
  Serial.println(payload);
}

// ============================================================================
// WIFI
// ============================================================================

void setupWiFi() {
  Serial.print("Connecting to WiFi \"");
  Serial.print(ssid);
  Serial.print("\"");

  // WiFi.begin() must be called EXACTLY ONCE per boot.  Calling it again
  // after a crash/reconnect hits a bug in the ESP8266 WiFi library
  // (esp8266/Arduino #4078, #6172) that corrupts lwIP PCBs and causes
  // Exception(0/28) crashes.
  //
  // Strategy:
  //   • persistent(false)      — never write credentials to flash; mid-write
  //                              corruption causes boot-loops (#3852)
  //   • setAutoReconnect(true) — SDK reconnects automatically if the AP drops;
  //                              we never need to call begin() a second time
  //   • disconnect(true)       — clears stale lwIP state from a previous WDT
  //                              reset; also calls WiFi.mode(WIFI_OFF)
  //   • delay(200)             — let the radio fully power down before begin()
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(true);   // SDK handles reconnects; we never call begin() again
  WiFi.disconnect(true);         // clear stale PCBs; puts radio in WIFI_OFF
  delay(200);                    // radio needs > 100 ms to fully power down

  WiFi.begin(ssid, password);   // implicitly enables WIFI_STA

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    // Failed to associate after 10 s.  Restart cleanly rather than
    // continuing in a broken state — a clean boot will retry from scratch.
    Serial.println("✗ WiFi FAILED — restarting in 10 s");
    delay(10000);
    ESP.restart();
  }
}

// ============================================================================
// MQTT RECONNECT
// ============================================================================

void reconnectMQTT() {
  // WiFi reconnection is handled by setAutoReconnect(true) in the SDK.
  // Never call setupWiFi() / WiFi.begin() here — doing so is what caused
  // the repeated Exception(0/28) WiFi library crashes.
  if (WiFi.status() != WL_CONNECTED) {
    return;  // Wait; the SDK will re-associate in the background
  }

  Serial.print("Connecting to MQTT at ");
  Serial.print(mqtt_server);
  Serial.print("...");

  // Last-will message sets availability to "offline" if device drops
  if (client.connect("weather_station_001",
                     mqtt_user, mqtt_password,
                     AVAIL_TOPIC, 0, true, "offline")) {
    Serial.println(" ✓");
    client.publish(AVAIL_TOPIC, "online", true);
    // Schedule discovery: loop() will publish one message per iteration so
    // the TCP stack and watchdog are serviced between each payload.
    discoveryStep = 0;
    Serial.println("Publishing MQTT discovery...");
  } else {
    Serial.print(" ✗ rc=");
    Serial.print(client.state());
    Serial.println("  (see PubSubClient.h for error codes)");
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("================================");
  Serial.println(" WEATHER STATION STARTING");
  Serial.print(  " Firmware v"); Serial.println(FW_VERSION);
  Serial.println("================================");

  // I2C — explicit pins matching hardware: SDA=GPIO4 (D2), SCL=GPIO5 (D1)
  Wire.begin(4, 5);
  Wire.setClock(100000);  // 100 kHz — safe for both SHT30 and SPS30

  Serial.print("SHT30 I2C scan... ");
  Wire.beginTransmission(SHT30_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "✓ found" : "✗ NOT found — check wiring!");

  Serial.print("SPS30 I2C scan... ");
  Wire.beginTransmission(SPS30_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("✓ found");
    if (sps30Begin()) {
      sps30Present   = true;
      sps30StartedAt = millis();
      Serial.print("✓ SPS30 measurement started (warming up for ");
      Serial.print(SPS30_WARMUP_MS / 1000);
      Serial.println("s)");
    } else {
      Serial.println("✗ SPS30 start failed — PM readings disabled");
    }
  } else {
    Serial.println("✗ NOT found — check 5V supply, SEL→GND, and SDA/SCL wiring!");
  }

  // Rain sensor
  pinMode(A0, INPUT);
  Serial.println("✓ Rain sensor pin A0 ready");

  // Configure MQTT client before WiFi so heap is as clean as possible.
  // setBufferSize() allocates a heap buffer; doing it early prevents
  // fragmentation failures.  Buffer must hold a complete MQTT PUBLISH packet:
  // headers (~10 B) + topic (~42 B) + payload (up to ~400 B) → ~450 B max;
  // 768 B gives comfortable headroom.
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);  // Prevents NULL-pointer crash if broker
                                     // sends an unexpected PUBLISH packet.
  // CRITICAL: default socket timeout is 15 s.  PubSubClient's CONNACK-wait
  // loop and readPacket() have no yield() call, so they block the CPU.
  // At 15 s the hardware WDT (≈8 s) fires long before the timeout expires.
  // Setting 2 s keeps every blocking section well under both the soft WDT
  // (~3.2 s) and hard WDT (~8 s) thresholds.
  // (knolleary/pubsubclient #1024, #583)
  client.setSocketTimeout(2);
  if (!client.setBufferSize(768)) {
    Serial.println("✗ MQTT setBufferSize(768) FAILED — insufficient heap!");
    // Keep going; publish() will return false for large payloads but won't crash.
  }

  setupWiFi();

  Serial.println("================================");
  Serial.println(" Setup complete!");
  Serial.println("================================");
  Serial.println();
  Serial.println("Precipitation thresholds (raw ADC, 0=wet, 1023=dry):");
  Serial.print("  No precip  : >= "); Serial.println(RAIN_DRY);
  Serial.print("  Light      : >= "); Serial.println(RAIN_LIGHT);
  Serial.print("  Moderate   : >= "); Serial.println(RAIN_MODERATE);
  Serial.println("  Heavy      : < everything above");
  Serial.println("  Snow mode activates when temperature < 0 °C");
  Serial.println();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  if (!client.connected()) {
    // Non-blocking reconnect: attempt at most once every 5 s so that the
    // main loop keeps returning promptly and the WDT stays fed.
    // (Never call connect() every iteration — the CONNACK-wait loop blocks
    // even with setSocketTimeout(2) and would stack up rapidly.)
    unsigned long now = millis();
    if (now - lastReconnectAttempt >= 5000UL) {
      lastReconnectAttempt = now;
      reconnectMQTT();
    }
    return;
  }

  // Service MQTT keep-alive and drain any incoming data before every action.
  // This also feeds the watchdog via the SDK background tasks.
  client.loop();

  // ── Discovery: publish one message per iteration ──────────────────────────
  // Spacing each payload across separate loop() calls gives the TCP send
  // buffer and watchdog time to recover between the large retained publishes.
  if (discoveryStep >= 0) {
    publishDiscoveryStep(discoveryStep);
    discoveryStep++;
    if (discoveryStep >= 8) {
      discoveryStep = -1;
      Serial.println("Discovery complete.");
      lastPublish = 0;  // Trigger first sensor publish after discovery
    }
    return;
  }

  // ── Normal sensor publishing ──────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = now;
    publishSensorData();
  }
}
