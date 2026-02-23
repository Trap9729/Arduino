/*
 * WEATHER STATION - HOME ASSISTANT 2025 VERSION
 * ==============================================
 *
 * Sensors:
 *   - SHT30 : Temperature & Humidity (I2C)
 *   - Rain Sensor Module : Analog rain / precipitation detection
 *
 * WIRING (NodeMCU / ESP8266):
 * ┌──────────────┬────────────┬──────────┐
 * │ Sensor       │ Sensor Pin │ NodeMCU  │
 * ├──────────────┼────────────┼──────────┤
 * │ SHT30        │ VCC        │ 3.3V     │
 * │              │ GND        │ GND      │
 * │              │ SDA        │ D2/GPIO4 │
 * │              │ SCL        │ D1/GPIO5 │
 * ├──────────────┼────────────┼──────────┤
 * │ Rain Sensor  │ VCC        │ 3.3V     │
 * │              │ GND        │ GND      │
 * │              │ AO (analog)│ A0       │
 * └──────────────┴────────────┴──────────┘
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
 *
 * CALIBRATION:
 *   Open Serial Monitor and observe "Rain Sensor Raw" values.
 *   Adjust RAIN_DRY / RAIN_LIGHT / RAIN_MODERATE thresholds below to match
 *   your specific sensor. Higher raw value = drier.
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

// Rain sensor analog thresholds — raw ADC range is 0 (soaked) to 1023 (dry)
// Calibrate by watching "Rain Sensor Raw" in Serial Monitor while wetting the
// sensor with different amounts of water.
#define RAIN_DRY      800   // ≥ this → no precipitation
#define RAIN_LIGHT    550   // ≥ this → light precipitation
#define RAIN_MODERATE 280   // ≥ this → moderate precipitation
                            //  < this → heavy precipitation

// How often to read sensors and publish to MQTT (milliseconds)
#define PUBLISH_INTERVAL_MS 30000UL   // 30 seconds

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

// ============================================================================
// GLOBALS
// ============================================================================

WiFiClient   espClient;
PubSubClient client(espClient);
unsigned long lastPublish = 0;

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
// PRECIPITATION LOGIC
// ============================================================================
// Below 0 °C the water on the sensor is assumed to be snow.

// Returns a short machine-readable type string for automation triggers etc.
String precipType(int raw, float temp) {
  if (raw >= RAIN_DRY)      return "none";
  bool snow = (temp < 0.0f);
  if (raw >= RAIN_LIGHT)    return snow ? "light_snow"    : "light_rain";
  if (raw >= RAIN_MODERATE) return snow ? "moderate_snow" : "moderate_rain";
  return                           snow ? "heavy_snow"    : "heavy_rain";
}

// Returns a human-readable label shown in the HA frontend card.
String precipLabel(int raw, float temp) {
  if (raw >= RAIN_DRY)      return "None";
  bool snow = (temp < 0.0f);
  const char* kind = snow ? "Snow" : "Rain";
  if (raw >= RAIN_LIGHT)    { String s = "Light ";    s += kind; return s; }
  if (raw >= RAIN_MODERATE) { String s = "Moderate "; s += kind; return s; }
  {                           String s = "Heavy ";    s += kind; return s; }
}

// Returns an appropriate MDI icon for the precipitation state.
const char* precipIcon(int raw, float temp) {
  if (raw >= RAIN_DRY)    return "mdi:weather-sunny";
  if (temp < 0.0f) {
    if (raw >= RAIN_LIGHT)    return "mdi:weather-snowy";
    if (raw >= RAIN_MODERATE) return "mdi:weather-snowy-heavy";
    return "mdi:weather-snowy-heavy";
  }
  if (raw >= RAIN_LIGHT)    return "mdi:weather-rainy";
  if (raw >= RAIN_MODERATE) return "mdi:weather-pouring";
  return "mdi:weather-pouring";
}

// ============================================================================
// MQTT DISCOVERY — publish once on (re)connect
// ============================================================================
// Each entity shares the same device block so HA groups them under one device.
// Buffer is set to 512 bytes; payloads are kept compact using abbreviations
// from the HA MQTT documentation.

void publishDiscovery() {
  Serial.println("Publishing MQTT discovery...");
  client.setBufferSize(600);  // Ensure buffer is large enough

  // Shared device fragment (appended to every payload)
  const char* devFrag =
    "\"dev\":{\"ids\":[\"weather_station_001\"],"
    "\"name\":\"Weather Station\",\"mf\":\"DIY\"}";

  // ── Temperature ──────────────────────────────────────────────────────────
  {
    String p = "{";
    p += "\"name\":\"Outside Temperature\","
         "\"uniq_id\":\"ws001_temp\","
         "\"dev_cla\":\"temperature\","
         "\"unit_of_meas\":\"\xc2\xb0""C\","   // °C (UTF-8)
         "\"stat_t\":\"" STATE_TOPIC "\","
         "\"val_tpl\":\"{{ value_json.temperature }}\","
         "\"avty_t\":\"" AVAIL_TOPIC "\","
         "\"pl_avail\":\"online\","
         "\"pl_not_avail\":\"offline\",";
    p += devFrag;
    p += "}";
    bool ok = client.publish(DISC_TEMP, p.c_str(), true);
    Serial.print(ok ? "  ✓ temp   " : "  ✗ temp   ");
    Serial.print(p.length()); Serial.println(" bytes");
    delay(100);
  }

  // ── Humidity ─────────────────────────────────────────────────────────────
  {
    String p = "{";
    p += "\"name\":\"Outside Humidity\","
         "\"uniq_id\":\"ws001_humid\","
         "\"dev_cla\":\"humidity\","
         "\"unit_of_meas\":\"%\","
         "\"stat_t\":\"" STATE_TOPIC "\","
         "\"val_tpl\":\"{{ value_json.humidity }}\","
         "\"avty_t\":\"" AVAIL_TOPIC "\","
         "\"pl_avail\":\"online\","
         "\"pl_not_avail\":\"offline\",";
    p += devFrag;
    p += "}";
    bool ok = client.publish(DISC_HUMID, p.c_str(), true);
    Serial.print(ok ? "  ✓ humid  " : "  ✗ humid  ");
    Serial.print(p.length()); Serial.println(" bytes");
    delay(100);
  }

  // ── Precipitation label ───────────────────────────────────────────────────
  {
    String p = "{";
    p += "\"name\":\"Outside Precipitation\","
         "\"uniq_id\":\"ws001_precip\","
         "\"stat_t\":\"" STATE_TOPIC "\","
         "\"val_tpl\":\"{{ value_json.precip_label }}\","
         "\"icon\":\"mdi:weather-rainy\","
         "\"avty_t\":\"" AVAIL_TOPIC "\","
         "\"pl_avail\":\"online\","
         "\"pl_not_avail\":\"offline\",";
    p += devFrag;
    p += "}";
    bool ok = client.publish(DISC_PRECIP, p.c_str(), true);
    Serial.print(ok ? "  ✓ precip " : "  ✗ precip ");
    Serial.print(p.length()); Serial.println(" bytes");
    delay(100);
  }

  // ── Rain sensor raw (diagnostic) ─────────────────────────────────────────
  {
    String p = "{";
    p += "\"name\":\"Rain Sensor Raw\","
         "\"uniq_id\":\"ws001_rain_raw\","
         "\"stat_t\":\"" STATE_TOPIC "\","
         "\"val_tpl\":\"{{ value_json.rain_raw }}\","
         "\"entity_cat\":\"diagnostic\","
         "\"icon\":\"mdi:water-percent\","
         "\"avty_t\":\"" AVAIL_TOPIC "\","
         "\"pl_avail\":\"online\","
         "\"pl_not_avail\":\"offline\",";
    p += devFrag;
    p += "}";
    bool ok = client.publish(DISC_RAW, p.c_str(), true);
    Serial.print(ok ? "  ✓ raw    " : "  ✗ raw    ");
    Serial.print(p.length()); Serial.println(" bytes");
    delay(100);
  }

  Serial.println("Discovery complete.");
}

// ============================================================================
// PUBLISH SENSOR DATA
// ============================================================================

void publishSensorData() {
  float temp = 0.0f, humid = 0.0f;
  if (!sht30Read(temp, humid)) {
    Serial.println("✗ Sensor read failed — skipping publish");
    return;
  }

  int   raw   = analogRead(A0);
  String type  = precipType(raw, temp);
  String label = precipLabel(raw, temp);

  // Build JSON payload
  String payload = "{";
  payload += "\"temperature\":"   + String(temp,  1) + ",";
  payload += "\"humidity\":"      + String(humid, 1) + ",";
  payload += "\"precip_type\":\""  + type  + "\",";
  payload += "\"precip_label\":\"" + label + "\",";
  payload += "\"rain_raw\":"       + String(raw);
  payload += "}";

  bool ok = client.publish(STATE_TOPIC, payload.c_str(), true);
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

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi FAILED — check SSID/password and 2.4 GHz band");
  }
}

// ============================================================================
// MQTT RECONNECT
// ============================================================================

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
    return;
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
    publishDiscovery();
    lastPublish = 0;  // Force immediate sensor publish
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
  Serial.println("================================");

  // I2C — default NodeMCU pins: SDA=D2 (GPIO4), SCL=D1 (GPIO5)
  Wire.begin();
  Wire.setClock(100000);  // 100 kHz — safe for SHT30
  Serial.print("SHT30 I2C scan... ");
  Wire.beginTransmission(SHT30_ADDR);
  Serial.println(Wire.endTransmission() == 0 ? "✓ found" : "✗ NOT found — check wiring!");

  // Rain sensor
  pinMode(A0, INPUT);
  Serial.println("✓ Rain sensor pin A0 ready");

  setupWiFi();

  client.setServer(mqtt_server, mqtt_port);
  // No inbound messages expected; no callback needed

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
    reconnectMQTT();
    delay(5000);
    return;
  }

  client.loop();

  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = now;
    publishSensorData();
  }
}
