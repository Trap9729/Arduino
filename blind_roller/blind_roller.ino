/*
 * BLIND ROLLER CONTROLLER - HOME ASSISTANT
 * =========================================
 *
 * Sensors / Actuators:
 *   - BH1750  : Ambient light sensor (I2C, address 0x23 — ADDR pin LOW)
 *   - Stepper : Blind roller motor via STEP/DIR driver (A4988 / DRV8825)
 *
 * Behaviour:
 *   - Light is measured every 60 seconds.
 *   - 10 seconds before each measurement the blind ramps open so the
 *     sensor gets an unobstructed sky reading.
 *   - After the reading the blind moves to a position calculated from
 *     the measured lux level.
 *   - Manual position commands and mode switching are available over MQTT.
 *
 * WIRING (NodeMCU / ESP8266):
 * ┌──────────────────┬────────────┬──────────────────┐
 * │ Device           │ Device Pin │ NodeMCU           │
 * ├──────────────────┼────────────┼──────────────────┤
 * │ BH1750           │ VCC        │ 3.3V              │
 * │ (ADDR = LOW)     │ GND        │ GND               │
 * │                  │ SDA        │ D2 / GPIO4        │
 * │                  │ SCL        │ D1 / GPIO5        │
 * │                  │ ADDR       │ GND  (→ addr 0x23)│
 * ├──────────────────┼────────────┼──────────────────┤
 * │ Stepper driver   │ STEP       │ D5 / GPIO14       │
 * │ (A4988/DRV8825)  │ DIR        │ D6 / GPIO12       │
 * │                  │ EN         │ D7 / GPIO13       │
 * │                  │ GND        │ GND               │
 * │                  │ VDD (logic)│ 3.3V              │
 * │                  │ VMOT       │ external 12V/24V  │
 * ├──────────────────┼────────────┼──────────────────┤
 * │ Home limit switch│ NO         │ D3 / GPIO0        │
 * │ (optional)       │ COM        │ GND               │
 * └──────────────────┴────────────┴──────────────────┘
 *
 * NOTE: The stepper motor (VMOT) needs its own supply (typically 12 V).
 *       The NodeMCU is powered via USB (5 V) as usual.
 *
 * REQUIRED LIBRARIES (install via Library Manager):
 *   - ESP8266WiFi       (bundled with ESP8266 core)
 *   - PubSubClient      by Nick O'Leary
 *   - BH1750            by Christopher Laws
 *
 * SETUP STEPS:
 *   1. Fill in WiFi and MQTT credentials below.
 *   2. Adjust STEPS_PER_REV and FULL_OPEN_REVS for your blind.
 *   3. Upload and open Serial Monitor at 115200 baud.
 *   4. HA will auto-discover:
 *        - Blind cover entity  (position 0–100)
 *        - Illuminance sensor  (lux)
 *        - Mode select         (Auto / Manual)
 *
 * MQTT TOPICS (all under base prefix "blind_roller"):
 *   Subscribe:
 *     blind_roller/set           – target position 0–100
 *     blind_roller/mode/set      – "auto" | "manual"
 *   Publish:
 *     blind_roller/position      – current position 0–100
 *     blind_roller/lux           – last measured lux
 *     blind_roller/mode          – current mode
 *     blind_roller/availability  – "online" | "offline"
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <BH1750.h>

// ============================================================================
// CONFIGURATION — UPDATE THESE VALUES
// ============================================================================

// WiFi (2.4 GHz only)
const char* WIFI_SSID     = "YOUR_WIFI_NAME";      // ← your WiFi SSID
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";  // ← your WiFi password

// MQTT / Home Assistant
const char* MQTT_SERVER   = "192.168.1.XXX";       // ← Home Assistant IP
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "mqtt_user";            // ← MQTT user in HA
const char* MQTT_PASSWORD = "mqtt_password";        // ← password for that user

// ============================================================================
// MOTOR CONFIGURATION — tune for your specific blind
// ============================================================================

// Steps per full motor revolution (depends on driver microstepping setting).
// A4988 default (no MS pins tied) = full step = 200 steps/rev.
// Half-step = 400, quarter = 800, eighth = 1600, sixteenth = 3200.
#define STEPS_PER_REV   400

// How many full motor revolutions correspond to moving the blind from
// fully closed (position 100) to fully open (position 0).
#define FULL_OPEN_REVS  15

// Motor speed limits (microseconds between steps).
// Larger value = slower. Keep MIN above the driver's minimum pulse timing.
#define STEP_DELAY_MIN_US  800    // fastest step (full cruise speed)
#define STEP_DELAY_MAX_US  5000   // slowest step (start of ramp)

// Number of steps over which to accelerate / decelerate.
#define RAMP_STEPS  200

// ============================================================================
// MEASUREMENT / TIMING
// ============================================================================

#define MEASURE_INTERVAL_MS   60000UL   // measure every 60 s
#define RAMP_ADVANCE_MS       10000UL   // open blind 10 s before measurement

// Lux thresholds → blind position (0 = fully open, 100 = fully closed).
// Adjust to your room / preference.
#define LUX_BRIGHT     30000.0f   // very bright → close blind most of the way
#define LUX_MODERATE    5000.0f   // moderate   → half open
#define LUX_DIM          500.0f   // dim        → mostly open

// Resulting positions for each bracket
#define POS_BRIGHT      80   // 80% closed
#define POS_MODERATE    50   // 50% closed
#define POS_DIM         20   // 20% closed
#define POS_DARK         0   // fully open

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define PIN_STEP    14   // D5
#define PIN_DIR     12   // D6
#define PIN_EN      13   // D7 — active LOW (LOW = driver enabled)
#define PIN_HOME     0   // D3 — limit switch to GND (INPUT_PULLUP)

// ============================================================================
// MQTT TOPICS
// ============================================================================

#define MQTT_BASE          "blind_roller"
#define TOPIC_SET          MQTT_BASE "/set"
#define TOPIC_MODE_SET     MQTT_BASE "/mode/set"
#define TOPIC_POSITION     MQTT_BASE "/position"
#define TOPIC_LUX          MQTT_BASE "/lux"
#define TOPIC_MODE         MQTT_BASE "/mode"
#define TOPIC_AVAIL        MQTT_BASE "/availability"

// Home Assistant MQTT discovery base
#define HA_DISCOVERY_PREFIX "homeassistant"

// ============================================================================
// GLOBALS
// ============================================================================

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
BH1750       lightMeter(0x23);   // ADDR = LOW → address 0x23  (option A)

// Current blind position: 0 = fully open, 100 = fully closed.
int  currentPosition = 100;   // assume closed on boot
int  targetPosition  = 100;

// Steps counted from home (fully open = 0 steps).
long currentSteps = (long)STEPS_PER_REV * FULL_OPEN_REVS;   // fully closed

bool autoMode = true;

// Timing
unsigned long lastMeasureTime = 0;
bool          rampDone        = false;   // has the pre-measurement ramp fired?
float         lastLux         = 0.0f;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void connectWiFi();
void connectMQTT();
void publishDiscovery();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishState();
void takeMeasurementAndAdjust();
void openForMeasurement();
void applyLuxToPosition(float lux);
void moveToPosition(int targetPct, bool withRamp);
void stepMotor(long steps, bool withRamp);
void enableDriver(bool en);
int  posToSteps(int positionPct);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n[Blind Roller] Booting..."));

  // Motor driver pins
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  enableDriver(false);   // disable until needed

  // Home switch
  pinMode(PIN_HOME, INPUT_PULLUP);

  // I2C + BH1750
  Wire.begin();
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("[BH1750] ERROR: sensor not found on 0x23!"));
  } else {
    Serial.println(F("[BH1750] OK — address 0x23 (option A)"));
  }

  connectWiFi();

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  connectMQTT();

  publishDiscovery();
  publishState();

  // Seed timing so first measure fires after one full interval
  lastMeasureTime = millis();
  rampDone        = false;

  Serial.println(F("[Blind Roller] Ready."));
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected())              connectMQTT();
  mqtt.loop();

  unsigned long now     = millis();
  unsigned long elapsed = now - lastMeasureTime;

  // ── Ramp open 10 s before the next measurement ──────────────────────────
  if (!rampDone && elapsed >= (MEASURE_INTERVAL_MS - RAMP_ADVANCE_MS)) {
    Serial.println(F("[Cycle] Ramping open for measurement..."));
    openForMeasurement();
    rampDone = true;
  }

  // ── Take measurement ─────────────────────────────────────────────────────
  if (elapsed >= MEASURE_INTERVAL_MS) {
    takeMeasurementAndAdjust();
    lastMeasureTime = millis();
    rampDone        = false;
  }
}

// ============================================================================
// MEASUREMENT & BLIND ADJUSTMENT
// ============================================================================

// Ramp blind fully open (position 0) in the 10 s window before measuring.
// Uses the motor ramp so motion starts slow and smoothly accelerates.
void openForMeasurement() {
  moveToPosition(0, /*withRamp=*/true);
}

void takeMeasurementAndAdjust() {
  float lux = lightMeter.readLightLevel();
  if (lux < 0) {
    Serial.println(F("[BH1750] Read error — skipping adjustment"));
    return;
  }

  lastLux = lux;
  Serial.print(F("[Measure] "));
  Serial.print(lux, 1);
  Serial.println(F(" lux"));

  if (autoMode) {
    applyLuxToPosition(lux);
  }

  publishState();
}

// Map measured lux to a blind position and move there.
void applyLuxToPosition(float lux) {
  int pos;
  if      (lux >= LUX_BRIGHT)   pos = POS_BRIGHT;
  else if (lux >= LUX_MODERATE) pos = POS_MODERATE;
  else if (lux >= LUX_DIM)      pos = POS_DIM;
  else                           pos = POS_DARK;

  Serial.print(F("[Auto] Target position: "));
  Serial.println(pos);
  moveToPosition(pos, /*withRamp=*/false);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

// Move blind to positionPct (0 = open, 100 = closed).
void moveToPosition(int posPct, bool withRamp) {
  posPct = constrain(posPct, 0, 100);
  if (posPct == currentPosition) return;

  long targetSteps = posToSteps(posPct);
  long delta       = targetSteps - currentSteps;

  Serial.print(F("[Motor] "));
  Serial.print(currentPosition);
  Serial.print(F("% → "));
  Serial.print(posPct);
  Serial.print(F("%  ("));
  Serial.print(abs(delta));
  Serial.println(F(" steps)"));

  stepMotor(delta, withRamp);

  currentSteps    = targetSteps;
  currentPosition = posPct;

  publishState();
}

// Drive the stepper motor by `steps` steps (positive = close, negative = open).
// withRamp = true applies a linear acceleration / deceleration profile.
void stepMotor(long steps, bool withRamp) {
  if (steps == 0) return;

  bool forward = (steps > 0);
  long count   = abs(steps);

  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
  delayMicroseconds(2);   // DIR setup time

  enableDriver(true);
  delay(5);               // driver wake-up

  for (long i = 0; i < count; i++) {
    unsigned int delayUs = STEP_DELAY_MIN_US;

    if (withRamp) {
      // Acceleration phase
      if (i < RAMP_STEPS) {
        delayUs = map(i, 0, RAMP_STEPS, STEP_DELAY_MAX_US, STEP_DELAY_MIN_US);
      }
      // Deceleration phase (mirror of acceleration at the end)
      else if (i >= (count - RAMP_STEPS)) {
        long fromEnd = count - 1 - i;
        delayUs = map(fromEnd, 0, RAMP_STEPS, STEP_DELAY_MAX_US, STEP_DELAY_MIN_US);
      }
    }

    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(delayUs);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(delayUs);

    // Allow background tasks while moving (yield every 64 steps)
    if ((i & 63) == 63) yield();
  }

  enableDriver(false);   // de-energise coils to reduce heat
}

void enableDriver(bool en) {
  digitalWrite(PIN_EN, en ? LOW : HIGH);   // EN is active LOW
}

// Convert position percentage to step count from home (0 steps = fully open).
int posToSteps(int positionPct) {
  return map(positionPct, 0, 100, 0, (long)STEPS_PER_REV * FULL_OPEN_REVS);
}

// ============================================================================
// MQTT
// ============================================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.print(F("[MQTT] ← "));
  Serial.print(topic);
  Serial.print(F(": "));
  Serial.println(msg);

  if (strcmp(topic, TOPIC_SET) == 0) {
    int pos = constrain(msg.toInt(), 0, 100);
    autoMode = false;
    mqtt.publish(TOPIC_MODE, "manual", /*retain=*/true);
    moveToPosition(pos, /*withRamp=*/true);
  }
  else if (strcmp(topic, TOPIC_MODE_SET) == 0) {
    autoMode = (msg == "auto");
    mqtt.publish(TOPIC_MODE, autoMode ? "auto" : "manual", true);
    publishState();
  }
}

void publishState() {
  char buf[16];

  snprintf(buf, sizeof(buf), "%d", currentPosition);
  mqtt.publish(TOPIC_POSITION, buf, /*retain=*/true);

  dtostrf(lastLux, 4, 1, buf);
  mqtt.publish(TOPIC_LUX, buf, true);

  mqtt.publish(TOPIC_MODE, autoMode ? "auto" : "manual", true);
}

// HA MQTT discovery payloads — cover entity + illuminance sensor + mode select
void publishDiscovery() {
  // Cover (blind)
  mqtt.publish(
    HA_DISCOVERY_PREFIX "/cover/blind_roller/config",
    "{"
      "\"name\":\"Blind Roller\","
      "\"unique_id\":\"blind_roller_cover\","
      "\"device_class\":\"blind\","
      "\"command_topic\":\"" TOPIC_SET "\","
      "\"position_topic\":\"" TOPIC_POSITION "\","
      "\"set_position_topic\":\"" TOPIC_SET "\","
      "\"availability_topic\":\"" TOPIC_AVAIL "\","
      "\"position_open\":0,"
      "\"position_closed\":100,"
      "\"device\":{"
        "\"identifiers\":[\"blind_roller_01\"],"
        "\"name\":\"Blind Roller Controller\","
        "\"model\":\"NodeMCU v2\","
        "\"manufacturer\":\"DIY\""
      "}"
    "}",
    /*retain=*/true
  );

  // Illuminance sensor
  mqtt.publish(
    HA_DISCOVERY_PREFIX "/sensor/blind_roller_lux/config",
    "{"
      "\"name\":\"Blind Roller Illuminance\","
      "\"unique_id\":\"blind_roller_lux\","
      "\"device_class\":\"illuminance\","
      "\"state_class\":\"measurement\","
      "\"unit_of_measurement\":\"lx\","
      "\"state_topic\":\"" TOPIC_LUX "\","
      "\"availability_topic\":\"" TOPIC_AVAIL "\","
      "\"device\":{\"identifiers\":[\"blind_roller_01\"]}"
    "}",
    true
  );

  // Mode select
  mqtt.publish(
    HA_DISCOVERY_PREFIX "/select/blind_roller_mode/config",
    "{"
      "\"name\":\"Blind Roller Mode\","
      "\"unique_id\":\"blind_roller_mode\","
      "\"command_topic\":\"" TOPIC_MODE_SET "\","
      "\"state_topic\":\"" TOPIC_MODE "\","
      "\"options\":[\"auto\",\"manual\"],"
      "\"availability_topic\":\"" TOPIC_AVAIL "\","
      "\"device\":{\"identifiers\":[\"blind_roller_01\"]}"
    "}",
    true
  );

  mqtt.publish(TOPIC_AVAIL, "online", true);
}

// ============================================================================
// CONNECTIVITY
// ============================================================================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print(F("[WiFi] Connecting to "));
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\n[WiFi] Connected — IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("\n[WiFi] Failed — will retry in main loop"));
  }
}

void connectMQTT() {
  if (mqtt.connected()) return;

  Serial.print(F("[MQTT] Connecting..."));

  // Last-will keeps availability accurate if the device drops offline
  if (mqtt.connect("blind_roller_01", MQTT_USER, MQTT_PASSWORD,
                   TOPIC_AVAIL, 0, /*retain=*/true, "offline")) {
    Serial.println(F(" connected."));
    mqtt.publish(TOPIC_AVAIL, "online", true);
    mqtt.subscribe(TOPIC_SET);
    mqtt.subscribe(TOPIC_MODE_SET);
  } else {
    Serial.print(F(" failed, rc="));
    Serial.println(mqtt.state());
  }
}
