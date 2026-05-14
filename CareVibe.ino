/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║                CareVibe v2.0 — CareVibe.ino                     ║
 * ║        ESP32 Health & Safety Wearable for Elderly Monitoring    ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  Architecture : Non-blocking state machine                      ║
 * ║  Protocols    : I2C (x3) | UART | 1-Wire | Analog | Digital    ║
 * ║  Cloud        : Firebase Realtime Database                      ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  Required Libraries (install via Arduino Library Manager):      ║
 * ║  • Adafruit SSD1306          (by Adafruit)                      ║
 * ║  • Adafruit GFX Library      (by Adafruit)                      ║
 * ║  • SparkFun MAX3010x          (by SparkFun)                     ║
 * ║  • MPU6050_light              (by rfetick)                      ║
 * ║  • DallasTemperature          (by Miles Burton)                 ║
 * ║  • OneWire                    (by Jim Studt)                    ║
 * ║  • TinyGPSPlus                (by Mikal Hart)                   ║
 * ║  • Firebase ESP Client        (by Mobizt) ← search "Firebase   ║
 * ║    ESP Client" in Library Manager                               ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

// ═══════════════════════════════════════════════════════════════════
// 0. INCLUDES & CONFIG
// ═══════════════════════════════════════════════════════════════════

#include "config.h"

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MAX30105.h>
#include <heartRate.h>           // Included with SparkFun MAX3010x
#include <MPU6050_light.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"  // Included with Firebase ESP Client
#include "addons/RTDBHelper.h"   // Included with Firebase ESP Client

// ═══════════════════════════════════════════════════════════════════
// 1. HARDWARE OBJECT DECLARATIONS
// ═══════════════════════════════════════════════════════════════════

Adafruit_SSD1306  display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
MAX30105          heartSensor;
MPU6050           imu(Wire);
OneWire           oneWirebus(PIN_ONE_WIRE);
DallasTemperature tempSensor(&oneWirebus);
TinyGPSPlus       gps;
HardwareSerial    gpsSerial(2);   // UART2

FirebaseData      fbData;
FirebaseAuth      fbAuth;
FirebaseConfig    fbConfig;

// ═══════════════════════════════════════════════════════════════════
// 2. DATA MODEL  — all sensor values live here
// ═══════════════════════════════════════════════════════════════════

struct SensorReadings {
  // Heart Rate & SpO₂
  float    heartRate     = 0;
  bool     fingerPresent = false;

  // Temperature
  float    temperature   = 0;

  // Motion / Fall
  float    accelMag      = 0;  // g-force magnitude

  // Sound
  int      soundLevel    = 0;

  // GPS
  double   gpsLat        = 0.0;
  double   gpsLng        = 0.0;
  bool     gpsValid      = false;
  uint32_t gpsSatellites = 0;
};

SensorReadings readings;

// ═══════════════════════════════════════════════════════════════════
// 3. ALERT STATE MACHINE
// ═══════════════════════════════════════════════════════════════════

enum AlertType {
  ALERT_NONE    = 0,
  ALERT_HR_LOW  = 1,
  ALERT_HR_HIGH = 2,
  ALERT_TEMP    = 3,
  ALERT_FALL    = 4,
  ALERT_SOUND   = 5,
  ALERT_SOS     = 6   // SOS is highest priority — manual clear only
};

struct AlertState {
  bool      active      = false;
  AlertType type        = ALERT_NONE;
  String    label       = "";
  uint32_t  startedAt   = 0;
};

AlertState alertState;

// ═══════════════════════════════════════════════════════════════════
// 4. INTERNAL TIMING  (non-blocking millis() approach)
// ═══════════════════════════════════════════════════════════════════

uint32_t tLastScreen     = 0;
uint32_t tLastTemp       = 0;
uint32_t tLastFirebase   = 0;
uint32_t tLastFall       = 0;
uint32_t tBuzzerToggle   = 0;
bool     buzzerBeepState = false;

uint8_t  currentScreen   = 0;   // 0 = Environment, 1 = Biometrics, 2 = GPS/Status

bool     displayOk       = false;
bool     hrSensorOk      = false;
bool     imuOk           = false;
bool     wifiConnected   = false;
bool     firebaseReady   = false;

// ═══════════════════════════════════════════════════════════════════
// 5. HEART RATE ROLLING AVERAGE
// ═══════════════════════════════════════════════════════════════════

byte     hrSamples[HR_SAMPLE_WINDOW];
uint8_t  hrSampleIdx  = 0;
uint8_t  hrSamplesFilled = 0;
uint32_t hrLastBeat   = 0;

// ═══════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════

// Init
void initOLED();
void initHeartSensor();
void initIMU();
void initTempSensor();
void initGPS();
void initWiFiAndFirebase();

// Reads
void readHeartRate();
void readTemperature();
void readIMU();
void readSound();
void readGPS();
void checkSOSButton();

// Alerts
void evaluateAlerts();
void raiseAlert(AlertType t, const String& label);
void clearAlert();
void handleBuzzer();

// Display
void drawOLED();
void drawSplashScreen();
void drawEnvironmentScreen();
void drawBiometricsScreen();
void drawGPSStatusScreen();
void drawAlertOverlay();
void drawHeader(const char* title);
void drawStatusBar();

// Firebase
void pushToFirebase(bool isEmergency = false);

// Utilities
String alertTypeToString(AlertType t);
bool   isValidHR(float hr);
bool   isValidTemp(float t);

// ═══════════════════════════════════════════════════════════════════
// 6. SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // ── OLED DIRECT TEST ──
  Adafruit_SSD1306 testDisplay(128, 32, &Wire, -1);
  if (testDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    testDisplay.clearDisplay();
    testDisplay.setTextSize(1);
    testDisplay.setTextColor(SSD1306_WHITE);
    testDisplay.setCursor(0, 10);
    testDisplay.print("CareVibe WORKS");
    testDisplay.display();
    Serial.println("OLED TEST: OK");
  } else {
    Serial.println("OLED TEST: FAILED");
  }
  delay(3000);
  // ── END TEST ──

  // ... rest of setup continues normally
void setup() {
  Serial.begin(115200);
  Serial.println("\n╔═══════════════════════════╗");
  Serial.println(  "║  CareVibe v2.0 Booting... ║");
  Serial.println(  "╚═══════════════════════════╝");

  // GPIO
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_BUZZER, LOW);

  // I2C Bus — explicitly set pins
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);  // 400kHz Fast Mode

  // Init each subsystem
  initOLED();
  initHeartSensor();
  initIMU();
  initTempSensor();
  initGPS();
  initWiFiAndFirebase();

  Serial.println("[CareVibe] Boot complete.\n");
}

// ═══════════════════════════════════════════════════════════════════
// 7. MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
  uint32_t now = millis();

  // ── High-frequency reads (every loop tick) ──────────────────────
  readHeartRate();   // MAX30102 needs constant polling
  readIMU();         // MPU fall detection
  readSound();       // Mic sampling
  readGPS();         // TinyGPS feed

  // ── Timed reads ─────────────────────────────────────────────────
  if (now - tLastTemp >= INTERVAL_TEMP) {
    tLastTemp = now;
    readTemperature();
  }

  // ── SOS button ──────────────────────────────────────────────────
  checkSOSButton();

  // ── Alert logic ─────────────────────────────────────────────────
  evaluateAlerts();
  handleBuzzer();

  // ── Display ─────────────────────────────────────────────────────
  // Auto-rotate screens (only when no alert)
  if (!alertState.active && (now - tLastScreen >= INTERVAL_SCREEN)) {
    tLastScreen   = now;
    currentScreen = (currentScreen + 1) % 3;
  }
  drawOLED();

  // ── Firebase push ───────────────────────────────────────────────
  if (firebaseReady && (now - tLastFirebase >= INTERVAL_FIREBASE)) {
    tLastFirebase = now;
    pushToFirebase(false);
  }
}

// ═══════════════════════════════════════════════════════════════════
// 8. INITIALIZATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) {
    Serial.println("[OLED] ✗ Not found at 0x3C — check SDA/SCL wiring");
    displayOk = false;
    return;
  }
  displayOk = true;
  drawSplashScreen();
  Serial.println("[OLED] ✓");
}

void initHeartSensor() {
  if (!heartSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[HR] ✗ MAX30102 not found at 0x57");
    hrSensorOk = false;
    return;
  }
  // Conservative setup for accurate readings
  heartSensor.setup(
    60,    // LED brightness (0=off to 255=50mA)
    4,     // Number of samples averaged per reading
    2,     // LED mode (1=Red only, 2=Red+IR, 3=Red+IR+Green)
    200,   // Sample rate (samples/second)
    411,   // Pulse width (microseconds, wider = more precise)
    4096   // ADC range
  );
  heartSensor.setPulseAmplitudeRed(0x0A);   // Low red LED — SpO2 needs both
  heartSensor.setPulseAmplitudeIR(0x1F);    // Higher IR for HR detection
  hrSensorOk = true;
  Serial.println("[HR] ✓ MAX30102 OK");
}

void initIMU() {
  // MPU6050_light auto-detects MPU6050 and MPU6500
  byte status = imu.begin();
  if (status != 0) {
    Serial.println("[IMU] ✗ MPU failed (status=" + String(status) + ")");
    imuOk = false;
    return;
  }
  Serial.println("[IMU] Calibrating — keep device still for 2 seconds...");
  if (displayOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 6);
    display.println("IMU Calibrating...");
    display.setCursor(16, 18);
    display.println("Keep STILL 2s");
    display.display();
  }
  delay(2000);
  imu.calcOffsets(true, true);  // Calc gyro + accel offsets
  imuOk = true;
  Serial.println("[IMU] ✓ MPU calibrated");
}

void initTempSensor() {
  tempSensor.begin();
  tempSensor.setResolution(12);            // 12-bit = 0.0625°C resolution
  tempSensor.setWaitForConversion(false);  // Non-blocking — CRITICAL
  tempSensor.requestTemperatures();        // Kick off first conversion
  Serial.println("[TEMP] ✓ DS18B20 OK");
}

void initGPS() {
  // GPS baud rate is always 9600 by default on NEO-6M
  gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("[GPS] ✓ NEO-6M started on UART2 (may take 1–3 min for lock)");
}

void initWiFiAndFirebase() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] Connecting to " + String(WIFI_SSID));

  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] ✗ Could not connect — running in offline mode");
    wifiConnected  = false;
    firebaseReady  = false;
    return;
  }

  wifiConnected = true;
  Serial.println("\n[WiFi] ✓ Connected: " + WiFi.localIP().toString());

  // Firebase setup
  fbConfig.api_key           = FIREBASE_API_KEY;
  fbConfig.database_url      = FIREBASE_HOST;
  fbConfig.token_status_callback = tokenStatusCallback;  // From TokenHelper.h

  // Anonymous auth (no login required for open RTDB rules)
  // To add email/pass: set fbAuth.user.email and fbAuth.user.password
  if (Firebase.signUp(&fbConfig, &fbAuth, "", "")) {
    Serial.println("[Firebase] ✓ Auth OK");
    firebaseReady = true;
  } else {
    Serial.println("[Firebase] ✗ Auth failed: " + String(fbConfig.signer.signupError.message.c_str()));
    firebaseReady = false;
    return;
  }

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

  // Write device online status
  String path = "/carevibe/" + String(DEVICE_ID) + "/status";
  Firebase.RTDB.setString(&fbData, path.c_str(), "online");
  Serial.println("[Firebase] ✓ Connected to RTDB");
}

// ═══════════════════════════════════════════════════════════════════
// 9. SENSOR READ FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

/**
 * readHeartRate()
 * Polls the MAX30102 FIFO buffer. Uses SparkFun beat-detection
 * algorithm and a rolling average for smooth output.
 * MUST be called every loop tick.
 */
void readHeartRate() {
  if (!hrSensorOk) return;

  heartSensor.check(); // Read FIFO into library buffer

  while (heartSensor.available()) {
    long irValue  = heartSensor.getIR();
    long redValue = heartSensor.getRed();

    // Finger detection: IR > threshold means finger is present
    readings.fingerPresent = (irValue > IR_FINGER_MIN);

    if (readings.fingerPresent) {
      // SparkFun's beat detection
      if (checkForBeat(irValue)) {
        long delta         = millis() - hrLastBeat;
        hrLastBeat         = millis();
        float instantBPM   = 60.0f / (delta / 1000.0f);

        // Sanity check the instant reading before adding to average
        if (instantBPM >= HR_VALID_MIN && instantBPM <= HR_VALID_MAX) {
          hrSamples[hrSampleIdx] = (byte)instantBPM;
          hrSampleIdx            = (hrSampleIdx + 1) % HR_SAMPLE_WINDOW;
          if (hrSamplesFilled < HR_SAMPLE_WINDOW) hrSamplesFilled++;

          // Rolling average over filled samples only
          float sum = 0;
          for (uint8_t i = 0; i < hrSamplesFilled; i++) sum += hrSamples[i];
          readings.heartRate = sum / hrSamplesFilled;
        }
      }
    } else {
      // No finger — reset everything so we don't hold stale values
      readings.heartRate    = 0;
      hrSamplesFilled       = 0;
      hrSampleIdx           = 0;
    }

    heartSensor.nextSample();
  }
}

/**
 * readTemperature()
 * Retrieves the last completed DS18B20 conversion, then immediately
 * queues the next one. setWaitForConversion(false) means getTempC()
 * returns instantly and never blocks.
 */
void readTemperature() {
  float t = tempSensor.getTempCByIndex(0);
  tempSensor.requestTemperatures();  // Queue next (completes in ~750ms in background)

  if (isValidTemp(t)) {
    readings.temperature = t;
  }
  // If invalid (DEVICE_DISCONNECTED = -127), keep last good value
}

/**
 * readIMU()
 * Updates MPU angles and reads raw acceleration.
 * Computes g-force magnitude for fall detection.
 */
void readIMU() {
  if (!imuOk) return;

  imu.update();
  float ax = imu.getAccX();
  float ay = imu.getAccY();
  float az = imu.getAccZ();

  readings.accelMag = sqrt(ax * ax + ay * ay + az * az);
}

/**
 * readSound()
 * Samples the ADC 20 times and takes the peak.
 * Applies an IIR low-pass filter to smooth out spike noise.
 * Smoothing factor: α = 0.3 (lower = smoother but more lag)
 */
void readSound() {
  int peak = 0;
  for (uint8_t i = 0; i < 20; i++) {
    int v = analogRead(PIN_MIC);
    if (v > peak) peak = v;
  }
  // IIR filter: smooth = (old * (1-α)) + (new * α)
  readings.soundLevel = (int)(readings.soundLevel * 0.7f + peak * 0.3f);
}

/**
 * readGPS()
 * Feeds all available bytes into TinyGPSPlus.
 * Location is marked valid only when NMEA reports a fix.
 */
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    readings.gpsLat        = gps.location.lat();
    readings.gpsLng        = gps.location.lng();
    readings.gpsValid      = true;
    readings.gpsSatellites = gps.satellites.value();
  }
}

/**
 * checkSOSButton()
 * Edge-detection (HIGH→LOW) to avoid repeated triggers.
 * SOS alert is the highest priority and must be manually cleared.
 */
void checkSOSButton() {
  static bool prevState = HIGH;
  bool curState = digitalRead(PIN_BUTTON);

  // Falling edge = button just pressed
  if (prevState == HIGH && curState == LOW) {
    raiseAlert(ALERT_SOS, "SOS PRESSED");
    pushToFirebase(true);  // Immediate emergency push
  }
  prevState = curState;
}

// ═══════════════════════════════════════════════════════════════════
// 10. ALERT ENGINE
// ═══════════════════════════════════════════════════════════════════

/**
 * evaluateAlerts()
 * Checks all sensor readings against thresholds.
 * Alerts are prioritized: SOS > HR > Temp > Fall > Sound.
 * A cooldown prevents constant re-triggering on borderline values.
 */
void evaluateAlerts() {
  // SOS is cleared only manually (via button press or Firebase)
  if (alertState.active && alertState.type == ALERT_SOS) return;

  // Cooldown: don't re-evaluate a non-SOS alert too rapidly
  if (alertState.active && (millis() - alertState.startedAt < ALERT_COOLDOWN_MS)) return;

  // ── Heart Rate ────────────────────────────────────────────────
  if (readings.fingerPresent && readings.heartRate > 0) {
    if (readings.heartRate < HR_LOW_ALERT) {
      raiseAlert(ALERT_HR_LOW, "LOW HEART RATE");
      return;
    }
    if (readings.heartRate > HR_HIGH_ALERT) {
      raiseAlert(ALERT_HR_HIGH, "HIGH HEART RATE");
      return;
    }
  }

  // ── Temperature ───────────────────────────────────────────────
  if (readings.temperature > 0 && readings.temperature > TEMP_ALERT) {
    raiseAlert(ALERT_TEMP, "HIGH TEMP " + String(readings.temperature, 1) + "C");
    return;
  }

  // ── Fall Detection ────────────────────────────────────────────
  // Debounce: only trigger if not already triggered within FALL_DEBOUNCE_MS
  if (readings.accelMag > FALL_THRESHOLD && (millis() - tLastFall > FALL_DEBOUNCE_MS)) {
    tLastFall = millis();
    raiseAlert(ALERT_FALL, "FALL DETECTED");
    return;
  }

  // ── Loud Sound ────────────────────────────────────────────────
  if (readings.soundLevel > SOUND_THRESHOLD) {
    raiseAlert(ALERT_SOUND, "LOUD SOUND");
    return;
  }

  // ── All Clear ────────────────────────────────────────────────
  clearAlert();
}

void raiseAlert(AlertType t, const String& label) {
  // Don't overwrite a higher-priority active alert
  if (alertState.active) {
    if (alertState.type == ALERT_SOS) return;
    if (alertState.type == t)         return;  // Same type already active
  }

  alertState.active    = true;
  alertState.type      = t;
  alertState.label     = label;
  alertState.startedAt = millis();

  // Lock OLED to status screen during alert
  currentScreen = 2;

  Serial.println("[ALERT] ▲ " + label + " (type=" + String(t) + ")");
}

void clearAlert() {
  if (alertState.active) {
    Serial.println("[ALERT] ✓ Cleared: " + alertState.label);
    alertState.active = false;
    alertState.type   = ALERT_NONE;
    alertState.label  = "";
    digitalWrite(PIN_BUZZER, LOW);
  }
}

/**
 * handleBuzzer()
 * Non-blocking intermittent beep pattern during active alert.
 * SOS = fast beeps. Others = slow beeps.
 */
void handleBuzzer() {
  if (!alertState.active) {
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }

  uint32_t now = millis();
  uint32_t onTime  = (alertState.type == ALERT_SOS) ? 150  : INTERVAL_BUZZER_ON;
  uint32_t offTime = (alertState.type == ALERT_SOS) ? 150  : INTERVAL_BUZZER_OFF;

  uint32_t elapsed = now - tBuzzerToggle;
  if (buzzerBeepState && elapsed >= onTime) {
    buzzerBeepState = false;
    tBuzzerToggle   = now;
    digitalWrite(PIN_BUZZER, LOW);
  } else if (!buzzerBeepState && elapsed >= offTime) {
    buzzerBeepState = true;
    tBuzzerToggle   = now;
    digitalWrite(PIN_BUZZER, HIGH);
  }
}

// ═══════════════════════════════════════════════════════════════════
// 11. OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════

void drawOLED() {
  if (!displayOk) return;
  display.clearDisplay();

  if (alertState.active) {
    drawAlertOverlay();
  } else {
    display.invertDisplay(false);  // Restore normal display if was inverted
    switch (currentScreen) {
      case 0: drawEnvironmentScreen(); break;
      case 1: drawBiometricsScreen();  break;
      case 2: drawGPSStatusScreen();   break;
    }
  }

  display.display();
}

/** Splash screen on boot — designed for 128x32 */
void drawSplashScreen() {
  display.clearDisplay();
  display.invertDisplay(false);
  display.setTextColor(SSD1306_WHITE);

  display.drawRect(0, 0, 128, 32, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 3);
  display.print("CareVibe");

  display.setTextSize(1);
  display.setCursor(14, 22);
  display.print("Health Wearable v2");
  display.display();
  delay(2000);
}

/** White header bar — fits 128x32 display (9px tall) */
void drawHeader(const char* title) {
  display.fillRect(0, 0, 128, 9, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 1);
  display.print(title);
  display.setTextColor(SSD1306_WHITE);
}

/** Bottom status bar — fits 128x32 (y=24 to 31) */
void drawStatusBar() {
  display.drawLine(0, 24, 128, 24, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 25);
  display.print(wifiConnected ? "WiFi" : "NoWiFi");
  display.setCursor(74, 25);
  display.print(alertState.active ? "! ALERT" : "  SAFE ");
}

/** Screen 0: Environment — Temp + Sound (128x32) */
void drawEnvironmentScreen() {
  drawHeader("ENVIRONMENT");

  // Temperature — left side
  display.setTextSize(1);
  display.setCursor(0, 11);
  display.print("Tmp:");
  if (readings.temperature > 0) {
    display.setTextSize(2);
    display.setCursor(24, 10);
    display.print(readings.temperature, 1);
    display.setTextSize(1);
    display.print("C");
  } else {
    display.setCursor(24, 11);
    display.print("---");
  }

  // Sound — right side compact
  display.setTextSize(1);
  display.setCursor(80, 11);
  display.print("Snd:");
  display.setCursor(80, 19);
  display.print(readings.soundLevel);

  drawStatusBar();
}

/** Screen 1: Biometrics — HR + Motion (128x32) */
void drawBiometricsScreen() {
  drawHeader("BIOMETRICS");

  // Heart Rate — left side
  display.setTextSize(1);
  display.setCursor(0, 11);
  display.print("HR:");
  if (!readings.fingerPresent) {
    display.setCursor(18, 11);
    display.print("No finger");
  } else if (readings.heartRate == 0) {
    display.setCursor(18, 11);
    display.print("Measuring..");
  } else {
    display.setTextSize(2);
    display.setCursor(18, 10);
    display.print((int)readings.heartRate);
    display.setTextSize(1);
    display.print("bpm");
  }

  // Accel — bottom left
  display.setTextSize(1);
  display.setCursor(0, 19);
  display.print("G:");
  display.print(readings.accelMag, 2);

  drawStatusBar();
}

/** Screen 2: GPS + Status (128x32) */
void drawGPSStatusScreen() {
  drawHeader("GPS / STATUS");

  display.setTextSize(1);
  if (readings.gpsValid) {
    display.setCursor(0, 11);
    display.print("Lat:");
    display.print(readings.gpsLat, 4);
    display.setCursor(0, 20);
    display.print("Lng:");
    display.print(readings.gpsLng, 4);
  } else {
    display.setCursor(0, 11);
    display.print("GPS: Searching...");
    display.setCursor(0, 20);
    display.print("Sats:");
    display.print(readings.gpsSatellites);
  }

  drawStatusBar();
}

/** Alert overlay — 128x32 inverted display */
void drawAlertOverlay() {
  display.invertDisplay(true);

  display.setTextSize(1);
  display.setCursor(20, 1);
  display.print("!! EMERGENCY !!");

  display.setCursor(0, 12);
  display.print(alertState.label.substring(0, 21));

  display.setCursor(0, 23);
  if (readings.gpsValid) {
    display.print(readings.gpsLat, 3);
    display.print(",");
    display.print(readings.gpsLng, 3);
  } else {
    display.print("GPS:No Fix  WiFi:");
    display.print(wifiConnected ? "OK" : "No");
  }
}

// ═══════════════════════════════════════════════════════════════════
// 12. FIREBASE
// ═══════════════════════════════════════════════════════════════════

/**
 * pushToFirebase()
 * Writes a snapshot of all sensor data to:
 *   /carevibe/{DEVICE_ID}/latest        ← always overwritten
 *   /carevibe/{DEVICE_ID}/alerts        ← appended (push = auto-key) on emergency
 *
 * isEmergency = true → also writes to alerts log immediately
 */
void pushToFirebase(bool isEmergency) {
  if (!firebaseReady || !Firebase.ready()) return;
  if (WiFi.status() != WL_CONNECTED)       return;

  String basePath = "/carevibe/" + String(DEVICE_ID);

  // ── Build the JSON payload ────────────────────────────────────
  FirebaseJson payload;
  payload.set("heartRate",     readings.heartRate);
  payload.set("fingerPresent", readings.fingerPresent);
  payload.set("temperature",   readings.temperature);
  payload.set("accelMag",      readings.accelMag);
  payload.set("soundLevel",    readings.soundLevel);
  payload.set("gpsLat",        readings.gpsLat);
  payload.set("gpsLng",        readings.gpsLng);
  payload.set("gpsValid",      readings.gpsValid);
  payload.set("gpsSatellites", (int)readings.gpsSatellites);
  payload.set("alertActive",   alertState.active);
  payload.set("alertType",     (int)alertState.type);
  payload.set("alertLabel",    alertState.label);
  payload.set("wifiRSSI",      WiFi.RSSI());
  payload.set("uptimeMs",      (int)millis());

  // Always overwrite /latest
  if (!Firebase.RTDB.setJSON(&fbData, (basePath + "/latest").c_str(), &payload)) {
    Serial.println("[Firebase] Push failed: " + fbData.errorReason());
    return;
  }

  // Append to /alerts log only on emergency
  if (isEmergency || alertState.active) {
    FirebaseJson alertEntry;
    alertEntry.set("type",      (int)alertState.type);
    alertEntry.set("label",     alertState.label);
    alertEntry.set("gpsLat",    readings.gpsLat);
    alertEntry.set("gpsLng",    readings.gpsLng);
    alertEntry.set("gpsValid",  readings.gpsValid);
    alertEntry.set("temp",      readings.temperature);
    alertEntry.set("heartRate", readings.heartRate);
    alertEntry.set("uptimeMs",  (int)millis());

    Firebase.RTDB.pushJSON(&fbData, (basePath + "/alerts").c_str(), &alertEntry);
    Serial.println("[Firebase] ▲ Alert logged to /alerts");
  }
}

// ═══════════════════════════════════════════════════════════════════
// 13. UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

bool isValidHR(float hr) {
  return (hr >= HR_VALID_MIN && hr <= HR_VALID_MAX);
}

bool isValidTemp(float t) {
  return (t >= TEMP_VALID_MIN && t <= TEMP_VALID_MAX);
}

String alertTypeToString(AlertType t) {
  switch (t) {
    case ALERT_HR_LOW:  return "LOW_HR";
    case ALERT_HR_HIGH: return "HIGH_HR";
    case ALERT_TEMP:    return "HIGH_TEMP";
    case ALERT_FALL:    return "FALL";
    case ALERT_SOUND:   return "LOUD_SOUND";
    case ALERT_SOS:     return "SOS";
    default:            return "NONE";
  }
}

// ═══════════════════════════════════════════════════════════════════
// END OF FILE
// ═══════════════════════════════════════════════════════════════════
