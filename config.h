/**
 * ╔══════════════════════════════════════════════════════╗
 * ║           CareVibe — config.h                        ║
 * ║  All pins, thresholds, and credentials in one place  ║
 * ╚══════════════════════════════════════════════════════╝
 *
 * INSTRUCTIONS:
 *   1. Fill in your WiFi credentials below.
 *   2. Fill in your Firebase project URL and Web API Key.
 *   3. Change DEVICE_ID if you run multiple units.
 *   4. Tune thresholds to match your test subject's baselines.
 */

#pragma once

// ─────────────────────────────────────────────────────────────
// WiFi & Firebase Credentials (FILL THESE IN)
// ─────────────────────────────────────────────────────────────
#define WIFI_SSID         "wifi"
#define WIFI_PASSWORD     "wifi1234"

// Firebase Realtime Database URL  (e.g. "https://myproject-default-rtdb.firebaseio.com/")
#define FIREBASE_HOST     "https://carevibe-e89d5-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Firebase Web API Key (from Firebase console → Project Settings → General)
#define FIREBASE_API_KEY  "AIzaSyBjXVmkMPst_4LiGMzyBs0QjxeN-6wIjp0"

// Unique ID for this device (used as the DB path prefix)
#define DEVICE_ID         "carevibe_01"

// ─────────────────────────────────────────────────────────────
// Hardware Pin Map
// ─────────────────────────────────────────────────────────────

// I2C (shared by OLED + MAX30102 + MPU6500)
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22

// DS18B20 Temperature (1-Wire)
#define PIN_ONE_WIRE     4

// MAX9814 Microphone (Analog)
#define PIN_MIC         34   // ADC-only pin — do NOT use as output

// GPS NEO-6M (UART via HardwareSerial 2)
#define PIN_GPS_RX      16   // ESP32 RX ← GPS TX
#define PIN_GPS_TX      17   // ESP32 TX → GPS RX (often unused)

// Buzzer (Active, 3.3V logic)
#define PIN_BUZZER      25

// SOS Push Button (wired to GND, uses INPUT_PULLUP)
#define PIN_BUTTON      26

// I2C Device Addresses
#define ADDR_OLED       0x3D
// MAX30102 = 0x57  (fixed, handled by library)
// MPU6500  = 0x68  (fixed, handled by library)

// ─────────────────────────────────────────────────────────────
// OLED Display Dimensions
// ─────────────────────────────────────────────────────────────
#define OLED_WIDTH      128
#define OLED_HEIGHT      32
#define OLED_RESET       -1  // Share reset with ESP32 EN pin

// ─────────────────────────────────────────────────────────────
// Alert Thresholds  ← tune these for your use case
// ─────────────────────────────────────────────────────────────

// Heart Rate (BPM)
#define HR_LOW_ALERT     50   // Below this → bradycardia alert
#define HR_HIGH_ALERT   120   // Above this → tachycardia alert
#define HR_VALID_MIN     20   // Sanity floor  (discard noise below)
#define HR_VALID_MAX    250   // Sanity ceiling (discard noise above)

// SpO2 (%)
#define SPO2_LOW_ALERT   94   // Below this → hypoxia warning
#define SPO2_VALID_MIN   70   // Sanity floor

// Temperature (°C)
#define TEMP_ALERT       38.0f  // Fever threshold
#define TEMP_VALID_MIN   34.0f  // Sanity floor  (not a corpse)
#define TEMP_VALID_MAX   43.0f  // Sanity ceiling (not a volcano)

// Fall Detection (g-force magnitude √(ax²+ay²+az²))
// 1.0g = stationary. 2.5g+ = strong impact.
#define FALL_THRESHOLD    2.5f

// Sound (12-bit ADC value, 0–4095)
#define SOUND_THRESHOLD  3500

// MAX30102: Minimum IR value to confirm finger is placed
#define IR_FINGER_MIN   50000L

// ─────────────────────────────────────────────────────────────
// Timing Intervals (milliseconds)
// ─────────────────────────────────────────────────────────────
#define INTERVAL_SCREEN     3000   // Rotate OLED screens every N ms
#define INTERVAL_TEMP       2000   // DS18B20 read interval
#define INTERVAL_FIREBASE  10000   // Normal periodic push interval
#define INTERVAL_BUZZER_ON   300   // Buzzer ON duration per beep
#define INTERVAL_BUZZER_OFF  700   // Buzzer OFF duration per beep
#define ALERT_COOLDOWN_MS   3000   // Min time between re-evaluating new alerts
#define FALL_DEBOUNCE_MS     800   // Ignore duplicate fall triggers within N ms

// ─────────────────────────────────────────────────────────────
// Heart Rate Averaging
// ─────────────────────────────────────────────────────────────
#define HR_SAMPLE_WINDOW   6  // Number of beats to average (higher = smoother but slower)
