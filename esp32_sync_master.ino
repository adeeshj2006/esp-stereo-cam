/*
 * ESP32 Sync Master for Stereo Camera
 *
 * Generates a periodic TTL pulse on a GPIO pin to synchronize
 * multiple ESP32-CAM modules. Connect this pin to the SYNC_PIN
 * of each camera ESP32 (configured as an input with interrupt
 * on rising edge) to trigger simultaneous frame capture.
 *
 * Default output: 30 Hz square wave (50% duty cycle) on GPIO4.
 * Adjust FREQ_HZ and DUTY_PCT as needed.
 *
 * Upload to an ESP32 (any model) that is NOT the camera.
 */

#include <Arduino.h>

// ------------------- USER CONFIG -------------------
const uint8_t SYNC_PIN = 4;        // GPIO outputting the sync pulse
const float   FREQ_HZ  = 30.0;    // Pulse frequency in Hz
const float   DUTY_PCT = 50.0;    // Duty cycle percentage (0-100)
// --------------------------------------------------

// Derived values
static const uint32_t period_us = (uint32_t)(1000000UL / FREQ_HZ);
static const uint32_t high_time_us = (uint32_t)(period_us * (DUTY_PCT / 100.0));
static const uint32_t low_time_us  = period_us - high_time_us;

// Timing variables
uint32_t lastToggle = 0;
bool     pinState   = false;

void setup() {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  Serial.begin(115200);
  Serial.println("\nESP32 Sync Master starting...");
  Serial.printf("Pin %d - %.2f Hz, %.1f%% duty cycle\n", SYNC_PIN, FREQ_HZ, DUTY_PCT);
  Serial.printf("Period: %d µs (high %d µs, low %d µs)\n", period_us, high_time_us, low_time_us);
}

void loop() {
  uint32_t now = micros();
  if (now - lastToggle >= (pinState ? low_time_us : high_time_us)) {
    pinState = !pinState;
    digitalWrite(SYNC_PIN, pinState ? HIGH : LOW);
    lastToggle = now;
  }
  // Optional: allow other tasks (e.g., serial) to run
  // delay(0);
}