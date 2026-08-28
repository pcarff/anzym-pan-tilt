/**
 * PanTiltController.ino
 * 
 * Dual-axis Pan-Tilt (Alt-Azimuth) Stepper Motor Controller
 * Hardware: Arduino Uno + STP-DRV-6575 Stepper Drives
 * 
 * Provides smooth trapezoidal acceleration, velocity jogging, soft limits,
 * limit switch homing, sensor integration hooks, and high-speed serial telemetry.
 */

#include "Config.h"
#include "MotionController.h"
#include "SensorManager.h"
#include "CommandParser.h"

// Global Controller Instances
MotionController motion;
SensorManager sensors;
CommandParser parser;

// Status LED blinker
unsigned long lastBlinkMillis = 0;
bool ledState = false;

void setup() {
    // 1. Initialize serial communication
    parser.init();
    Serial.println(F("=== Pan-Tilt Controller Initialized ==="));
    Serial.print(F("Pan steps/deg: "));
    Serial.print(PAN_STEPS_PER_DEG, 3);
    Serial.print(F(" | Tilt steps/deg: "));
    Serial.println(TILT_STEPS_PER_DEG, 3);

    // 2. Initialize hardware modules
    motion.init();
    sensors.init();

    // 3. Signal readiness with LED blink
    pinMode(PIN_STATUS_LED, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(60);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(60);
    }
}

void loop() {
    // High-priority non-blocking step generation & motion updates
    motion.update();

    // Sensor debouncing & homing state machine
    sensors.update(motion);

    // Incoming serial command parsing & telemetry output
    parser.processSerial(motion, sensors);

    // Heartbeat LED: Fast blink when moving, slow pulse when idle
    unsigned long now = millis();
    unsigned long blinkInterval = motion.isMoving() ? 100 : 800;
    if (now - lastBlinkMillis >= blinkInterval) {
        lastBlinkMillis = now;
        ledState = !ledState;
        digitalWrite(PIN_STATUS_LED, ledState ? HIGH : LOW);
    }
}

