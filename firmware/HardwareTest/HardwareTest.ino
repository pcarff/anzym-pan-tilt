/**
 * HardwareTest.ino
 * 
 * Simple, standalone hardware test script to isolate and verify:
 * - Pan Motor (D2 Step, D3 Dir) in both Forward and Reverse
 * - Tilt Motor (D5 Step, D6 Dir) in both Forward and Reverse
 * - Optocoupler timing with generous pulse widths and delays
 * - Continuous serial diagnostic output (115200 baud)
 */

#include <Arduino.h>

// Pan Axis Pins
const int PAN_STEP_PIN = 2;
const int PAN_DIR_PIN  = 3;

// Tilt Axis Pins
const int TILT_STEP_PIN = 5;
const int TILT_DIR_PIN  = 6;

// Status LED
const int LED_PIN = 13;

// Step pulse settings
const int PULSE_WIDTH_US = 20;     // 20 microseconds step pulse
const int STEP_DELAY_US  = 1200;   // ~800 steps/sec (gentle visible speed)
const int TEST_STEPS     = 800;    // Number of steps per test move (~180 deg or 1/2 turn)

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait for serial connection

    pinMode(PAN_STEP_PIN, OUTPUT);
    pinMode(PAN_DIR_PIN, OUTPUT);
    pinMode(TILT_STEP_PIN, OUTPUT);
    pinMode(TILT_DIR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(PAN_STEP_PIN, LOW);
    digitalWrite(PAN_DIR_PIN, LOW);
    digitalWrite(TILT_STEP_PIN, LOW);
    digitalWrite(TILT_DIR_PIN, LOW);

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("  Pan-Tilt Hardware Test Program"));
    Serial.println(F("========================================"));
    Serial.println(F("Testing pinouts:"));
    Serial.println(F("  Pan:  STEP=D2, DIR=D3, GND=STEP-/DIR-"));
    Serial.println(F("  Tilt: STEP=D5, DIR=D6, GND=STEP-/DIR-"));
    Serial.println(F("========================================"));
    delay(2000);
}

void stepMotor(int stepPin, int steps) {
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(PULSE_WIDTH_US);
        digitalWrite(stepPin, LOW);
        digitalWrite(LED_PIN, LOW);
        delayMicroseconds(STEP_DELAY_US);
    }
}

void loop() {
    // ------------------------------------------------------------------------
    // TEST 1: Pan Motor - Forward (DIR = LOW)
    // ------------------------------------------------------------------------
    Serial.println(F(">>> [1/4] PAN Motor: DIR pin (D3) = LOW (Forward)..."));
    digitalWrite(PAN_DIR_PIN, LOW);
    delay(50); // Generous DIR setup time
    stepMotor(PAN_STEP_PIN, TEST_STEPS);
    Serial.println(F("    Pan forward completed. Pausing 1.5s..."));
    delay(1500);

    // ------------------------------------------------------------------------
    // TEST 2: Pan Motor - Reverse (DIR = HIGH)
    // ------------------------------------------------------------------------
    Serial.println(F(">>> [2/4] PAN Motor: DIR pin (D3) = HIGH (Reverse)..."));
    digitalWrite(PAN_DIR_PIN, HIGH);
    delay(50); // Generous DIR setup time
    stepMotor(PAN_STEP_PIN, TEST_STEPS);
    Serial.println(F("    Pan reverse completed. Pausing 1.5s..."));
    delay(1500);

    // ------------------------------------------------------------------------
    // TEST 3: Tilt Motor - Forward (DIR = LOW)
    // ------------------------------------------------------------------------
    Serial.println(F(">>> [3/4] TILT Motor: DIR pin (D6) = LOW (Forward)..."));
    digitalWrite(TILT_DIR_PIN, LOW);
    delay(50);
    stepMotor(TILT_STEP_PIN, TEST_STEPS);
    Serial.println(F("    Tilt forward completed. Pausing 1.5s..."));
    delay(1500);

    // ------------------------------------------------------------------------
    // TEST 4: Tilt Motor - Reverse (DIR = HIGH)
    // ------------------------------------------------------------------------
    Serial.println(F(">>> [4/4] TILT Motor: DIR pin (D6) = HIGH (Reverse)..."));
    digitalWrite(TILT_DIR_PIN, HIGH);
    delay(50);
    stepMotor(TILT_STEP_PIN, TEST_STEPS);
    Serial.println(F("    Tilt reverse completed. Pausing 2.5s before next cycle..."));
    delay(2500);

    Serial.println(F("----------------------------------------"));
    Serial.println(F("Cycle complete! Repeating test..."));
    Serial.println(F("----------------------------------------"));
}

