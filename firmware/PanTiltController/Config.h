#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// HARDWARE PIN ASSIGNMENTS (Arduino Uno)
// ============================================================================

// Pan Axis (Azimuth / Yaw) - Connected to STP-DRV-6575 Drive #1
#define PIN_PAN_STEP          2     // Step / Pulse output (Active HIGH)
#define PIN_PAN_DIR           3     // Direction output
#define PIN_PAN_ENABLE        4     // Enable output (Optional, STP-DRV-6575 EN input)

// Tilt Axis (Altitude / Pitch) - Connected to STP-DRV-6575 Drive #2
#define PIN_TILT_STEP         5     // Step / Pulse output (Active HIGH)
#define PIN_TILT_DIR          6     // Direction output
#define PIN_TILT_ENABLE       7     // Enable output (Optional, STP-DRV-6575 EN input)

// Endstop / Limit Switches (Optional)
#define PIN_PAN_LIMIT         8     // Pan Home/Limit switch (Active LOW with internal pull-up)
#define PIN_TILT_LIMIT        9     // Tilt Home/Limit switch (Active LOW with internal pull-up)

// Auxiliary & Status
#define PIN_STATUS_LED        13    // On-board LED indicator
#define PIN_I2C_SDA           A4    // I2C Data for future sensors (MPU6050, BNO055, etc.)
#define PIN_I2C_SCL           A5    // I2C Clock for future sensors

// ============================================================================
// STEPPER MOTOR & DRIVE CONFIGURATION (STP-DRV-6575)
// ============================================================================

// Signal polarity for STP-DRV-6575 optoisolated inputs
// In common-cathode wiring: STEP-, DIR-, EN- to GND; STEP+, DIR+, EN+ to Arduino pins.
#define STEP_PULSE_ACTIVE_HIGH  true
#define DIR_INVERT_PAN          true   // Inverted per physical platform test
#define DIR_INVERT_TILT         true   // Inverted per physical platform test
#define ENABLE_ACTIVE_LOW       true   // STP-DRV-6575: LOW (opto off) = ENABLED, HIGH (opto on) = DISABLED
// Minimum pulse width in microseconds required by STP-DRV-6575 (spec min is 1.0 us, 2-5 us is safe)
#define STEP_PULSE_WIDTH_US     5

// Motor physical specs
#define PAN_MOTOR_STEPS_PER_REV   200.0f  // 1.8 degree stepper
#define TILT_MOTOR_STEPS_PER_REV  200.0f  // 1.8 degree stepper

// STP-DRV-6575 Microstepping switch setting (10x = 2,000 steps/rev)
#define PAN_MICROSTEPS            10.0f   // 2,000 steps/rev (SW5=OFF, SW6=ON, SW7=OFF)
#define TILT_MICROSTEPS           10.0f   // 2,000 steps/rev

// Mechanical Gear Reduction (Motor revs per 1 axis rev. 1.0 for direct drive)
#define PAN_GEAR_RATIO            1.0f    // Adjust if belt/gear reduction is used
#define TILT_GEAR_RATIO           1.0f    // Adjust if belt/gear reduction is used

// Calculated Steps per Degree
// steps_per_deg = (steps_per_rev * microsteps * gear_ratio) / 360.0
#define PAN_STEPS_PER_DEG         ((PAN_MOTOR_STEPS_PER_REV * PAN_MICROSTEPS * PAN_GEAR_RATIO) / 360.0f)
#define TILT_STEPS_PER_DEG        ((TILT_MOTOR_STEPS_PER_REV * TILT_MICROSTEPS * TILT_GEAR_RATIO) / 360.0f)

// ============================================================================
// MOTION DEFAULTS & TRAVEL LIMITS (in Degrees)
// ============================================================================

// Default & Maximum Velocities (Degrees / sec)
#define DEFAULT_MAX_SPEED_PAN     45.0f   // deg/sec
#define DEFAULT_MAX_SPEED_TILT    30.0f   // deg/sec
#define ABSOLUTE_MAX_SPEED_PAN    180.0f  // Upper safety clamp
#define ABSOLUTE_MAX_SPEED_TILT   120.0f  // Upper safety clamp

// Default Acceleration (Degrees / sec^2)
#define DEFAULT_ACCEL_PAN         90.0f   // deg/sec^2
#define DEFAULT_ACCEL_TILT        60.0f   // deg/sec^2

// Soft Travel Limits (in Degrees)
#define SOFT_LIMITS_ENABLED       true
#define PAN_MIN_DEG               -180.0f
#define PAN_MAX_DEG               180.0f
#define TILT_MIN_DEG              -45.0f  // Below horizon
#define TILT_MAX_DEG              90.0f   // Zenith (straight up)

// Homing Speeds & Direction
#define HOMING_SPEED_PAN          15.0f   // deg/sec
#define HOMING_SPEED_TILT         10.0f   // deg/sec
#define HOMING_TIMEOUT_MS         15000   // 15 seconds max homing timeout

// ============================================================================
// COMMUNICATION & TELEMETRY
// ============================================================================

#define SERIAL_BAUD_RATE          115200
#define SERIAL_RX_BUFFER_SIZE     128

// Telemetry streaming frequency (Hz). Set to 0 to disable auto-streaming.
#define DEFAULT_TELEMETRY_RATE_HZ 10      // 10 Hz = every 100 ms

#endif // CONFIG_H

