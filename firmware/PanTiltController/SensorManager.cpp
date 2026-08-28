#include "SensorManager.h"
#include "MotionController.h"
#include <Wire.h>

SensorManager::SensorManager() :
    bno(BNO055_DEFAULT_ADDR),
    lastImuReadTime(0),
    homingState(HOMING_IDLE),
    homingStartTime(0),
    homingPanRequested(false),
    homingTiltRequested(false),
    panDebounceCounter(0),
    tiltDebounceCounter(0)
{
    readings.panLimitPressed = false;
    readings.tiltLimitPressed = false;
    readings.imuAvailable = false;
    readings.imuPitch = 0.0f;
    readings.imuRoll = 0.0f;
    readings.imuYaw = 0.0f;
    readings.calSys = 0;
    readings.calGyro = 0;
    readings.calAccel = 0;
    readings.calMag = 0;
    readings.temperature = 25.0f;
}

void SensorManager::init() {
    pinMode(PIN_PAN_LIMIT, INPUT_PULLUP);
    pinMode(PIN_TILT_LIMIT, INPUT_PULLUP);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    // Initial limit switch read
    readings.panLimitPressed = (digitalRead(PIN_PAN_LIMIT) == LOW);
    readings.tiltLimitPressed = (digitalRead(PIN_TILT_LIMIT) == LOW);

    // Initialize BNO055 IMU via I2C
    readings.imuAvailable = bno.begin(BNO055_OPR_NDOF);
    if (readings.imuAvailable) {
        digitalWrite(PIN_STATUS_LED, HIGH); // Light status LED when IMU detected!
    }
}

void SensorManager::updateLimitSwitches() {
    // Debounce readings (active LOW when switch triggers to GND)
    bool panRaw = (digitalRead(PIN_PAN_LIMIT) == LOW);
    bool tiltRaw = (digitalRead(PIN_TILT_LIMIT) == LOW);

    if (panRaw) {
        if (panDebounceCounter < 5) panDebounceCounter++;
    } else {
        if (panDebounceCounter > 0) panDebounceCounter--;
    }
    readings.panLimitPressed = (panDebounceCounter >= 3);

    if (tiltRaw) {
        if (tiltDebounceCounter < 5) tiltDebounceCounter++;
    } else {
        if (tiltDebounceCounter > 0) tiltDebounceCounter--;
    }
    readings.tiltLimitPressed = (tiltDebounceCounter >= 3);
}

void SensorManager::updateAuxSensors() {
    unsigned long now = millis();
    if (now - lastImuReadTime < 50UL) return; // 20 Hz update rate
    lastImuReadTime = now;

    if (!readings.imuAvailable) {
        // Attempt periodic reconnection every 2 seconds if sensor was plugged in late
        static unsigned long lastRetry = 0;
        if (now - lastRetry > 2000UL) {
            lastRetry = now;
            readings.imuAvailable = bno.begin(BNO055_OPR_NDOF);
        }
        return;
    }

    BNO055_Orientation ori;
    if (bno.readEuler(ori) && ori.valid) {
        readings.imuYaw   = ori.heading;
        readings.imuRoll  = ori.roll;
        readings.imuPitch = ori.pitch;
    }

    BNO055_Calibration cal;
    if (bno.readCalibration(cal)) {
        readings.calSys   = cal.sys;
        readings.calGyro  = cal.gyro;
        readings.calAccel = cal.accel;
        readings.calMag   = cal.mag;
    }
}

BNO055_Driver& SensorManager::getIMU() {
    return bno;
}

bool SensorManager::startHoming(MotionController &motion, bool homePan, bool homeTilt) {
    if (homingState != HOMING_IDLE) return false;

    homingPanRequested = homePan;
    homingTiltRequested = homeTilt;
    homingStartTime = millis();

    // Temporarily disable soft limits while homing
    motion.setSoftLimitsEnabled(false);

    if (homingPanRequested) {
        homingState = HOMING_PAN_SEARCH;
        motion.setJogVelocity(-HOMING_SPEED_PAN, 0.0f);
    } else if (homingTiltRequested) {
        homingState = HOMING_TILT_SEARCH;
        motion.setJogVelocity(0.0f, -HOMING_SPEED_TILT);
    } else {
        homingState = HOMING_COMPLETE;
    }

    return true;
}

void SensorManager::cancelHoming(MotionController &motion) {
    if (homingState != HOMING_IDLE) {
        motion.stop();
        motion.setSoftLimitsEnabled(SOFT_LIMITS_ENABLED);
        homingState = HOMING_IDLE;
    }
}

bool SensorManager::isHomingActive() const {
    return (homingState != HOMING_IDLE && homingState != HOMING_COMPLETE && homingState != HOMING_FAILED);
}

HomingState SensorManager::getHomingState() const {
    return homingState;
}

bool SensorManager::isPanLimitPressed() const {
    return readings.panLimitPressed;
}

bool SensorManager::isTiltLimitPressed() const {
    return readings.tiltLimitPressed;
}

const SensorReadings& SensorManager::getReadings() const {
    return readings;
}

void SensorManager::processHomingStateMachine(MotionController &motion) {
    if (homingState == HOMING_IDLE || homingState == HOMING_COMPLETE || homingState == HOMING_FAILED) {
        return;
    }

    // Safety timeout check
    if (millis() - homingStartTime > HOMING_TIMEOUT_MS) {
        motion.emergencyStop();
        motion.setSoftLimitsEnabled(SOFT_LIMITS_ENABLED);
        homingState = HOMING_FAILED;
        return;
    }

    switch (homingState) {
        case HOMING_PAN_SEARCH:
            if (readings.panLimitPressed) {
                motion.stop();
                motion.setPosition(0.0f, motion.getTiltDeg());
                // Back off slightly from switch
                motion.moveRel(3.0f, 0.0f);
                homingState = HOMING_PAN_BACKOFF;
            }
            break;

        case HOMING_PAN_BACKOFF:
            if (!motion.isMoving()) {
                // Pan zeroed and backed off
                motion.setPosition(0.0f, motion.getTiltDeg());
                if (homingTiltRequested) {
                    homingState = HOMING_TILT_SEARCH;
                    motion.setJogVelocity(0.0f, -HOMING_SPEED_TILT);
                } else {
                    motion.setSoftLimitsEnabled(SOFT_LIMITS_ENABLED);
                    homingState = HOMING_COMPLETE;
                }
            }
            break;

        case HOMING_TILT_SEARCH:
            if (readings.tiltLimitPressed) {
                motion.stop();
                motion.setPosition(motion.getPanDeg(), 0.0f);
                // Back off slightly from switch
                motion.moveRel(0.0f, 3.0f);
                homingState = HOMING_TILT_BACKOFF;
            }
            break;

        case HOMING_TILT_BACKOFF:
            if (!motion.isMoving()) {
                // Tilt zeroed and backed off
                motion.setPosition(motion.getPanDeg(), 0.0f);
                motion.setSoftLimitsEnabled(SOFT_LIMITS_ENABLED);
                homingState = HOMING_COMPLETE;
            }
            break;

        default:
            break;
    }
}

void SensorManager::update(MotionController &motion) {
    updateLimitSwitches();
    updateAuxSensors();
    processHomingStateMachine(motion);
}

