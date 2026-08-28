#include "MotionController.h"
#include <math.h>

MotionController::MotionController() : drivesEnabled(true), softLimitsActive(SOFT_LIMITS_ENABLED) {
}

void MotionController::init() {
    initAxis(panAxis, PIN_PAN_STEP, PIN_PAN_DIR, PIN_PAN_ENABLE,
             DIR_INVERT_PAN, PAN_STEPS_PER_DEG, DEFAULT_MAX_SPEED_PAN,
             DEFAULT_ACCEL_PAN, PAN_MIN_DEG, PAN_MAX_DEG);

    initAxis(tiltAxis, PIN_TILT_STEP, PIN_TILT_DIR, PIN_TILT_ENABLE,
             DIR_INVERT_TILT, TILT_STEPS_PER_DEG, DEFAULT_MAX_SPEED_TILT,
             DEFAULT_ACCEL_TILT, TILT_MIN_DEG, TILT_MAX_DEG);

    setEnabled(true);
}

void MotionController::initAxis(AxisState &axis, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, 
                                bool dirInvert, float stepsPerDeg, float maxSpeed, float accel, 
                                float minLim, float maxLim) {
    axis.stepPin = stepPin;
    axis.dirPin = dirPin;
    axis.enablePin = enablePin;
    axis.dirInvert = dirInvert;

    axis.currentStep = 0;
    axis.targetStep = 0;

    axis.currentSpeedDeg = 0.0f;
    axis.targetSpeedDeg = 0.0f;
    axis.maxSpeedDeg = maxSpeed;
    axis.accelDeg = accel;

    axis.stepsPerDeg = stepsPerDeg;
    axis.minLimitDeg = minLim;
    axis.maxLimitDeg = maxLim;

    axis.mode = MODE_IDLE;
    axis.direction = 0;

    axis.lastStepMicros = micros();
    axis.stepIntervalMicros = 1000000UL;
    axis.lastProfileUpdateMicros = micros();

    axis.stepPinHigh = false;
    axis.stepPulseStartMicros = 0;

    pinMode(axis.stepPin, OUTPUT);
    pinMode(axis.dirPin, OUTPUT);
    pinMode(axis.enablePin, OUTPUT);

    digitalWrite(axis.stepPin, LOW);
    digitalWrite(axis.dirPin, LOW);
}

void MotionController::setEnabled(bool enable) {
    drivesEnabled = enable;
    uint8_t state = enable ? (ENABLE_ACTIVE_LOW ? LOW : HIGH) : (ENABLE_ACTIVE_LOW ? HIGH : LOW);
    digitalWrite(panAxis.enablePin, state);
    digitalWrite(tiltAxis.enablePin, state);

    if (!enable) {
        panAxis.mode = MODE_IDLE;
        tiltAxis.mode = MODE_IDLE;
        panAxis.currentSpeedDeg = 0.0f;
        tiltAxis.currentSpeedDeg = 0.0f;
    }
}

float MotionController::clampPan(float deg) const {
    if (!softLimitsActive) return deg;
    if (deg < panAxis.minLimitDeg) return panAxis.minLimitDeg;
    if (deg > panAxis.maxLimitDeg) return panAxis.maxLimitDeg;
    return deg;
}

float MotionController::clampTilt(float deg) const {
    if (!softLimitsActive) return deg;
    if (deg < tiltAxis.minLimitDeg) return tiltAxis.minLimitDeg;
    if (deg > tiltAxis.maxLimitDeg) return tiltAxis.maxLimitDeg;
    return deg;
}

bool MotionController::moveTo(float panDeg, float tiltDeg) {
    if (!drivesEnabled) setEnabled(true);

    float clampedPan = clampPan(panDeg);
    float clampedTilt = clampTilt(tiltDeg);

    panAxis.targetStep = (long)roundf(clampedPan * panAxis.stepsPerDeg);
    tiltAxis.targetStep = (long)roundf(clampedTilt * tiltAxis.stepsPerDeg);

    panAxis.mode = MODE_POSITION;
    tiltAxis.mode = MODE_POSITION;

    return (clampedPan == panDeg && clampedTilt == tiltDeg);
}

bool MotionController::moveRel(float deltaPanDeg, float deltaTiltDeg) {
    float newPan = getPanDeg() + deltaPanDeg;
    float newTilt = getTiltDeg() + deltaTiltDeg;
    return moveTo(newPan, newTilt);
}

void MotionController::setJogVelocity(float panDegPerSec, float tiltDegPerSec) {
    if (!drivesEnabled) setEnabled(true);

    // Clamp to max allowable speeds
    if (panDegPerSec > panAxis.maxSpeedDeg) panDegPerSec = panAxis.maxSpeedDeg;
    if (panDegPerSec < -panAxis.maxSpeedDeg) panDegPerSec = -panAxis.maxSpeedDeg;

    if (tiltDegPerSec > tiltAxis.maxSpeedDeg) tiltDegPerSec = tiltAxis.maxSpeedDeg;
    if (tiltDegPerSec < -tiltAxis.maxSpeedDeg) tiltDegPerSec = -tiltAxis.maxSpeedDeg;

    panAxis.targetSpeedDeg = panDegPerSec;
    tiltAxis.targetSpeedDeg = tiltDegPerSec;

    panAxis.mode = (fabs(panDegPerSec) > 0.001f || fabs(panAxis.currentSpeedDeg) > 0.001f) ? MODE_JOG : MODE_IDLE;
    tiltAxis.mode = (fabs(tiltDegPerSec) > 0.001f || fabs(tiltAxis.currentSpeedDeg) > 0.001f) ? MODE_JOG : MODE_IDLE;
}

void MotionController::stop() {
    // Decelerate to stop smoothly
    panAxis.targetSpeedDeg = 0.0f;
    tiltAxis.targetSpeedDeg = 0.0f;
    panAxis.targetStep = panAxis.currentStep;
    tiltAxis.targetStep = tiltAxis.currentStep;
    panAxis.mode = MODE_JOG;
    tiltAxis.mode = MODE_JOG;
}

void MotionController::emergencyStop() {
    // Instant stop and disable
    panAxis.currentSpeedDeg = 0.0f;
    tiltAxis.currentSpeedDeg = 0.0f;
    panAxis.targetSpeedDeg = 0.0f;
    tiltAxis.targetSpeedDeg = 0.0f;
    panAxis.targetStep = panAxis.currentStep;
    tiltAxis.targetStep = tiltAxis.currentStep;
    panAxis.mode = MODE_IDLE;
    tiltAxis.mode = MODE_IDLE;

    digitalWrite(panAxis.stepPin, LOW);
    digitalWrite(tiltAxis.stepPin, LOW);
    setEnabled(false);
}

void MotionController::setZero() {
    panAxis.currentStep = 0;
    panAxis.targetStep = 0;
    tiltAxis.currentStep = 0;
    tiltAxis.targetStep = 0;
    panAxis.currentSpeedDeg = 0.0f;
    tiltAxis.currentSpeedDeg = 0.0f;
    panAxis.mode = MODE_IDLE;
    tiltAxis.mode = MODE_IDLE;
}

void MotionController::setPosition(float panDeg, float tiltDeg) {
    panAxis.currentStep = (long)roundf(panDeg * panAxis.stepsPerDeg);
    panAxis.targetStep = panAxis.currentStep;
    tiltAxis.currentStep = (long)roundf(tiltDeg * tiltAxis.stepsPerDeg);
    tiltAxis.targetStep = tiltAxis.currentStep;
    panAxis.currentSpeedDeg = 0.0f;
    tiltAxis.currentSpeedDeg = 0.0f;
    panAxis.mode = MODE_IDLE;
    tiltAxis.mode = MODE_IDLE;
}

void MotionController::setMaxSpeed(float panDegPerSec, float tiltDegPerSec) {
    if (panDegPerSec > 0.1f && panDegPerSec <= ABSOLUTE_MAX_SPEED_PAN) {
        panAxis.maxSpeedDeg = panDegPerSec;
    }
    if (tiltDegPerSec > 0.1f && tiltDegPerSec <= ABSOLUTE_MAX_SPEED_TILT) {
        tiltAxis.maxSpeedDeg = tiltDegPerSec;
    }
}

void MotionController::setAcceleration(float panDegPerSec2, float tiltDegPerSec2) {
    if (panDegPerSec2 > 1.0f) panAxis.accelDeg = panDegPerSec2;
    if (tiltDegPerSec2 > 1.0f) tiltAxis.accelDeg = tiltDegPerSec2;
}

void MotionController::setStepsPerDegree(float panStepsPerDeg, float tiltStepsPerDeg) {
    if (panStepsPerDeg > 0.01f) {
        float currentDeg = getPanDeg();
        panAxis.stepsPerDeg = panStepsPerDeg;
        panAxis.currentStep = (long)roundf(currentDeg * panStepsPerDeg);
        panAxis.targetStep = panAxis.currentStep;
    }
    if (tiltStepsPerDeg > 0.01f) {
        float currentDeg = getTiltDeg();
        tiltAxis.stepsPerDeg = tiltStepsPerDeg;
        tiltAxis.currentStep = (long)roundf(currentDeg * tiltStepsPerDeg);
        tiltAxis.targetStep = tiltAxis.currentStep;
    }
}

void MotionController::setDirectionInvert(bool invertPan, bool invertTilt) {
    panAxis.dirInvert = invertPan;
    tiltAxis.dirInvert = invertTilt;
}

void MotionController::setSoftLimitsEnabled(bool enabled) {
    softLimitsActive = enabled;
}

void MotionController::setSoftLimits(float panMin, float panMax, float tiltMin, float tiltMax) {
    if (panMin < panMax) {
        panAxis.minLimitDeg = panMin;
        panAxis.maxLimitDeg = panMax;
    }
    if (tiltMin < tiltMax) {
        tiltAxis.minLimitDeg = tiltMin;
        tiltAxis.maxLimitDeg = tiltMax;
    }
}

bool MotionController::isMoving() const {
    return (panAxis.mode != MODE_IDLE || tiltAxis.mode != MODE_IDLE ||
            fabs(panAxis.currentSpeedDeg) > 0.01f || fabs(tiltAxis.currentSpeedDeg) > 0.01f);
}

bool MotionController::isEnabled() const {
    return drivesEnabled;
}

float MotionController::getPanDeg() const {
    return (float)panAxis.currentStep / panAxis.stepsPerDeg;
}

float MotionController::getTiltDeg() const {
    return (float)tiltAxis.currentStep / tiltAxis.stepsPerDeg;
}

float MotionController::getTargetPanDeg() const {
    return (float)panAxis.targetStep / panAxis.stepsPerDeg;
}

float MotionController::getTargetTiltDeg() const {
    return (float)tiltAxis.targetStep / tiltAxis.stepsPerDeg;
}

float MotionController::getPanSpeedDeg() const {
    return panAxis.currentSpeedDeg;
}

float MotionController::getTiltSpeedDeg() const {
    return tiltAxis.currentSpeedDeg;
}

float MotionController::getPanMaxSpeed() const {
    return panAxis.maxSpeedDeg;
}

float MotionController::getTiltMaxSpeed() const {
    return tiltAxis.maxSpeedDeg;
}

float MotionController::getPanAccel() const {
    return panAxis.accelDeg;
}

float MotionController::getTiltAccel() const {
    return tiltAxis.accelDeg;
}

float MotionController::getPanStepsPerDeg() const {
    return panAxis.stepsPerDeg;
}

float MotionController::getTiltStepsPerDeg() const {
    return tiltAxis.stepsPerDeg;
}

bool MotionController::isPanDirInverted() const {
    return panAxis.dirInvert;
}

bool MotionController::isTiltDirInverted() const {
    return tiltAxis.dirInvert;
}

bool MotionController::areSoftLimitsEnabled() const {
    return softLimitsActive;
}

void MotionController::updateAxisMotionProfile(AxisState &axis, unsigned long nowMicros) {
    unsigned long dtMicros = nowMicros - axis.lastProfileUpdateMicros;
    if (dtMicros < 1000UL) return; // Update velocity profile every ~1ms for smooth ramping
    axis.lastProfileUpdateMicros = nowMicros;

    float dtSec = (float)dtMicros / 1000000.0f;
    float maxDeltaV = axis.accelDeg * dtSec;

    if (axis.mode == MODE_POSITION) {
        long stepsRemaining = axis.targetStep - axis.currentStep;

        if (stepsRemaining == 0 && fabs(axis.currentSpeedDeg) < 0.05f) {
            axis.currentSpeedDeg = 0.0f;
            axis.direction = 0;
            axis.mode = MODE_IDLE;
            return;
        }

        // Target movement direction
        int8_t desiredDir = (stepsRemaining > 0) ? 1 : -1;
        float degRemaining = fabs((float)stepsRemaining / axis.stepsPerDeg);

        // Calculate stopping distance for current speed: d = v^2 / (2 * a)
        float currentSpeedAbs = fabs(axis.currentSpeedDeg);
        float stoppingDistanceDeg = (currentSpeedAbs * currentSpeedAbs) / (2.0f * axis.accelDeg);

        float desiredSpeedDeg = axis.maxSpeedDeg;
        if (degRemaining <= stoppingDistanceDeg + 0.01f) {
            // Decelerating toward target
            desiredSpeedDeg = sqrtf(fmaxf(0.0f, 2.0f * axis.accelDeg * degRemaining));
        }

        // Ramp speed towards desiredSpeed in desired direction
        float signedDesiredSpeed = desiredDir * desiredSpeedDeg;
        if (axis.currentSpeedDeg < signedDesiredSpeed) {
            axis.currentSpeedDeg = fminf(axis.currentSpeedDeg + maxDeltaV, signedDesiredSpeed);
        } else if (axis.currentSpeedDeg > signedDesiredSpeed) {
            axis.currentSpeedDeg = fmaxf(axis.currentSpeedDeg - maxDeltaV, signedDesiredSpeed);
        }

        axis.direction = (axis.currentSpeedDeg > 0.001f) ? 1 : ((axis.currentSpeedDeg < -0.001f) ? -1 : 0);

    } else if (axis.mode == MODE_JOG) {
        // Soft limit checks during jog
        if (softLimitsActive) {
            float currentDeg = (float)axis.currentStep / axis.stepsPerDeg;
            if (currentDeg >= axis.maxLimitDeg && axis.targetSpeedDeg > 0.0f) {
                axis.targetSpeedDeg = 0.0f;
            }
            if (currentDeg <= axis.minLimitDeg && axis.targetSpeedDeg < 0.0f) {
                axis.targetSpeedDeg = 0.0f;
            }
        }

        if (axis.currentSpeedDeg < axis.targetSpeedDeg) {
            axis.currentSpeedDeg = fminf(axis.currentSpeedDeg + maxDeltaV, axis.targetSpeedDeg);
        } else if (axis.currentSpeedDeg > axis.targetSpeedDeg) {
            axis.currentSpeedDeg = fmaxf(axis.currentSpeedDeg - maxDeltaV, axis.targetSpeedDeg);
        }

        if (fabs(axis.currentSpeedDeg) < 0.001f && fabs(axis.targetSpeedDeg) < 0.001f) {
            axis.currentSpeedDeg = 0.0f;
            axis.direction = 0;
            axis.mode = MODE_IDLE;
        } else {
            axis.direction = (axis.currentSpeedDeg > 0.001f) ? 1 : ((axis.currentSpeedDeg < -0.001f) ? -1 : 0);
        }
    }

    // Compute step interval for step generator
    float absSpeed = fabs(axis.currentSpeedDeg);
    if (absSpeed > 0.05f) {
        float stepsPerSec = absSpeed * axis.stepsPerDeg;
        if (stepsPerSec > 0.1f) {
            unsigned long interval = (unsigned long)(1000000.0f / stepsPerSec);
            // Minimum interval cap (e.g. 50us = 20,000 steps/sec)
            axis.stepIntervalMicros = (interval < 50UL) ? 50UL : interval;
        }
    }
}

void MotionController::stepAxis(AxisState &axis, unsigned long nowMicros) {
    // Finish active pulse if elapsed
    if (axis.stepPinHigh) {
        if ((nowMicros - axis.stepPulseStartMicros) >= STEP_PULSE_WIDTH_US) {
            digitalWrite(axis.stepPin, LOW);
            axis.stepPinHigh = false;
        }
    }

    if (axis.direction == 0 || fabs(axis.currentSpeedDeg) < 0.05f) return;

    // Check if time for next step pulse
    if ((nowMicros - axis.lastStepMicros) >= axis.stepIntervalMicros) {
        axis.lastStepMicros = nowMicros;

        // Set direction pin
        bool dirPinLevel = (axis.direction > 0);
        if (axis.dirInvert) dirPinLevel = !dirPinLevel;
        digitalWrite(axis.dirPin, dirPinLevel ? HIGH : LOW);

        // Soft limit enforcement
        if (softLimitsActive) {
            float nextDeg = (float)(axis.currentStep + axis.direction) / axis.stepsPerDeg;
            if (axis.direction > 0 && nextDeg > axis.maxLimitDeg) {
                axis.currentSpeedDeg = 0.0f;
                axis.direction = 0;
                axis.mode = MODE_IDLE;
                return;
            }
            if (axis.direction < 0 && nextDeg < axis.minLimitDeg) {
                axis.currentSpeedDeg = 0.0f;
                axis.direction = 0;
                axis.mode = MODE_IDLE;
                return;
            }
        }

        // Trigger step pulse
        digitalWrite(axis.stepPin, HIGH);
        axis.stepPinHigh = true;
        axis.stepPulseStartMicros = nowMicros;

        // Update position counter
        axis.currentStep += axis.direction;
    }
}

void MotionController::update() {
    if (!drivesEnabled) return;

    unsigned long now = micros();

    // 1. Update acceleration/velocity profiles
    updateAxisMotionProfile(panAxis, now);
    updateAxisMotionProfile(tiltAxis, now);

    // 2. Execute step pulses
    stepAxis(panAxis, now);
    stepAxis(tiltAxis, now);
}

