#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"

enum MotionMode {
    MODE_IDLE,
    MODE_POSITION,
    MODE_JOG
};

struct AxisState {
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    bool dirInvert;

    long currentStep;
    long targetStep;

    float currentSpeedDeg;  // Current velocity in deg/sec
    float targetSpeedDeg;   // Target velocity in deg/sec
    float maxSpeedDeg;      // Max velocity limit in deg/sec
    float accelDeg;         // Acceleration in deg/sec^2

    float stepsPerDeg;
    float minLimitDeg;
    float maxLimitDeg;

    MotionMode mode;
    int8_t direction;       // 1 = positive, -1 = negative, 0 = stopped

    unsigned long lastStepMicros;
    unsigned long stepIntervalMicros;
    unsigned long lastProfileUpdateMicros;

    bool stepPinHigh;
    unsigned long stepPulseStartMicros;
};

class MotionController {
public:
    MotionController();

    void init();
    void update(); // Must be called frequently in loop()

    // Motion commands (in degrees and deg/sec)
    bool moveTo(float panDeg, float tiltDeg);
    bool moveRel(float deltaPanDeg, float deltaTiltDeg);
    void setJogVelocity(float panDegPerSec, float tiltDegPerSec);
    void stop();
    void emergencyStop();

    // Configuration & limits
    void setZero();
    void setPosition(float panDeg, float tiltDeg);
    void setMaxSpeed(float panDegPerSec, float tiltDegPerSec);
    void setAcceleration(float panDegPerSec2, float tiltDegPerSec2);
    void setStepsPerDegree(float panStepsPerDeg, float tiltStepsPerDeg);
    void setDirectionInvert(bool invertPan, bool invertTilt);
    void setSoftLimitsEnabled(bool enabled);
    void setSoftLimits(float panMin, float panMax, float tiltMin, float tiltMax);
    void setEnabled(bool enable);

    // Status queries
    bool isMoving() const;
    bool isEnabled() const;
    float getPanDeg() const;
    float getTiltDeg() const;
    float getTargetPanDeg() const;
    float getTargetTiltDeg() const;
    float getPanSpeedDeg() const;
    float getTiltSpeedDeg() const;
    float getPanMaxSpeed() const;
    float getTiltMaxSpeed() const;
    float getPanAccel() const;
    float getTiltAccel() const;
    float getPanStepsPerDeg() const;
    float getTiltStepsPerDeg() const;
    bool isPanDirInverted() const;
    bool isTiltDirInverted() const;
    bool areSoftLimitsEnabled() const;

private:
    AxisState panAxis;
    AxisState tiltAxis;
    bool drivesEnabled;
    bool softLimitsActive;

    void initAxis(AxisState &axis, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, 
                  bool dirInvert, float stepsPerDeg, float maxSpeed, float accel, 
                  float minLim, float maxLim);
    
    void updateAxisMotionProfile(AxisState &axis, unsigned long nowMicros);
    void stepAxis(AxisState &axis, unsigned long nowMicros);
    float clampPan(float deg) const;
    float clampTilt(float deg) const;
};

#endif // MOTION_CONTROLLER_H

