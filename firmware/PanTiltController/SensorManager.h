#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "Config.h"
#include "BNO055_Driver.h"

enum HomingState {
    HOMING_IDLE,
    HOMING_PAN_SEARCH,
    HOMING_PAN_BACKOFF,
    HOMING_TILT_SEARCH,
    HOMING_TILT_BACKOFF,
    HOMING_COMPLETE,
    HOMING_FAILED
};

struct SensorReadings {
    bool panLimitPressed;
    bool tiltLimitPressed;
    bool imuAvailable;
    float imuPitch;
    float imuRoll;
    float imuYaw;
    uint8_t calSys;
    uint8_t calGyro;
    uint8_t calAccel;
    uint8_t calMag;
    float temperature;
};

class MotionController; // Forward declaration

class SensorManager {
public:
    SensorManager();

    void init();
    void update(MotionController &motion);

    bool startHoming(MotionController &motion, bool homePan = true, bool homeTilt = true);
    void cancelHoming(MotionController &motion);
    bool isHomingActive() const;
    HomingState getHomingState() const;

    bool isPanLimitPressed() const;
    bool isTiltLimitPressed() const;

    const SensorReadings& getReadings() const;
    BNO055_Driver& getIMU();

private:
    SensorReadings readings;
    BNO055_Driver bno;
    unsigned long lastImuReadTime;

    HomingState homingState;
    unsigned long homingStartTime;
    bool homingPanRequested;
    bool homingTiltRequested;

    // Debounce counters
    uint8_t panDebounceCounter;
    uint8_t tiltDebounceCounter;

    void updateLimitSwitches();
    void updateAuxSensors();
    void processHomingStateMachine(MotionController &motion);
};

#endif // SENSOR_MANAGER_H

