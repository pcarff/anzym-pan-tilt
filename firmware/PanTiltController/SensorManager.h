#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "Config.h"

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

private:
    SensorReadings readings;
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

