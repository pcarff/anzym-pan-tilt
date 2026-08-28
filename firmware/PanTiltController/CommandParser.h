#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include "Config.h"

class MotionController;
class SensorManager;

class CommandParser {
public:
    CommandParser();

    void init();
    void processSerial(MotionController &motion, SensorManager &sensors);
    void sendTelemetry(const MotionController &motion, const SensorManager &sensors);
    
    uint16_t getTelemetryRateHz() const;
    void setTelemetryRateHz(uint16_t rateHz);

private:
    char rxBuffer[SERIAL_RX_BUFFER_SIZE];
    uint8_t rxIndex;
    uint16_t telemetryRateHz;
    unsigned long lastTelemetryMillis;

    void handleCommandLine(char *line, MotionController &motion, SensorManager &sensors);
    void printHelp();
    void printStatus(const MotionController &motion, const SensorManager &sensors);
};

#endif // COMMAND_PARSER_H

