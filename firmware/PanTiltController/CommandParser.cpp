#include "CommandParser.h"
#include "MotionController.h"
#include "SensorManager.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

CommandParser::CommandParser() :
    rxIndex(0),
    telemetryRateHz(DEFAULT_TELEMETRY_RATE_HZ),
    lastTelemetryMillis(0)
{
    rxBuffer[0] = '\0';
}

void CommandParser::init() {
    Serial.begin(SERIAL_BAUD_RATE);
    rxIndex = 0;
    lastTelemetryMillis = millis();
}

uint16_t CommandParser::getTelemetryRateHz() const {
    return telemetryRateHz;
}

void CommandParser::setTelemetryRateHz(uint16_t rateHz) {
    telemetryRateHz = (rateHz > 50) ? 50 : rateHz;
}

void CommandParser::processSerial(MotionController &motion, SensorManager &sensors) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r' || c == '\n') {
            if (rxIndex > 0) {
                rxBuffer[rxIndex] = '\0';
                handleCommandLine(rxBuffer, motion, sensors);
                rxIndex = 0;
            }
        } else if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
        }
    }

    // Handle periodic telemetry stream
    if (telemetryRateHz > 0) {
        unsigned long intervalMs = 1000UL / telemetryRateHz;
        if (millis() - lastTelemetryMillis >= intervalMs) {
            lastTelemetryMillis = millis();
            sendTelemetry(motion, sensors);
        }
    }
}

static char* skipWhitespace(char* str) {
    while (*str == ' ' || *str == '\t') str++;
    return str;
}

static void toUpperStr(char* str) {
    for (int i = 0; str[i]; i++) {
        str[i] = (char)toupper((unsigned char)str[i]);
    }
}

void CommandParser::handleCommandLine(char *line, MotionController &motion, SensorManager &sensors) {
    line = skipWhitespace(line);
    if (*line == '\0') return;

    // Extract first token (command verb)
    char verb[16] = {0};
    int i = 0;
    while (*line && *line != ' ' && *line != '\t' && i < 15) {
        verb[i++] = *line++;
    }
    verb[i] = '\0';
    toUpperStr(verb);

    char *args = skipWhitespace(line);

    if (strcmp(verb, "PING") == 0) {
        Serial.println(F("PONG"));
    }
    else if (strcmp(verb, "MOVE") == 0 || strcmp(verb, "GOTO") == 0) {
        float p = 0.0f, t = 0.0f;
        char *pArg = strstr(args, "P=");
        char *tArg = strstr(args, "T=");

        if (pArg || tArg) {
            p = pArg ? atof(pArg + 2) : motion.getTargetPanDeg();
            t = tArg ? atof(tArg + 2) : motion.getTargetTiltDeg();
        } else {
            p = atof(args);
            char *next = skipWhitespace(args);
            while (*next && *next != ' ') next++;
            next = skipWhitespace(next);
            t = (*next) ? atof(next) : motion.getTargetTiltDeg();
        }

        bool ok = motion.moveTo(p, t);
        if (ok) {
            Serial.println(F("OK"));
        } else {
            Serial.println(F("OK CLAMPED"));
        }
    }
    else if (strcmp(verb, "MOVEREL") == 0 || strcmp(verb, "REL") == 0) {
        float dp = atof(args);
        char *next = skipWhitespace(args);
        while (*next && *next != ' ') next++;
        next = skipWhitespace(next);
        float dt = (*next) ? atof(next) : 0.0f;

        motion.moveRel(dp, dt);
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "JOG") == 0) {
        float pSpeed = atof(args);
        char *next = skipWhitespace(args);
        while (*next && *next != ' ') next++;
        next = skipWhitespace(next);
        float tSpeed = (*next) ? atof(next) : 0.0f;

        motion.setJogVelocity(pSpeed, tSpeed);
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "STOP") == 0) {
        motion.stop();
        sensors.cancelHoming(motion);
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "ESTOP") == 0) {
        motion.emergencyStop();
        sensors.cancelHoming(motion);
        Serial.println(F("OK ESTOP_ACTIVE"));
    }
    else if (strcmp(verb, "ENABLE") == 0) {
        motion.setEnabled(true);
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "DISABLE") == 0) {
        motion.setEnabled(false);
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "ZERO") == 0) {
        motion.setZero();
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "SETPOS") == 0) {
        float p = atof(args);
        char *next = skipWhitespace(args);
        while (*next && *next != ' ') next++;
        next = skipWhitespace(next);
        float t = (*next) ? atof(next) : 0.0f;
        motion.setPosition(p, t);
        Serial.println(F("OK"));
    }
    else if (strcmp(verb, "SYNC") == 0) {
        toUpperStr(args);
        if (strstr(args, "IMU")) {
            const SensorReadings &readings = sensors.getReadings();
            if (readings.imuAvailable) {
                motion.setPosition(readings.imuYaw, readings.imuPitch);
                Serial.print(F("OK SYNCED_IMU P="));
                Serial.print(motion.getPanDeg(), 2);
                Serial.print(F(" T="));
                Serial.println(motion.getTiltDeg(), 2);
            } else {
                Serial.println(F("ERR IMU_NOT_DETECTED"));
            }
        } else {
            Serial.println(F("ERR UNKNOWN_SYNC_TARGET"));
        }
    }
    else if (strcmp(verb, "HOME") == 0) {
        toUpperStr(args);
        bool homePan = true;
        bool homeTilt = true;
        if (strstr(args, "PAN") && !strstr(args, "ALL")) homeTilt = false;
        if (strstr(args, "TILT") && !strstr(args, "ALL")) homePan = false;

        sensors.startHoming(motion, homePan, homeTilt);
        Serial.println(F("OK HOMING_STARTED"));
    }
    else if (strcmp(verb, "SET") == 0) {
        char sub[16] = {0};
        int s = 0;
        while (*args && *args != ' ' && *args != '\t' && s < 15) {
            sub[s++] = *args++;
        }
        sub[s] = '\0';
        toUpperStr(sub);
        char *subArgs = skipWhitespace(args);

        if (strcmp(sub, "SPEED") == 0 || strcmp(sub, "MAXSPD") == 0) {
            float pSpd = atof(subArgs);
            char *next = skipWhitespace(subArgs);
            while (*next && *next != ' ') next++;
            next = skipWhitespace(next);
            float tSpd = (*next) ? atof(next) : pSpd;
            motion.setMaxSpeed(pSpd, tSpd);
            Serial.println(F("OK"));
        }
        else if (strcmp(sub, "ACCEL") == 0) {
            float pAcc = atof(subArgs);
            char *next = skipWhitespace(subArgs);
            while (*next && *next != ' ') next++;
            next = skipWhitespace(next);
            float tAcc = (*next) ? atof(next) : pAcc;
            motion.setAcceleration(pAcc, tAcc);
            Serial.println(F("OK"));
        }
        else if (strcmp(sub, "LIMITS") == 0) {
            toUpperStr(subArgs);
            if (strstr(subArgs, "ON") || strstr(subArgs, "1") || strstr(subArgs, "TRUE")) {
                motion.setSoftLimitsEnabled(true);
                Serial.println(F("OK LIMITS ON"));
            } else if (strstr(subArgs, "OFF") || strstr(subArgs, "0") || strstr(subArgs, "FALSE")) {
                motion.setSoftLimitsEnabled(false);
                Serial.println(F("OK LIMITS OFF"));
            } else {
                float pMin = atof(subArgs);
                char *next = skipWhitespace(subArgs); while (*next && *next != ' ') next++; next = skipWhitespace(next);
                float pMax = atof(next);
                while (*next && *next != ' ') next++; next = skipWhitespace(next);
                float tMin = atof(next);
                while (*next && *next != ' ') next++; next = skipWhitespace(next);
                float tMax = atof(next);

                motion.setSoftLimits(pMin, pMax, tMin, tMax);
                Serial.println(F("OK"));
            }
        }
        else if (strcmp(sub, "SCALE") == 0) {
            float pScale = atof(subArgs);
            char *next = skipWhitespace(subArgs); while (*next && *next != ' ') next++; next = skipWhitespace(next);
            float tScale = (*next) ? atof(next) : pScale;
            motion.setStepsPerDegree(pScale, tScale);
            Serial.print(F("OK SCALE P="));
            Serial.print(motion.getPanStepsPerDeg(), 3);
            Serial.print(F(" T="));
            Serial.println(motion.getTiltStepsPerDeg(), 3);
        }
        else if (strcmp(sub, "INVERT") == 0) {
            int pInv = atoi(subArgs);
            char *next = skipWhitespace(subArgs); while (*next && *next != ' ') next++; next = skipWhitespace(next);
            int tInv = (*next) ? atoi(next) : pInv;
            motion.setDirectionInvert(pInv == 1, tInv == 1);
            Serial.print(F("OK INVERT P="));
            Serial.print(motion.isPanDirInverted() ? 1 : 0);
            Serial.print(F(" T="));
            Serial.println(motion.isTiltDirInverted() ? 1 : 0);
        }
        else if (strcmp(sub, "TELEMETRY") == 0) {
            int rate = atoi(subArgs);
            setTelemetryRateHz(rate);
            Serial.print(F("OK TELEMETRY_HZ="));
            Serial.println(telemetryRateHz);
        }
        else {
            Serial.println(F("ERR UNKNOWN_SET_PARAMETER"));
        }
    }
    else if (strcmp(verb, "GET") == 0) {
        char sub[16] = {0};
        int s = 0;
        while (*args && *args != ' ' && *args != '\t' && s < 15) {
            sub[s++] = *args++;
        }
        sub[s] = '\0';
        toUpperStr(sub);

        if (strcmp(sub, "POS") == 0) {
            Serial.print(F("POS P="));
            Serial.print(motion.getPanDeg(), 2);
            Serial.print(F(" T="));
            Serial.println(motion.getTiltDeg(), 2);
        }
        else if (strcmp(sub, "SCALE") == 0) {
            Serial.print(F("SCALE P="));
            Serial.print(motion.getPanStepsPerDeg(), 3);
            Serial.print(F(" T="));
            Serial.print(motion.getTiltStepsPerDeg(), 3);
            Serial.print(F(" INVP="));
            Serial.print(motion.isPanDirInverted() ? 1 : 0);
            Serial.print(F(" INVT="));
            Serial.println(motion.isTiltDirInverted() ? 1 : 0);
        }
        else if (strcmp(sub, "STATUS") == 0) {
            printStatus(motion, sensors);
        }
        else {
            Serial.println(F("ERR UNKNOWN_GET_PARAMETER"));
        }
    }
    else if (strcmp(verb, "HELP") == 0 || strcmp(verb, "?") == 0) {
        printHelp();
    }
    else {
        Serial.print(F("ERR UNKNOWN_CMD: "));
        Serial.println(verb);
    }
}

void CommandParser::sendTelemetry(const MotionController &motion, const SensorManager &sensors) {
    const SensorReadings &readings = sensors.getReadings();
    Serial.print(F("STATUS P="));
    Serial.print(motion.getPanDeg(), 2);
    Serial.print(F(" T="));
    Serial.print(motion.getTiltDeg(), 2);
    Serial.print(F(" TP="));
    Serial.print(motion.getTargetPanDeg(), 2);
    Serial.print(F(" TT="));
    Serial.print(motion.getTargetTiltDeg(), 2);
    Serial.print(F(" SP="));
    Serial.print(motion.getPanSpeedDeg(), 1);
    Serial.print(F(" ST="));
    Serial.print(motion.getTiltSpeedDeg(), 1);
    Serial.print(F(" MV="));
    Serial.print(motion.isMoving() ? 1 : 0);
    Serial.print(F(" EN="));
    Serial.print(motion.isEnabled() ? 1 : 0);
    Serial.print(F(" LP="));
    Serial.print(readings.panLimitPressed ? 1 : 0);
    Serial.print(F(" LT="));
    Serial.print(readings.tiltLimitPressed ? 1 : 0);
    if (readings.imuAvailable) {
        Serial.print(F(" IP="));
        Serial.print(readings.imuPitch, 2);
        Serial.print(F(" IR="));
        Serial.print(readings.imuRoll, 2);
        Serial.print(F(" IY="));
        Serial.print(readings.imuYaw, 2);
        Serial.print(F(" IC="));
        Serial.print(readings.calSys);
        Serial.print(',');
        Serial.print(readings.calGyro);
        Serial.print(',');
        Serial.print(readings.calAccel);
        Serial.print(',');
        Serial.print(readings.calMag);
    }
    Serial.println();
}

void CommandParser::printStatus(const MotionController &motion, const SensorManager &sensors) {
    sendTelemetry(motion, sensors);
}

void CommandParser::printHelp() {
    Serial.println(F("=== Pan-Tilt Controller Commands ==="));
    Serial.println(F("MOVE <pan> <tilt>        - Absolute move in degrees"));
    Serial.println(F("MOVEREL <d_pan> <d_tilt> - Relative move in degrees"));
    Serial.println(F("JOG <p_spd> <t_spd>      - Continuous velocity jog in deg/s"));
    Serial.println(F("STOP                     - Decelerate and stop"));
    Serial.println(F("ESTOP                    - Instant stop and disable drives"));
    Serial.println(F("ENABLE / DISABLE         - Enable / Disable stepper drives"));
    Serial.println(F("ZERO                     - Set current position as (0,0)"));
    Serial.println(F("SETPOS <pan> <tilt>      - Set current coordinate offset"));
    Serial.println(F("HOME [ALL|PAN|TILT]      - Run homing routine"));
    Serial.println(F("SET SPEED <pan> <tilt>   - Set max speed (deg/s)"));
    Serial.println(F("SET ACCEL <pan> <tilt>   - Set acceleration (deg/s^2)"));
    Serial.println(F("SET LIMITS <ON|OFF>      - Enable/Disable soft limits"));
    Serial.println(F("SET TELEMETRY <hz>       - Set auto status rate (0=off)"));
    Serial.println(F("GET POS / GET STATUS     - Query position or telemetry"));
    Serial.println(F("PING / HELP              - Connectivity test and info"));
}

