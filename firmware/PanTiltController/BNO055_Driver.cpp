#include "BNO055_Driver.h"

BNO055_Driver::BNO055_Driver(uint8_t i2cAddr) 
    : addr(i2cAddr), connected(false), currentMode(BNO055_OPR_CONFIG) {
}

bool BNO055_Driver::begin(uint8_t mode) {
    Wire.begin();

    // Check Chip ID at primary address (0x28)
    uint8_t id = readRegister(BNO055_REG_CHIP_ID);
    if (id != BNO055_CHIP_ID_VAL) {
        // Try alternate address (0x29)
        addr = BNO055_ALT_ADDR;
        id = readRegister(BNO055_REG_CHIP_ID);
        if (id != BNO055_CHIP_ID_VAL) {
            connected = false;
            return false;
        }
    }

    connected = true;

    // Switch to CONFIG mode first to set parameters
    setMode(BNO055_OPR_CONFIG);
    delay(30);

    // Reset page ID to 0
    writeRegister(BNO055_REG_PAGE_ID, 0);

    // Set normal power mode
    writeRegister(BNO055_REG_PWR_MODE, 0x00);
    delay(10);

    // Set units: Celsius, Degrees, deg/s, m/s^2 (all bits 0)
    writeRegister(BNO055_REG_UNIT_SEL, 0x00);
    delay(10);

    // Switch to requested operational mode (default NDOF 9-DOF fusion)
    setMode(mode);
    delay(30);

    return true;
}

bool BNO055_Driver::isConnected() const {
    return connected;
}

void BNO055_Driver::setMode(uint8_t mode) {
    writeRegister(BNO055_REG_OPR_MODE, mode);
    currentMode = mode;
    delay(20);
}

void BNO055_Driver::reset() {
    writeRegister(BNO055_REG_SYS_TRIGGER, 0x20); // Trigger system reset
    delay(650); // BNO055 takes ~650ms to reboot
    begin(currentMode);
}

void BNO055_Driver::setAxisRemap(uint8_t config, uint8_t sign) {
    uint8_t prevMode = currentMode;
    setMode(BNO055_OPR_CONFIG);
    delay(20);
    writeRegister(BNO055_REG_AXIS_MAP_CONFIG, config);
    writeRegister(BNO055_REG_AXIS_MAP_SIGN, sign);
    setMode(prevMode);
    delay(20);
}

bool BNO055_Driver::readEuler(BNO055_Orientation &outOri) {
    if (!connected) {
        outOri.valid = false;
        return false;
    }

    uint8_t buffer[6];
    if (!readRegisters(BNO055_REG_EULER_H_LSB, buffer, 6)) {
        outOri.valid = false;
        return false;
    }

    int16_t hRaw = (int16_t)(buffer[0] | ((int16_t)buffer[1] << 8));
    int16_t rRaw = (int16_t)(buffer[2] | ((int16_t)buffer[3] << 8));
    int16_t pRaw = (int16_t)(buffer[4] | ((int16_t)buffer[5] << 8));

    // 1 LSB = 1/16 degree = 0.0625 degrees
    outOri.heading = (float)hRaw / 16.0f;
    outOri.roll    = (float)rRaw / 16.0f;
    outOri.pitch   = (float)pRaw / 16.0f;
    outOri.valid   = true;

    return true;
}

bool BNO055_Driver::readCalibration(BNO055_Calibration &outCal) {
    if (!connected) return false;

    uint8_t stat = readRegister(BNO055_REG_CALIB_STAT);
    outCal.sys   = (stat >> 6) & 0x03;
    outCal.gyro  = (stat >> 4) & 0x03;
    outCal.accel = (stat >> 2) & 0x03;
    outCal.mag   = stat & 0x03;

    return true;
}

bool BNO055_Driver::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

uint8_t BNO055_Driver::readRegister(uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return 0xFF;

    Wire.requestFrom((int)addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}

bool BNO055_Driver::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;

    uint8_t readCount = Wire.requestFrom((int)addr, (int)len);
    if (readCount != len) return false;

    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

