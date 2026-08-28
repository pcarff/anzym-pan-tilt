#include "BNO055_Driver.h"

BNO055_Driver::BNO055_Driver(uint8_t i2cAddr) 
    : addr(i2cAddr), connected(false), currentMode(BNO055_OPR_CONFIG),
      consecutiveReadFails(0) {
}

bool BNO055_Driver::begin(uint8_t mode) {
    // NOTE: Wire.begin() must only be called ONCE in the application.
    // Calling it repeatedly resets the ATmega328P TWI hardware state machine
    // and can corrupt in-flight transactions.
    // The caller (SensorManager::init) should call Wire.begin() before this.

    delay(50); // Allow BNO055 power-on boot (~400ms from cold, but usually already up)

    // Check Chip ID at primary address (0x28)
    addr = BNO055_DEFAULT_ADDR;
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
    consecutiveReadFails = 0;

    // Switch to CONFIG mode first
    setMode(BNO055_OPR_CONFIG);
    delay(30);

    // Set page ID to 0
    writeRegister(BNO055_REG_PAGE_ID, 0x00);
    delay(10);

    // Set normal power mode (0x00)
    writeRegister(BNO055_REG_PWR_MODE, 0x00);
    delay(10);

    // Use internal oscillator
    writeRegister(BNO055_REG_SYS_TRIGGER, 0x00);
    delay(10);

    // Set units: Celsius, Degrees, deg/s, m/s^2
    writeRegister(BNO055_REG_UNIT_SEL, 0x00);
    delay(10);

    // Switch to fusion mode
    setMode(mode);
    delay(50);

    return true;
}

bool BNO055_Driver::isConnected() const {
    return connected;
}

void BNO055_Driver::setMode(uint8_t mode) {
    writeRegister(BNO055_REG_OPR_MODE, mode);
    currentMode = mode;
    delay(30);
}

void BNO055_Driver::reset() {
    writeRegister(BNO055_REG_SYS_TRIGGER, 0x20);
    delay(650);
    begin(currentMode);
}

uint8_t BNO055_Driver::getChipId() {
    return readRegister(BNO055_REG_CHIP_ID);
}

uint8_t BNO055_Driver::getOprMode() {
    return readRegister(BNO055_REG_OPR_MODE);
}

uint8_t BNO055_Driver::getSysStatus() {
    return readRegister(BNO055_REG_SYS_STATUS);
}

uint8_t BNO055_Driver::getSysError() {
    return readRegister(BNO055_REG_SYS_ERR);
}

void BNO055_Driver::setAxisRemap(uint8_t config, uint8_t sign) {
    uint8_t prevMode = currentMode;
    setMode(BNO055_OPR_CONFIG);
    delay(25);
    writeRegister(BNO055_REG_AXIS_MAP_CONFIG, config);
    writeRegister(BNO055_REG_AXIS_MAP_SIGN, sign);
    setMode(prevMode);
    delay(25);
}

bool BNO055_Driver::readEuler(BNO055_Orientation &outOri) {
    if (!connected) {
        outOri.valid = false;
        return false;
    }

    uint8_t buffer[6] = {0};
    if (!readRegisters(BNO055_REG_EULER_H_LSB, buffer, 6)) {
        consecutiveReadFails++;
        // After 10 consecutive failures, mark sensor as disconnected
        // so SensorManager's retry loop can reinitialize it
        if (consecutiveReadFails > 10) {
            connected = false;
        }
        outOri.valid = false;
        return false;
    }

    consecutiveReadFails = 0; // Reset on success

    int16_t hRaw = (int16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8));
    int16_t rRaw = (int16_t)((uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8));
    int16_t pRaw = (int16_t)((uint16_t)buffer[4] | ((uint16_t)buffer[5] << 8));

    outOri.heading = (float)hRaw / 16.0f;
    outOri.roll    = (float)rRaw / 16.0f;
    outOri.pitch   = (float)pRaw / 16.0f;
    outOri.valid   = true;

    return true;
}

bool BNO055_Driver::readCalibration(BNO055_Calibration &outCal) {
    if (!connected) return false;

    uint8_t stat = readRegister(BNO055_REG_CALIB_STAT);
    if (stat == 0xFF) return false;

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
    // Use STOP condition (true) instead of repeated-start (false).
    // The ATmega328P TWI peripheral is known to hang with repeated-start
    // when the BNO055 clock-stretches during fusion calculations.
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return 0xFF;

    if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) == 1) {
        return Wire.read();
    }
    return 0xFF;
}

bool BNO055_Driver::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    // Use STOP condition — same rationale as readRegister().
    // The BNO055 auto-increments its register pointer even after a STOP,
    // so this works correctly for burst reads.
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    uint8_t readCount = Wire.requestFrom((uint8_t)addr, (uint8_t)len);
    if (readCount != len) {
        // Drain any partial data from Wire buffer
        while (Wire.available()) Wire.read();
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

