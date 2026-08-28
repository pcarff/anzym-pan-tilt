#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

// BNO055 I2C Registers
#define BNO055_DEFAULT_ADDR        0x28
#define BNO055_ALT_ADDR            0x29
#define BNO055_CHIP_ID_VAL         0xA0

#define BNO055_REG_CHIP_ID         0x00
#define BNO055_REG_PAGE_ID         0x07
#define BNO055_REG_EULER_H_LSB     0x1A
#define BNO055_REG_CALIB_STAT      0x35
#define BNO055_REG_SYS_STATUS      0x39
#define BNO055_REG_SYS_ERR         0x3A
#define BNO055_REG_UNIT_SEL        0x3B
#define BNO055_REG_OPR_MODE        0x3D
#define BNO055_REG_PWR_MODE        0x3E
#define BNO055_REG_SYS_TRIGGER     0x3F
#define BNO055_REG_AXIS_MAP_CONFIG 0x41
#define BNO055_REG_AXIS_MAP_SIGN   0x42

// Operation Modes
#define BNO055_OPR_CONFIG          0x00
#define BNO055_OPR_ACCONLY         0x01
#define BNO055_OPR_MAGONLY         0x02
#define BNO055_OPR_GYRONLY         0x03
#define BNO055_OPR_ACCMAG          0x04
#define BNO055_OPR_ACCGYRO         0x05
#define BNO055_OPR_MAGGYRO         0x06
#define BNO055_OPR_AMG             0x07
#define BNO055_OPR_IMUPLUS         0x08  // Relative fusion (Accel + Gyro)
#define BNO055_OPR_COMPASS         0x09
#define BNO055_OPR_M4G             0x0A
#define BNO055_OPR_NDOF_FMC_OFF    0x0B
#define BNO055_OPR_NDOF            0x0C  // 9-DOF absolute orientation fusion

struct BNO055_Orientation {
    float heading;  // 0.0 to 360.0 degrees (Azimuth/Pan)
    float pitch;    // -180.0 to +180.0 degrees (Elevation/Tilt)
    float roll;     // -90.0 to +90.0 degrees
    bool valid;
};

struct BNO055_Calibration {
    uint8_t sys;    // 0 (uncalibrated) to 3 (fully calibrated)
    uint8_t gyro;   // 0 to 3
    uint8_t accel;  // 0 to 3
    uint8_t mag;    // 0 to 3
};

class BNO055_Driver {
public:
    BNO055_Driver(uint8_t i2cAddr = BNO055_DEFAULT_ADDR);

    bool begin(uint8_t mode = BNO055_OPR_NDOF);
    bool isConnected() const;

    bool readEuler(BNO055_Orientation &outOri);
    bool readCalibration(BNO055_Calibration &outCal);

    uint8_t getChipId();
    uint8_t getOprMode();
    uint8_t getSysStatus();
    uint8_t getSysError();

    void setMode(uint8_t mode);
    void reset();

    // Axis remapping for different mounting configurations
    void setAxisRemap(uint8_t config, uint8_t sign);

private:
    uint8_t addr;
    bool connected;
    uint8_t currentMode;
    uint8_t consecutiveReadFails;

    bool writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
};

#endif // BNO055_DRIVER_H

