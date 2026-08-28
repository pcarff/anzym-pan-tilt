#ifndef WIRE_H
#define WIRE_H

#include <cstdint>
#include <vector>
#include <queue>

class TwoWire {
public:
    uint8_t targetAddr = 0;
    uint8_t lastReg = 0;
    std::queue<uint8_t> readQueue;

    void begin() {}
    void beginTransmission(uint8_t addr) { targetAddr = addr; }
    void write(uint8_t val) { lastReg = val; }
    uint8_t endTransmission() { return 0; }

    uint8_t requestFrom(int addr, int len) {
        while (readQueue.size() < (size_t)len) {
            if (lastReg == 0x00) readQueue.push(0xA0); // CHIP_ID
            else if (lastReg == 0x35) readQueue.push(0xFF); // CALIB_STAT (3,3,3,3)
            else readQueue.push(0x00);
        }
        return len;
    }

    int available() { return (int)readQueue.size(); }
    uint8_t read() {
        if (readQueue.empty()) return 0;
        uint8_t b = readQueue.front();
        readQueue.pop();
        return b;
    }
};

inline TwoWire Wire;

#endif // WIRE_H

