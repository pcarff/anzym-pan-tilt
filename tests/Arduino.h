#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <vector>
#include <iomanip>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define A4 18
#define A5 19

#define F(x) x

class MockSerial {
public:
    std::string rxBuffer;
    std::stringstream txStream;

    void begin(long baud) { (void)baud; }
    int available() { return rxBuffer.length(); }
    int read() {
        if (rxBuffer.empty()) return -1;
        char c = rxBuffer[0];
        rxBuffer.erase(0, 1);
        return (unsigned char)c;
    }
    void write(const char* s) { txStream << s; }
    void print(const char* s) { txStream << s; }
    void print(float f, int prec = 2) { 
        txStream << std::fixed << std::setprecision(prec) << f; 
    }
    void print(long val) { txStream << val; }
    void print(int val) { txStream << val; }
    void println(const char* s = "") { txStream << s << "\n"; }
    void println(float f, int prec = 2) { 
        txStream << std::fixed << std::setprecision(prec) << f << "\n"; 
    }
    void println(long val) { txStream << val << "\n"; }
    void println(int val) { txStream << val << "\n"; }

    void injectCommand(const std::string& cmd) {
        rxBuffer += cmd + "\n";
    }

    std::string getOutput() {
        std::string s = txStream.str();
        txStream.str("");
        txStream.clear();
        return s;
    }
};

inline MockSerial Serial;

inline uint8_t mockPinModes[30] = {0};
inline uint8_t mockPinStates[30] = {0};
inline unsigned long mockCurrentMicros = 1000000;

inline void pinMode(uint8_t pin, uint8_t mode) {
    if (pin < 30) mockPinModes[pin] = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin < 30) mockPinStates[pin] = val;
}

inline int digitalRead(uint8_t pin) {
    if (pin < 30) return mockPinStates[pin];
    return HIGH;
}

inline unsigned long micros() {
    return mockCurrentMicros;
}

inline unsigned long millis() {
    return mockCurrentMicros / 1000;
}

inline void delay(unsigned long ms) {
    mockCurrentMicros += ms * 1000;
}

inline void advanceTimeMicros(unsigned long us) {
    mockCurrentMicros += us;
}

#endif // MOCK_ARDUINO_H
