#include <cassert>
#include <iostream>
#include <cmath>
#include "Arduino.h"
#include "../firmware/PanTiltController/Config.h"
#include "../firmware/PanTiltController/MotionController.h"
#include "../firmware/PanTiltController/SensorManager.h"
#include "../firmware/PanTiltController/CommandParser.h"

void test_motion_controller() {
    std::cout << "[TEST] Running MotionController tests..." << std::endl;
    MotionController motion;
    motion.init();

    // Check initial zero position
    assert(fabs(motion.getPanDeg()) < 0.001f);
    assert(fabs(motion.getTiltDeg()) < 0.001f);
    assert(!motion.isMoving());

    // Test moveTo within limits
    bool moved = motion.moveTo(45.0f, 30.0f);
    assert(moved);
    assert(fabs(motion.getTargetPanDeg() - 45.0f) < 0.3f);
    assert(fabs(motion.getTargetTiltDeg() - 30.0f) < 0.3f);
    assert(motion.isMoving());

    // Simulate time advancing to reach target
    for (int step = 0; step < 5000; step++) {
        advanceTimeMicros(1000); // 1 ms per loop
        motion.update();
        if (!motion.isMoving()) break;
    }

    assert(fabs(motion.getPanDeg() - 45.0f) < 0.3f);
    assert(fabs(motion.getTiltDeg() - 30.0f) < 0.3f);
    std::cout << "  ✓ Position move reached: Pan=" << motion.getPanDeg() << "°, Tilt=" << motion.getTiltDeg() << "°" << std::endl;

    // Test soft limit clamping
    // Max tilt is 90 deg, attempt 120 deg
    moved = motion.moveTo(0.0f, 120.0f);
    assert(!moved); // Was clamped
    assert(fabs(motion.getTargetTiltDeg() - 90.0f) < 0.3f);
    std::cout << "  ✓ Soft limits successfully clamped tilt to " << motion.getTargetTiltDeg() << "°" << std::endl;

    // Test Jog mode
    motion.setJogVelocity(20.0f, -10.0f);
    assert(motion.isMoving());
    advanceTimeMicros(50000); // 50 ms
    motion.update();
    assert(motion.getPanSpeedDeg() > 0.0f);
    
    // Stop
    motion.stop();
    for (int step = 0; step < 2000; step++) {
        advanceTimeMicros(1000);
        motion.update();
        if (!motion.isMoving()) break;
    }
    assert(fabs(motion.getPanSpeedDeg()) < 0.01f);
    std::cout << "  ✓ Jog and smooth deceleration verified" << std::endl;

    // Emergency stop
    motion.moveTo(90.0f, 45.0f);
    assert(motion.isMoving());
    motion.emergencyStop();
    assert(!motion.isMoving());
    assert(!motion.isEnabled());
    std::cout << "  ✓ Emergency stop verified" << std::endl;
}

void test_command_parser() {
    std::cout << "[TEST] Running CommandParser tests..." << std::endl;
    MotionController motion;
    SensorManager sensors;
    CommandParser parser;

    motion.init();
    sensors.init();
    parser.init();

    // 1. PING test
    Serial.injectCommand("PING");
    parser.processSerial(motion, sensors);
    std::string out = Serial.getOutput();
    assert(out.find("PONG") != std::string::npos);
    std::cout << "  ✓ PING -> PONG acknowledged" << std::endl;

    // 2. MOVE command test
    Serial.injectCommand("MOVE 15.5 -10.0");
    parser.processSerial(motion, sensors);
    out = Serial.getOutput();
    assert(out.find("OK") != std::string::npos);
    assert(fabs(motion.getTargetPanDeg() - 15.5f) < 0.3f);
    assert(fabs(motion.getTargetTiltDeg() - -10.0f) < 0.3f);
    std::cout << "  ✓ MOVE 15.5 -10.0 parsed and executed" << std::endl;

    // 3. JOG command test
    Serial.injectCommand("JOG 25.0 -12.5");
    parser.processSerial(motion, sensors);
    out = Serial.getOutput();
    assert(out.find("OK") != std::string::npos);
    std::cout << "  ✓ JOG command executed" << std::endl;

    // 4. STOP command test
    Serial.injectCommand("STOP");
    parser.processSerial(motion, sensors);
    out = Serial.getOutput();
    assert(out.find("OK") != std::string::npos);
    std::cout << "  ✓ STOP command executed" << std::endl;

    // 5. GET STATUS test
    Serial.injectCommand("GET STATUS");
    parser.processSerial(motion, sensors);
    out = Serial.getOutput();
    assert(out.find("STATUS P=") != std::string::npos);
    assert(out.find("TP=") != std::string::npos);
    assert(out.find("MV=") != std::string::npos);
    std::cout << "  ✓ GET STATUS returned valid telemetry: " << out;

    // 6. ZERO test
    Serial.injectCommand("ZERO");
    parser.processSerial(motion, sensors);
    assert(fabs(motion.getPanDeg()) < 0.001f);
    assert(fabs(motion.getTiltDeg()) < 0.001f);
    std::cout << "  ✓ ZERO command reset coordinates" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Starting Pan-Tilt Firmware Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_motion_controller();
    test_command_parser();

    std::cout << "========================================" << std::endl;
    std::cout << "ALL FIRMWARE TESTS PASSED SUCCESSFULLY! ✓" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
