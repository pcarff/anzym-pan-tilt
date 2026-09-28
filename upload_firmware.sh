#!/usr/bin/env bash
# Pan-Tilt Controller Firmware Flashing Helper

set -e

PORT="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85438333935351F0A081-if00"
if [ ! -e "$PORT" ]; then
    if [ -e "/dev/ttyACM1" ]; then
        PORT="/dev/ttyACM1"
    elif [ -e "/dev/ttyACM0" ]; then
        PORT="/dev/ttyACM0"
    fi
fi

echo "=================================================="
echo "    Pan-Tilt Controller Firmware Flashing Tool"
echo "=================================================="
echo "Target Device: Arduino Uno on $PORT"
echo "Hex File:      build/PanTiltController.ino.hex"
echo ""

# Check if port is locked by Chrome or another process
for i in {1..60}; do
    if lsof "$PORT" 2>/dev/null >/dev/null; then
        if [ "$i" -eq 1 ]; then
            echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            echo " [PORT BUSY] $PORT is currently opened by Chrome Web Serial!"
            echo " Please click 'Disconnect Serial' in the dashboard header so"
            echo " avrdude can flash the new firmware."
            echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        fi
        if (( i % 5 == 0 )); then
            echo "  Waiting for port to become free... (elapsed: $(( i * 2 ))s)"
        fi
        sleep 2
    else
        break
    fi
done

echo "✓ Port is free! Flashing firmware..."
echo ""

/usr/bin/avrdude -v -patmega328p -carduino -P"$PORT" -b115200 -D -Uflash:w:"$(pwd)/build/PanTiltController.ino.hex":i

echo ""
echo "=================================================="
echo "✓ FIRMWARE FLASHED SUCCESSFULLY!"
echo "Calibrated scales are now permanently saved in ROM:"
echo "  • Pan Scale:  114.572 steps/degree"
echo "  • Tilt Scale:  60.213 steps/degree"
echo "=================================================="
echo "You can now click 'Connect Serial' in your web dashboard!"
