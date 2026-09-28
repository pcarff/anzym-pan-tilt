#!/usr/bin/env python3
"""
Dual-Axis Pan-Tilt (Alt-Azimuth) Stepper Controller
Automated Pan Axis Calibration via BNO055 IMU Ground Truth

Executes discrete rotations (+90°, -90°, +180°, -180°), measures true physical
rotation with the BNO055 IMU, and calculates the exact steps/degree and steps/rev.
"""

import sys
import os
import time
import math
import serial
import serial.tools.list_ports

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


def find_port():
    if os.path.exists(SERIAL_PORT):
        return SERIAL_PORT
    for p in serial.tools.list_ports.comports():
        if "Arduino" in p.description or "ACM" in p.device or "USB" in p.device:
            return p.device
    return SERIAL_PORT


def signed_angle_diff(angle_end, angle_start):
    """Computes signed difference (end - start) wrapped to [-180, +180]."""
    diff = (angle_end - angle_start) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff


class PanCalibrator:
    def __init__(self, port):
        self.port = port
        self.ser = None

    def connect(self):
        print(f"Connecting to controller on {self.port} at {BAUD_RATE} baud...")
        # Check if port is locked / busy (e.g. Chrome Web Serial)
        for attempt in range(60):
            try:
                self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.5)
                break
            except serial.SerialException as e:
                if "Device or resource busy" in str(e) or "Errno 16" in str(e):
                    if attempt == 0:
                        print("\n" + "!" * 70)
                        print(" [PORT BUSY] /dev/ttyACM0 is currently held open by Chrome!")
                        print(" Please click 'Disconnect Serial' in the dashboard header so this")
                        print(" script can take control of the serial bus.")
                        print("!" * 70 + "\n")
                    if (attempt + 1) % 5 == 0:
                        print(f"  Waiting for port to become free... (elapsed: {(attempt+1)*1.5:.0f}s)")
                    time.sleep(1.5)
                else:
                    raise e
        else:
            raise RuntimeError(f"Could not open {self.port}. Please ensure no other process (like Chrome) is connected.")

        # Allow Arduino bootloader to initialize
        time.sleep(1.8)
        self.ser.reset_input_buffer()
        print("✓ Connected successfully!")

    def send_cmd(self, cmd: str) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd.strip()}\n".encode("utf-8"))
        time.sleep(0.05)
        response = self.ser.readline().decode("utf-8", errors="ignore").strip()
        return response

    def query_imu(self) -> dict:
        """Queries GET IMU and returns dictionary with P, R, Y and CAL."""
        self.ser.reset_input_buffer()
        self.ser.write(b"GET IMU\n")
        time.sleep(0.06)
        
        # Read lines looking for IMU response
        timeout = time.time() + 1.0
        while time.time() < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if "IMU" in line and "Y=" in line:
                    data = {}
                    tokens = line.split()
                    for t in tokens:
                        if "=" in t:
                            k, v = t.split("=", 1)
                            try:
                                if "," in v:
                                    data[k] = [int(x) for x in v.split(",")]
                                elif "." in v:
                                    data[k] = float(v)
                                else:
                                    data[k] = int(v)
                            except ValueError:
                                data[k] = v
                    return data
            time.sleep(0.02)
        return {}

    def get_averaged_imu(self, samples=5, delay=0.08) -> dict:
        """Samples the IMU multiple times to reject sensor noise."""
        yaws, pitches, rolls = [], [], []
        for _ in range(samples):
            data = self.query_imu()
            if data and data.get("AVAIL", 0) == 1:
                yaws.append(data.get("Y", 0.0))
                pitches.append(data.get("P", 0.0))
                rolls.append(data.get("R", 0.0))
            time.sleep(delay)

        if not yaws:
            return {}

        # Circular mean for yaw to handle 0/360 wrap
        rads = [math.radians(y) for y in yaws]
        avg_sin = sum(math.sin(r) for r in rads) / len(rads)
        avg_cos = sum(math.cos(r) for r in rads) / len(rads)
        mean_yaw = (math.degrees(math.atan2(avg_sin, avg_cos))) % 360.0

        return {
            "Y": mean_yaw,
            "P": sum(pitches) / len(pitches),
            "R": sum(rolls) / len(rolls),
            "samples": len(yaws)
        }

    def wait_for_motion_complete(self, timeout=20.0):
        start = time.time()
        time.sleep(0.2)
        while time.time() - start < timeout:
            self.ser.reset_input_buffer()
            self.ser.write(b"GET STATUS\n")
            time.sleep(0.05)
            while self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("STATUS") and "MV=" in line:
                    tokens = line.split()
                    for t in tokens:
                        if t.startswith("MV="):
                            is_moving = t.split("=")[1]
                            if is_moving == "0":
                                return True
            time.sleep(0.1)
        return False

    def run_calibration(self):
        self.connect()

        # Check scale
        self.send_cmd("SET LIMITS OFF")  # Avoid clamping during test
        scale_res = self.send_cmd("GET SCALE")
        print(f"Current Controller Scale: {scale_res}")

        # Parse current pan steps/deg
        current_pan_scale = 5.556
        tokens = scale_res.split()
        for t in tokens:
            if t.startswith("P="):
                try:
                    current_pan_scale = float(t.split("=")[1])
                except ValueError:
                    pass

        # Check IMU
        print("\nVerifying BNO055 IMU status...")
        imu_init = self.query_imu()
        if not imu_init or imu_init.get("AVAIL", 0) != 1:
            print("❌ BNO055 IMU is not responding! (AVAIL=0)")
            print("Please verify SDA (A4) / SCL (A5) connections and 5V/GND.")
            return

        print(f"✓ BNO055 Online: Chip ID = {imu_init.get('CHIP', '0x28')}")
        print(f"  Current IMU Orientation: Yaw={imu_init.get('Y')}°, Pitch={imu_init.get('P')}°, Roll={imu_init.get('R')}°")
        print(f"  Calibration (Sys,Gyr,Acc,Mag): {imu_init.get('CAL', 'N/A')}")

        # Zero axes at current position
        print("\nZeroing current axis position as starting origin (0.0°)...")
        self.send_cmd("ZERO")
        time.sleep(0.5)

        # Tests to run: 90° and 180° in both directions
        test_angles = [90.0, -90.0, 180.0, -180.0]
        results = []

        print("\n" + "=" * 75)
        print("          STARTING AUTOMATED PAN ROTATION TESTS")
        print("=" * 75)

        for target in test_angles:
            direction_name = "CW (+)" if target > 0 else "CCW (-)"
            angle_mag = abs(target)
            print(f"\n--- Test: Rotate {direction_name} {angle_mag:.0f}° ---")

            # Record baseline IMU orientation
            time.sleep(0.5)
            base_imu = self.get_averaged_imu()
            print(f"  Baseline IMU Heading: Yaw={base_imu['Y']:.2f}°, Pitch={base_imu['P']:.2f}°, Roll={base_imu['R']:.2f}°")

            # Send MOVE command
            print(f"  Commanding move to {target:+.1f}° (Generating ~{angle_mag * current_pan_scale:.0f} steps)...")
            self.send_cmd(f"MOVE {target:.1f} 0.0")

            # Wait for physical motion to complete
            self.wait_for_motion_complete()

            # Wait 1.5 seconds for mechanical vibrations and IMU filter to settle
            print("  Motion complete. Settling IMU reading (1.5s)...")
            time.sleep(1.5)

            # Read final IMU orientation
            final_imu = self.get_averaged_imu()
            print(f"  Settled IMU Heading:  Yaw={final_imu['Y']:.2f}°, Pitch={final_imu['P']:.2f}°, Roll={final_imu['R']:.2f}°")

            # Calculate measured delta
            delta_yaw = signed_angle_diff(final_imu['Y'], base_imu['Y'])
            delta_pitch = final_imu['P'] - base_imu['P']
            delta_roll = final_imu['R'] - base_imu['R']

            # Identify active axis (usually Yaw, or Roll if mounted on side)
            max_delta = delta_yaw
            active_axis = "Yaw"
            if abs(delta_roll) > abs(max_delta):
                max_delta = delta_roll
                active_axis = "Roll"
            if abs(delta_pitch) > abs(max_delta):
                max_delta = delta_pitch
                active_axis = "Pitch"

            measured_angle = abs(max_delta)
            commanded_steps = angle_mag * current_pan_scale
            measured_steps_per_deg = commanded_steps / measured_angle if measured_angle > 0.5 else 0

            print(f"  Measured IMU Delta ({active_axis}): {max_delta:+.2f}° (Expected: {target:+.1f}°)")
            print(f"  Calculated Sample Ratio:    {measured_steps_per_deg:.3f} steps/degree")

            results.append({
                "target": target,
                "commanded_steps": commanded_steps,
                "measured_delta": max_delta,
                "measured_angle": measured_angle,
                "steps_per_deg": measured_steps_per_deg,
                "axis": active_axis
            })

            # Return to origin (0.0°) before next test
            print("  Returning to 0.0° origin...")
            self.send_cmd("MOVE 0.0 0.0")
            self.wait_for_motion_complete()
            time.sleep(1.0)

        # Re-enable soft limits
        self.send_cmd("SET LIMITS ON")

        # Summary calculations
        valid_ratios = [r["steps_per_deg"] for r in results if r["steps_per_deg"] > 0]
        if not valid_ratios:
            print("\n❌ Could not calculate valid step ratios. Check IMU response.")
            return

        avg_steps_per_deg = sum(valid_ratios) / len(valid_ratios)
        steps_per_rev = avg_steps_per_deg * 360.0

        print("\n" + "=" * 75)
        print("                   CALIBRATION RESULTS & ANALYSIS")
        print("=" * 75)
        print(f"{'Target Move':<16} | {'Steps Issued':<14} | {'IMU Measured':<14} | {'Measured Scale':<16}")
        print("-" * 75)
        for r in results:
            print(f"{r['target']:+6.1f}° ({r['axis']})   | {r['commanded_steps']:>10.0f}     | {r['measured_delta']:>+10.2f}°   | {r['steps_per_deg']:>10.3f} steps/°")
        print("-" * 75)
        print(f"Current Firmware Scale:      {current_pan_scale:.3f} steps/degree ({current_pan_scale * 360.0:.1f} steps/rev)")
        print(f"Calibrated True Scale:       {avg_steps_per_deg:.3f} steps/degree")
        print(f"Calibrated Steps/Revolution: {steps_per_rev:.1f} steps/rev")

        # Estimate hardware configuration
        print("\nHardware Configuration Match:")
        for candidate_spr in [2000, 3200, 4000, 5000, 6400, 8000, 10000, 12800, 20000, 25000, 50000]:
            diff_pct = abs(steps_per_rev - candidate_spr) / candidate_spr * 100.0
            if diff_pct < 8.0:
                print(f"  ⭐ High probability match: {candidate_spr} steps/rev (Error: {diff_pct:.2f}%)")

        print("\n" + "=" * 75)
        print("                      RECOMMENDED ACTIONS")
        print("=" * 75)
        print(f"1. To update the running Arduino immediately, send via Terminal or Python:")
        print(f"   SET SCALE {avg_steps_per_deg:.3f} 5.556")
        print(f"\n2. To make this permanent in firmware, update firmware/PanTiltController/Config.h:")
        print(f"   // Set PAN_STEPS_PER_DEG or adjust PAN_MICROSTEPS / PAN_GEAR_RATIO:")
        print(f"   #define PAN_STEPS_PER_DEG {avg_steps_per_deg:.3f}f")
        print("=" * 75)

        # Apply to live controller
        self.send_cmd(f"SET SCALE {avg_steps_per_deg:.3f} 5.556")
        print("\n✓ Live controller updated with calibrated scale!")
        print("Disconnecting so you can reconnect the Web Dashboard.\n")
        self.ser.close()


if __name__ == "__main__":
    port = find_port()
    calibrator = PanCalibrator(port)
    try:
        calibrator.run_calibration()
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user.")
        if calibrator.ser and calibrator.ser.is_open:
            calibrator.ser.close()
    except Exception as e:
        print(f"\nError during calibration: {e}")
        if calibrator.ser and calibrator.ser.is_open:
            calibrator.ser.close()
