#!/usr/bin/env python3
"""
Dual-Axis Pan-Tilt 10x Automated Calibration & Ground Data Analysis Tool
Iterates 10 full calibration cycles, computes high-precision averaged scales,
updates Config.h, recompiles, flashes the Arduino, and sets Pan/Tilt to horizontal and 000.
"""

import sys
import os
import time
import math
import subprocess
import serial
import serial.tools.list_ports

BAUD_RATE = 115200


def find_port():
    by_id_dir = "/dev/serial/by-id"
    if os.path.exists(by_id_dir):
        for f in os.listdir(by_id_dir):
            if "Arduino" in f:
                return os.path.join(by_id_dir, f)
    for p in serial.tools.list_ports.comports():
        if "Arduino" in p.description or "ACM" in p.device or "USB" in p.device:
            return p.device
    if os.path.exists("/dev/ttyACM1"):
        return "/dev/ttyACM1"
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"
    return "/dev/ttyACM1"


def signed_angle_diff(angle_end, angle_start):
    diff = (angle_end - angle_start) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff


class CalibrationManager10x:
    def __init__(self, port):
        self.port = port
        self.ser = None

    def connect(self):
        print(f"Connecting to controller on {self.port} at {BAUD_RATE} baud...")
        for attempt in range(60):
            try:
                self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.5)
                break
            except serial.SerialException as e:
                if "Device or resource busy" in str(e) or "Errno 16" in str(e):
                    if attempt == 0:
                        print("\n" + "!" * 70)
                        print(" [PORT BUSY] Serial port is currently held open by Chrome!")
                        print(" Please click 'Disconnect Serial' in the dashboard header.")
                        print("!" * 70 + "\n")
                    if (attempt + 1) % 5 == 0:
                        print(f"  Waiting for port to become free... (elapsed: {(attempt+1)*1.5:.0f}s)")
                    time.sleep(1.5)
                else:
                    raise e
        else:
            raise RuntimeError(f"Could not open {self.port}. Ensure no other process holds the port.")

        time.sleep(1.8)
        self.ser.reset_input_buffer()
        print("✓ Connected successfully!")

        # Turn OFF periodic telemetry during calibration commands
        self.ser.write(b"SET TELEMETRY 0\n")
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def send_cmd(self, cmd: str) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd.strip()}\n".encode("utf-8"))
        time.sleep(0.05)
        for _ in range(15):
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line and not line.startswith("STATUS"):
                    return line
            time.sleep(0.02)
        return ""

    def get_scale(self) -> tuple:
        self.ser.reset_input_buffer()
        self.ser.write(b"GET SCALE\n")
        for _ in range(25):
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("SCALE"):
                    p_scale, t_scale = 114.572, 60.213
                    for t in line.split():
                        if t.startswith("P="):
                            try: p_scale = float(t.split("=")[1])
                            except ValueError: pass
                        elif t.startswith("T="):
                            try: t_scale = float(t.split("=")[1])
                            except ValueError: pass
                    return p_scale, t_scale
            time.sleep(0.03)
        return 114.572, 60.213

    def query_imu(self) -> dict:
        self.ser.reset_input_buffer()
        self.ser.write(b"GET IMU\n")
        time.sleep(0.06)
        timeout = time.time() + 1.2
        while time.time() < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("IMU") and "Y=" in line:
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

    def get_averaged_imu(self, samples=6, delay=0.08) -> dict:
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

    def wait_for_motion_complete(self, timeout=25.0):
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
                            if t.split("=")[1] == "0":
                                return True
            time.sleep(0.08)
        return False

    def run_10x_calibration(self):
        self.connect()

        self.send_cmd("SET LIMITS OFF")
        cur_p_scale, cur_t_scale = self.get_scale()
        print(f"Starting Base Scales: Pan = {cur_p_scale:.3f} steps/°, Tilt = {cur_t_scale:.3f} steps/°")

        print("Checking BNO055 IMU status...")
        imu_init = self.query_imu()
        if not imu_init or imu_init.get("AVAIL", 0) != 1:
            print("❌ BNO055 IMU is not responding!")
            return None, None

        print(f"✓ BNO055 Online (Chip {imu_init.get('CHIP', '0xA0')}): Heading={imu_init.get('Y')}°, Pitch={imu_init.get('P')}°, Roll={imu_init.get('R')}°")
        print("Zeroing starting origin (0.0°, 0.0°)...")
        self.send_cmd("ZERO")
        time.sleep(0.5)

        all_pan_samples = []
        all_tilt_samples = []
        active_tilt_axis = "Roll"

        TOTAL_RUNS = 10
        print("\n" + "=" * 75)
        print(f"       STARTING {TOTAL_RUNS}-RUN HIGH-PRECISION CALIBRATION SUITE")
        print("=" * 75)

        for run in range(1, TOTAL_RUNS + 1):
            print(f"\n>>> ITERATION {run}/{TOTAL_RUNS} <<<")

            # --- Pan Moves ---
            # Standard runs: +90° and -90°. Runs 5 & 10 also include ±180° sweeps
            pan_test_angles = [90.0, -90.0]
            if run in [5, 10]:
                pan_test_angles = [90.0, -90.0, 180.0, -180.0]

            for target in pan_test_angles:
                time.sleep(0.3)
                base_imu = self.get_averaged_imu()
                cmd_steps = abs(target) * cur_p_scale
                self.send_cmd(f"MOVE {target:.1f} 0.0")
                self.wait_for_motion_complete()
                time.sleep(1.2)  # Settle IMU filter
                final_imu = self.get_averaged_imu()

                d_yaw = signed_angle_diff(final_imu['Y'], base_imu['Y'])
                meas_angle = abs(d_yaw)
                if meas_angle > 0.5:
                    sample_scale = cmd_steps / meas_angle
                    all_pan_samples.append(sample_scale)
                    print(f"  [Run {run:2d}] Pan {target:+6.1f}° -> IMU {d_yaw:+6.2f}° | Sample Scale: {sample_scale:7.3f} steps/°")

                # Return to center
                self.send_cmd("MOVE 0.0 0.0")
                self.wait_for_motion_complete()
                time.sleep(0.5)

            # --- Tilt Moves ---
            tilt_test_angles = [30.0, -30.0]
            if run in [5, 10]:
                tilt_test_angles = [30.0, -30.0, 45.0, -45.0]

            for target in tilt_test_angles:
                time.sleep(0.3)
                base_imu = self.get_averaged_imu()
                cmd_steps = abs(target) * cur_t_scale
                self.send_cmd(f"MOVE 0.0 {target:.1f}")
                self.wait_for_motion_complete()
                time.sleep(1.2)
                final_imu = self.get_averaged_imu()

                d_pitch = final_imu['P'] - base_imu['P']
                d_roll = final_imu['R'] - base_imu['R']
                meas_delta = d_roll if abs(d_roll) > abs(d_pitch) else d_pitch
                meas_angle = abs(meas_delta)

                if meas_angle > 0.3:
                    sample_scale = cmd_steps / meas_angle
                    all_tilt_samples.append(sample_scale)
                    print(f"  [Run {run:2d}] Tilt {target:+6.1f}° -> IMU {meas_delta:+6.2f}° | Sample Scale: {sample_scale:7.3f} steps/°")

                # Return to center
                self.send_cmd("MOVE 0.0 0.0")
                self.wait_for_motion_complete()
                time.sleep(0.5)

        # Statistical analysis
        if not all_pan_samples or not all_tilt_samples:
            print("❌ Calibration failed to collect sufficient valid samples.")
            return None, None

        avg_pan_scale = sum(all_pan_samples) / len(all_pan_samples)
        std_pan = math.sqrt(sum((x - avg_pan_scale) ** 2 for x in all_pan_samples) / len(all_pan_samples))

        avg_tilt_scale = sum(all_tilt_samples) / len(all_tilt_samples)
        std_tilt = math.sqrt(sum((x - avg_tilt_scale) ** 2 for x in all_tilt_samples) / len(all_tilt_samples))

        print("\n" + "=" * 75)
        print("          10-RUN STATISTICAL ANALYSIS & GROUND TRUTH RESULTS")
        print("=" * 75)
        print(f"PAN AXIS (Azimuth):")
        print(f"  • Total Samples:           {len(all_pan_samples)}")
        print(f"  • Mean Steps/Degree:       {avg_pan_scale:.3f} steps/° (StdDev: ±{std_pan:.3f})")
        print(f"  • Min / Max Sample:        {min(all_pan_samples):.3f} / {max(all_pan_samples):.3f}")
        print(f"  • Full Revolution:         {avg_pan_scale * 360.0:.1f} steps/rev")
        print(f"  • Nominal Ratio (2000 spr): {avg_pan_scale * 360.0 / 2000.0:.3f}:1")
        print("-" * 75)
        print(f"TILT AXIS (Altitude):")
        print(f"  • Total Samples:           {len(all_tilt_samples)}")
        print(f"  • Mean Steps/Degree:       {avg_tilt_scale:.3f} steps/° (StdDev: ±{std_tilt:.3f})")
        print(f"  • Min / Max Sample:        {min(all_tilt_samples):.3f} / {max(all_tilt_samples):.3f}")
        print(f"  • Full Revolution:         {avg_tilt_scale * 360.0:.1f} steps/rev")
        print(f"  • Nominal Ratio (2000 spr): {avg_tilt_scale * 360.0 / 2000.0:.3f}:1")
        print("=" * 75)

        # Update live controller with averaged scales
        self.send_cmd(f"SET SCALE {avg_pan_scale:.3f} {avg_tilt_scale:.3f}")

        # Set to horizontal and 000
        print("\nAligning Pan/Tilt to Horizontal (0.0°) and 000°...")
        self.send_cmd("MOVE 0.0 0.0")
        self.wait_for_motion_complete()
        time.sleep(1.0)
        self.send_cmd("ZERO")
        time.sleep(0.5)

        final_check = self.get_averaged_imu()
        print(f"✓ Position Set to Origin (0.0°, 0.0°). Final IMU Readout:")
        print(f"  Heading (Pan): {final_check.get('Y', 0.0):.2f}°")
        print(f"  Pitch:         {final_check.get('P', 0.0):.2f}°")
        print(f"  Roll (Tilt):   {final_check.get('R', 0.0):.2f}°")

        self.send_cmd("SET LIMITS ON")
        self.send_cmd("SET TELEMETRY 10")
        self.ser.close()

        return avg_pan_scale, avg_tilt_scale


def update_and_flash_firmware(pan_scale, tilt_scale):
    config_file = "/workspaces/anzym_altaz_ws/firmware/PanTiltController/Config.h"
    print(f"\nUpdating {config_file} with calibrated values...")

    pan_ratio = (pan_scale * 360.0) / 2000.0
    tilt_ratio = (tilt_scale * 360.0) / 2000.0

    with open(config_file, "r") as f:
        content = f.read()

    # Replace gear ratios
    lines = content.splitlines()
    new_lines = []
    for line in lines:
        if "#define PAN_GEAR_RATIO" in line:
            new_lines.append(f"#define PAN_GEAR_RATIO            {pan_ratio:.3f}f // Calibrated 10x ({pan_scale:.3f} steps/deg, {pan_scale*360:.1f} spr)")
        elif "#define TILT_GEAR_RATIO" in line:
            new_lines.append(f"#define TILT_GEAR_RATIO           {tilt_ratio:.3f}f // Calibrated 10x ({tilt_scale:.3f} steps/deg, {tilt_scale*360:.1f} spr)")
        else:
            new_lines.append(line)

    with open(config_file, "w") as f:
        f.write("\n".join(new_lines) + "\n")
    print("✓ Config.h updated successfully!")

    # Run unit tests
    print("\nRunning unit tests...")
    test_res = subprocess.run(["npm", "run", "test:firmware"], cwd="/workspaces/anzym_altaz_ws", capture_output=True, text=True)
    if test_res.returncode != 0:
        print(f"❌ Unit test warning/failure:\n{test_res.stdout}\n{test_res.stderr}")
    else:
        print("✓ Unit tests passed successfully!")

    # Compile with arduino-cli
    print("\nCompiling firmware with arduino-cli...")
    comp_cmd = ["arduino-cli", "compile", "--fqbn", "arduino:avr:uno", "--output-dir", "/workspaces/anzym_altaz_ws/build", "firmware/PanTiltController"]
    comp_res = subprocess.run(comp_cmd, cwd="/workspaces/anzym_altaz_ws", capture_output=True, text=True)
    if comp_res.returncode != 0:
        print(f"❌ Compilation error:\n{comp_res.stderr}")
        return False
    print("✓ Firmware compiled cleanly!")

    # Flash with avrdude
    port = find_port()
    print(f"\nFlashing hex to Arduino Uno on {port}...")
    hex_path = "/workspaces/anzym_altaz_ws/build/PanTiltController.ino.hex"
    flash_cmd = ["/usr/bin/avrdude", "-v", "-patmega328p", "-carduino", f"-P{port}", "-b115200", "-D", f"-Uflash:w:{hex_path}:i"]
    flash_res = subprocess.run(flash_cmd, capture_output=True, text=True)
    if flash_res.returncode != 0:
        print(f"❌ Flashing error:\n{flash_res.stderr}")
        return False

    print("✓ Firmware successfully flashed into Arduino Uno flash ROM!")
    return True


if __name__ == "__main__":
    port = find_port()
    mgr = CalibrationManager10x(port)
    pan_scale, tilt_scale = mgr.run_10x_calibration()

    if pan_scale and tilt_scale:
        ok = update_and_flash_firmware(pan_scale, tilt_scale)
        if ok:
            print("\n" + "=" * 75)
            print("🎉 10X CALIBRATION & FIRMWARE FLASH COMPLETED!")
            print(f"Final Pan Scale:  {pan_scale:.3f} steps/degree ({pan_scale*360:.1f} steps/rev)")
            print(f"Final Tilt Scale: {tilt_scale:.3f} steps/degree ({tilt_scale*360:.1f} steps/rev)")
            print("Pan/Tilt set to Horizontal (0.0°) and 000°.")
            print("=" * 75)
            print("You can now click 'Connect Serial' in the Web Dashboard.\n")
