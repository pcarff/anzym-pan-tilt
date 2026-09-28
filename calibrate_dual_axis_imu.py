#!/usr/bin/env python3
"""
Dual-Axis Pan-Tilt (Alt-Azimuth) Stepper Controller
Automated Dual-Axis Calibration via BNO055 IMU Ground Truth

Features:
1. Auto-detects Arduino port (/dev/serial/by-id or /dev/ttyACM*)
2. Disables periodic telemetry to ensure clean serial communication
3. Calibrates Pan axis (Yaw) across discrete moves
4. Calibrates Tilt axis (detects Pitch vs Roll on IMU)
5. Applies calibrated scale via SET SCALE
6. Executes full verification test moves (+45° Pan, +30° Tilt)
"""

import sys
import os
import time
import math
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
    """Computes signed difference (end - start) wrapped to [-180, +180]."""
    diff = (angle_end - angle_start) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff


class DualAxisCalibrator:
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

        # Turn OFF periodic telemetry so replies are synchronous
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
                    p_scale = 5.556
                    t_scale = 5.556
                    for t in line.split():
                        if t.startswith("P="):
                            try: p_scale = float(t.split("=")[1])
                            except ValueError: pass
                        elif t.startswith("T="):
                            try: t_scale = float(t.split("=")[1])
                            except ValueError: pass
                    return p_scale, t_scale
            time.sleep(0.03)
        return 5.556, 5.556

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
            time.sleep(0.1)
        return False

    def run_calibration(self):
        self.connect()

        self.send_cmd("SET LIMITS OFF")
        cur_p_scale, cur_t_scale = self.get_scale()
        print(f"Current Controller Scale: Pan = {cur_p_scale:.3f} steps/°, Tilt = {cur_t_scale:.3f} steps/°")

        print("\nVerifying BNO055 IMU status...")
        imu_init = self.query_imu()
        if not imu_init or imu_init.get("AVAIL", 0) != 1:
            print("❌ BNO055 IMU is not responding! (AVAIL=0)")
            return

        print(f"✓ BNO055 Online: Chip ID = {imu_init.get('CHIP', '0x28')}")
        print(f"  Current IMU Orientation: Yaw={imu_init.get('Y')}°, Pitch={imu_init.get('P')}°, Roll={imu_init.get('R')}°")
        print(f"  Calibration (Sys,Gyr,Acc,Mag): {imu_init.get('CAL', 'N/A')}")

        # Zero origin
        print("\nZeroing current axis position as starting origin (0.0°, 0.0°)...")
        self.send_cmd("ZERO")
        time.sleep(0.5)

        # =========================================================================
        # 1. PAN AXIS CALIBRATION
        # =========================================================================
        print("\n" + "=" * 75)
        print("          PHASE 1: PAN (AZIMUTH) AXIS CALIBRATION")
        print("=" * 75)

        # Pre-seed pan scale with ~112 steps/deg if it reset to 5.556
        test_pan_scale = cur_p_scale if cur_p_scale > 20.0 else 112.352
        self.send_cmd(f"SET SCALE {test_pan_scale:.3f} {cur_t_scale:.3f}")
        print(f"Using Pan base scale: {test_pan_scale:.3f} steps/°")

        pan_targets = [90.0, -90.0, 180.0, -180.0]
        pan_results = []

        for target in pan_targets:
            dir_str = "CW (+)" if target > 0 else "CCW (-)"
            mag = abs(target)
            print(f"\n--- Pan Test: {dir_str} {mag:.0f}° ---")

            time.sleep(0.4)
            base_imu = self.get_averaged_imu()

            cmd_steps = mag * test_pan_scale
            print(f"  Commanding MOVE {target:+.1f} 0.0 (Generating {cmd_steps:.0f} steps)...")
            self.send_cmd(f"MOVE {target:.1f} 0.0")
            self.wait_for_motion_complete()

            time.sleep(1.5)
            final_imu = self.get_averaged_imu()

            d_yaw = signed_angle_diff(final_imu['Y'], base_imu['Y'])
            meas_angle = abs(d_yaw)
            scale_sample = cmd_steps / meas_angle if meas_angle > 0.5 else 0.0

            print(f"  IMU Heading Delta: {d_yaw:+.2f}° (Target: {target:+.1f}°)")
            print(f"  Calculated Scale:  {scale_sample:.3f} steps/degree")

            pan_results.append({
                "target": target,
                "cmd_steps": cmd_steps,
                "d_imu": d_yaw,
                "scale": scale_sample
            })

            # Return to center
            self.send_cmd("MOVE 0.0 0.0")
            self.wait_for_motion_complete()
            time.sleep(0.8)

        pan_scales = [r["scale"] for r in pan_results if r["scale"] > 0]
        cal_pan_scale = sum(pan_scales) / len(pan_scales) if pan_scales else test_pan_scale
        print(f"\n>> Calibrated Pan Scale: {cal_pan_scale:.3f} steps/degree ({cal_pan_scale * 360:.1f} steps/rev)")

        # =========================================================================
        # 2. TILT AXIS CALIBRATION
        # =========================================================================
        print("\n" + "=" * 75)
        print("          PHASE 2: TILT (ALTITUDE) AXIS CALIBRATION")
        print("=" * 75)

        # Exploratory step check for Tilt:
        # Start with small commanded moves to measure the response safely
        tilt_targets = [30.0, -30.0, 45.0, -45.0]
        tilt_results = []
        active_tilt_axis = "Pitch"

        print(f"Initial Tilt scale in controller: {cur_t_scale:.3f} steps/°")

        for target in tilt_targets:
            dir_str = "UP (+)" if target > 0 else "DOWN (-)"
            mag = abs(target)
            print(f"\n--- Tilt Test: {dir_str} {mag:.0f}° ---")

            time.sleep(0.4)
            base_imu = self.get_averaged_imu()

            cmd_steps = mag * cur_t_scale
            print(f"  Commanding MOVE 0.0 {target:+.1f} (Generating {cmd_steps:.0f} steps)...")
            self.send_cmd(f"MOVE 0.0 {target:.1f}")
            self.wait_for_motion_complete()

            time.sleep(1.5)
            final_imu = self.get_averaged_imu()

            d_pitch = final_imu['P'] - base_imu['P']
            d_roll = final_imu['R'] - base_imu['R']

            # Choose axis with larger displacement
            if abs(d_roll) > abs(d_pitch):
                meas_delta = d_roll
                active_tilt_axis = "Roll"
            else:
                meas_delta = d_pitch
                active_tilt_axis = "Pitch"

            meas_angle = abs(meas_delta)
            scale_sample = cmd_steps / meas_angle if meas_angle > 0.1 else 0.0

            print(f"  IMU Delta ({active_tilt_axis}): {meas_delta:+.2f}° (Pitch={d_pitch:+.2f}°, Roll={d_roll:+.2f}°)")
            print(f"  Calculated Scale:         {scale_sample:.3f} steps/degree")

            tilt_results.append({
                "target": target,
                "cmd_steps": cmd_steps,
                "d_imu": meas_delta,
                "axis": active_tilt_axis,
                "scale": scale_sample
            })

            # Return to center
            self.send_cmd("MOVE 0.0 0.0")
            self.wait_for_motion_complete()
            time.sleep(0.8)

        tilt_scales = [r["scale"] for r in tilt_results if r["scale"] > 0]
        cal_tilt_scale = sum(tilt_scales) / len(tilt_scales) if tilt_scales else cur_t_scale
        print(f"\n>> Calibrated Tilt Scale ({active_tilt_axis}): {cal_tilt_scale:.3f} steps/degree ({cal_tilt_scale * 360:.1f} steps/rev)")

        # =========================================================================
        # 3. APPLY CALIBRATION AND VERIFY
        # =========================================================================
        print("\n" + "=" * 75)
        print("          PHASE 3: APPLYING CALIBRATION & VERIFICATION")
        print("=" * 75)
        print(f"Sending SET SCALE {cal_pan_scale:.3f} {cal_tilt_scale:.3f} ...")
        self.send_cmd(f"SET SCALE {cal_pan_scale:.3f} {cal_tilt_scale:.3f}")
        self.send_cmd("ZERO")
        time.sleep(0.5)

        # Verification Move 1: Pan +45°
        print("\n--- Verification: Commanding Pan +45.0° ---")
        base_imu = self.get_averaged_imu()
        self.send_cmd("MOVE 45.0 0.0")
        self.wait_for_motion_complete()
        time.sleep(1.5)
        final_imu = self.get_averaged_imu()
        v_pan_meas = signed_angle_diff(final_imu['Y'], base_imu['Y'])
        v_pan_err = v_pan_meas - 45.0
        print(f"  Measured Pan: {v_pan_meas:+.2f}° (Target: +45.0°, Error: {v_pan_err:+.2f}°)")
        self.send_cmd("MOVE 0.0 0.0")
        self.wait_for_motion_complete()
        time.sleep(0.8)

        # Verification Move 2: Tilt +20°
        print("\n--- Verification: Commanding Tilt +20.0° ---")
        base_imu = self.get_averaged_imu()
        self.send_cmd("MOVE 0.0 20.0")
        self.wait_for_motion_complete()
        time.sleep(1.5)
        final_imu = self.get_averaged_imu()
        d_p = final_imu['P'] - base_imu['P']
        d_r = final_imu['R'] - base_imu['R']
        v_tilt_meas = d_r if active_tilt_axis == "Roll" else d_p
        v_tilt_err = v_tilt_meas - 20.0
        print(f"  Measured Tilt ({active_tilt_axis}): {v_tilt_meas:+.2f}° (Target: +20.0°, Error: {v_tilt_err:+.2f}°)")
        self.send_cmd("MOVE 0.0 0.0")
        self.wait_for_motion_complete()

        # Re-enable soft limits and telemetry
        self.send_cmd("SET LIMITS ON")
        self.send_cmd("SET TELEMETRY 10")

        # =========================================================================
        # 4. FINAL SUMMARY TABLE
        # =========================================================================
        print("\n" + "=" * 75)
        print("                   FINAL DUAL-AXIS CALIBRATION REPORT")
        print("=" * 75)
        print(f"PAN AXIS (Azimuth):")
        print(f"  • Calibrated Scale:       {cal_pan_scale:.3f} steps/degree")
        print(f"  • Steps / Full 360° Rev:  {cal_pan_scale * 360.0:.1f} steps")
        print(f"  • Est. Ratio (on 2000 spr): {cal_pan_scale * 360.0 / 2000.0:.2f}:1")
        print("-" * 75)
        print(f"TILT AXIS (Altitude):")
        print(f"  • Active IMU Axis:        {active_tilt_axis}")
        print(f"  • Calibrated Scale:       {cal_tilt_scale:.3f} steps/degree")
        print(f"  • Steps / Full 360° Rev:  {cal_tilt_scale * 360.0:.1f} steps")
        print(f"  • Est. Ratio (on 2000 spr): {cal_tilt_scale * 360.0 / 2000.0:.2f}:1")
        print("=" * 75)
        print("To make this permanent in firmware, update firmware/PanTiltController/Config.h:")
        print(f"   #define PAN_STEPS_PER_DEG   {cal_pan_scale:.3f}f")
        print(f"   #define TILT_STEPS_PER_DEG  {cal_tilt_scale:.3f}f")
        print("=" * 75)
        print("\n✓ Live controller updated with both calibrated scales!")
        print("Serial port closed. You can now reconnect in the Web Dashboard.\n")
        self.ser.close()


if __name__ == "__main__":
    port = find_port()
    calibrator = DualAxisCalibrator(port)
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
