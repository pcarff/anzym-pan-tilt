# Dual-Axis Pan-Tilt Calibration Procedures & Operations Guide

This comprehensive reference document outlines the physical calibration procedures, operating methodologies, and permanent firmware flashing steps for the **Dual-Axis Alt-Azimuth (Pan-Tilt) Stepper Platform** powered by the **Arduino Uno**, **AutomationDirect SureStep STP-DRV-6575** drives, and **Bosch BNO055 9-DOF Absolute IMU**.

---

## 📑 Table of Contents

1. [System Architecture & Mathematics](#1-system-architecture--mathematics)
2. [Method 1: Interactive 2-Point Manual Calibration Studio (Web Dashboard)](#2-method-1-interactive-2-point-manual-calibration-studio-web-dashboard)
   - [Overview & Advantage](#overview--advantage)
   - [Step-by-Step Procedure](#step-by-step-procedure)
   - [Multi-Trial Averaging & Statistical Analysis](#multi-trial-averaging--statistical-analysis)
   - [Live Scale Injection Protocol](#live-scale-injection-protocol)
3. [Method 2: Automated Multi-Run IMU Ground-Truth Calibration (CLI)](#3-method-2-automated-multi-run-imu-ground-truth-calibration-cli)
   - [Prerequisites & Port Arbitration](#prerequisites--port-arbitration)
   - [Running the 10-Iteration Calibration](#running-the-10-iteration-calibration)
   - [Forward-Reverse Backlash Compensation](#forward-reverse-backlash-compensation)
4. [Permanent Firmware Configuration & Flashing](#4-permanent-firmware-configuration--flashing)
   - [Editing `Config.h`](#editing-configh)
   - [Unit Test Verification](#unit-test-verification)
   - [Compiling & Flashing via avrdude](#compiling--flashing-via-avrdude)
5. [Post-Reboot Resumption Checklist](#5-post-reboot-resumption-checklist)

---

## 1. System Architecture & Mathematics

### Hardware Baseline
* **Motors**: 2-Phase Hybrid Stepper Motors, $200\text{ full steps / revolution}$ ($1.8^\circ/\text{step}$).
* **Drivers**: AutomationDirect STP-DRV-6575 microstepping drives set to **10× microstepping** ($2,000\text{ pulses / motor rev}$).
* **Sensors**: Bosch BNO055 9-DOF IMU mounted under-base, running BSX3.0 sensor fusion via hardware I2C (pins `A4`/`A5`).
* **Microcontroller**: Arduino Uno running custom real-time trapezoidal motion firmware at 115,200 baud.

### The Governing Gear Ratio & Step Relationship

$$\text{Steps per Revolution (Axis)} = \text{Steps per Degree} \times 360^\circ$$

$$\text{Gear Reduction Ratio} = \frac{\text{Steps per Revolution}}{2,000\text{ steps/motor rev}} = \frac{\text{Steps per Degree} \times 360^\circ}{2,000}$$

$$\text{Calibrated Scale } (S_{\text{new}}) = \frac{\Delta\text{ Motor Pulse Steps Counted}}{\Delta\text{ True Physical Angle Measured } (^\circ)}$$

* **Pan Baseline**: Nominal 19.826:1 ratio $\to \mathbf{110.145\text{ steps/degree}}$ ($39,652\text{ steps/rev}$).
* **Tilt Baseline**: Nominal 10.668:1 ratio $\to \mathbf{59.267\text{ steps/degree}}$ ($21,336\text{ steps/rev}$).

---

## 2. Method 1: Interactive 2-Point Manual Calibration Studio (Web Dashboard)

### Overview & Advantage
The **2-Point Manual Calibration Studio** is embedded directly within the Web Serial dashboard (`http://localhost:8080`). It provides a complete visual wizard that allows you to:
1. Jog or nudge the platform to a physical zero mark and lock the reference counter.
2. Slew or nudge to a known physical reference angle ($30^\circ$, $45^\circ$, $90^\circ$, $180^\circ$, or custom).
3. Record the end position, compute steps/deg, steps/rev, and gear ratio, and cross-check against the live BNO055 IMU ground truth.
4. Apply the calculated scale directly to the live controller RAM (`SET SCALE`) without losing serial connection or rebooting.
5. Record multiple trials to compute a running mean and standard deviation ($\pm\sigma$).

---

### Step-by-Step Procedure

#### Step A: Launch the Web Dashboard
1. Open a terminal in the project root:
   ```bash
   python3 -m http.server 8080 --directory dashboard
   ```
2. Open **Google Chrome**, **Microsoft Edge**, or an Chromium-based browser supporting the Web Serial API at:
   ```
   http://localhost:8080
   ```
3. Click **"Connect Serial"** in the top-right header and select your Arduino Uno port (`/dev/ttyACM0` or `/dev/ttyUSB0`) at 115200 baud. *(Alternatively, click "Simulator Mode" if testing offline).*

#### Step B: Open the Calibration Studio
1. Click the **`📐 2-Point Calibration Studio`** button in the top navigation bar (or in the System Status panel).
2. Select the target axis tab:
   - **`Pan Axis (Azimuth)`**
   - **`Tilt Axis (Altitude)`**
3. Notice the **Active Scale** banner displays the current steps/degree and steps/revolution active in controller memory.

#### Step C: Step 1 — Set Starting Reference
1. Position the axis to your physical start mark (such as a mechanical index pointer, level mark, or external protractor):
   - Use the **Fine Nudge** buttons (`-10°`, `-1°`, `-0.1°`, `+0.1°`, `+1°`, `+10°`) inside the dialog.
   - Or click **`🔓 Release Motor Coils (Hand Move)`** to de-energize the STP-DRV-6575 holding torque and turn the gimbal by hand, then lock coils again. *(Note: Because STP-DRV-6575 is open-loop, use the fine nudge buttons for the actual measured displacement so the controller counts pulse steps).*
2. Click **`📌 Zero Counter & Set Start Reference`**.
3. **Verified Outcome**:
   - The controller sends `ZERO` to the Arduino.
   - The Elapsed Motor Steps counter resets to `+0 STEPS`.
   - The start status changes to green: `✓ Zero reference set at 0.00° (IMU: X.X°)`.
   - The Step 2 card glows cyan, signaling ready for target movement.

#### Step D: Step 2 — Move to Target & Calculate
1. Slew or nudge the axis from the zero mark to your desired target reference angle (for example, exactly $90.0^\circ$ or $180.0^\circ$).
2. Enter the physical displacement angle:
   - Click one of the quick preset buttons: **`30°`**, **`45°`**, **`90°`**, or **`180°`**.
   - Or type a custom angle into the **Target Angle (°)** input.
   - *Optional*: Check **"Use live BNO055 IMU angle as target"** if you want the high-precision BNO055 sensor fusion delta to serve as the ground truth angle automatically.
3. Click **`🎯 Record Position & Calculate Steps`**.

#### Step E: Review Results & Apply
The **Calculation Result** card will appear with computed metrics:
* **Calibrated Steps / Degree**: $\text{Scale} = \frac{\text{Elapsed Steps}}{\text{Target Angle}}$
* **Steps / Full Rev (360°)**: $\text{Steps/Deg} \times 360$
* **Calculated Gear Ratio**: $\frac{\text{Steps/Rev}}{2,000} : 1$
* **Steps Issued / Angle**: Exact step pulse count and commanded angle.
* **IMU Ground Truth Check**: Real-time angular delta reported by the BNO055 IMU and deviation from target.

**Actions Available**:
* **`⚡ Apply Scale to Live Controller`**: Transmits `SET SCALE <pan> <tilt>` over Web Serial directly into Arduino RAM. The system immediately adopts the new scale without losing position.
* **`➕ Add to Trial Average`**: Adds the trial to the calibration history table for multi-run averaging.

---

### Multi-Trial Averaging & Statistical Analysis
To achieve sub-0.05° tracking precision across gear manufacturing tolerances and belt tension variations:
1. Repeat the calibration sequence **3 to 10 times** in alternating directions (clockwise and counterclockwise).
2. After each run, click **`➕ Add to Trial Average`**.
3. The **Calibration Trial History** table records:
   - Trial Number & Timestamp
   - Axis (Pan vs. Tilt)
   - Target Angle ($^\circ$)
   - Steps Taken
   - Measured Scale ($\text{steps/deg}$)
   - IMU Delta Confirmation
4. The **Running Average Summary Bar** automatically computes:
   - **Running Average Scale**: Arithmetic mean $\mu = \frac{1}{N}\sum_{i=1}^N S_i$
   - **Standard Deviation**: $\sigma = \sqrt{\frac{1}{N}\sum_{i=1}^N (S_i - \mu)^2}$ and relative percentage error
   - **Total Trials Count**
5. Click **`⚡ Apply Averaged Scale`** to commit the statistically filtered scale to the live controller.

---

## 3. Method 2: Automated Multi-Run IMU Ground-Truth Calibration (CLI)

### Prerequisites & Port Arbitration
> [!IMPORTANT]
> The Web Serial API in Chrome holds an exclusive lock on `/dev/ttyACM0`. Before running CLI Python scripts, click **Disconnect** in the web dashboard or close the browser tab.

Check port availability:
```bash
ls -l /dev/ttyACM*
```

### Running the 10-Iteration Calibration
The repository contains standalone automated Python calibration routines:
* [`calibrate_10x.py`](file:///workspaces/anzym_altaz_ws/calibrate_10x.py): Executes 10 consecutive $90^\circ$ and $180^\circ$ calibration cycles on both Pan and Tilt axes, queries IMU Euler orientation registers, and computes the aggregate statistical mean.
* [`calibrate_dual_axis_imu.py`](file:///workspaces/anzym_altaz_ws/calibrate_dual_axis_imu.py): Interactive dual-axis testing harness.

To execute the 10-run automated calibration:
```bash
python3 calibrate_10x.py
```

### Forward-Reverse Backlash Compensation
The script executes moves in alternating pairs:
1. Move Forward $+90.0^\circ$ $\to$ Measure IMU delta $\to$ Calculate Scale.
2. Move Reverse $-90.0^\circ$ $\to$ Measure IMU delta $\to$ Calculate Scale.
3. Move Forward $+180.0^\circ$ $\to$ Measure IMU delta $\to$ Calculate Scale.
4. Move Reverse $-180.0^\circ$ $\to$ Measure IMU delta $\to$ Calculate Scale.

By averaging positive and negative displacement runs, any mechanical gear backlash cancels out of the scale coefficient.

---

## 4. Permanent Firmware Configuration & Flashing

Once the desired scale is verified in the dashboard or via Python, bake the values permanently into the Arduino Uno's flash memory.

### Editing `Config.h`
Open [`firmware/PanTiltController/Config.h`](file:///workspaces/anzym_altaz_ws/firmware/PanTiltController/Config.h) and update lines 50–57 with your calibrated gear ratios:

```cpp
// Mechanical Gear Reduction (Motor revs per 1 axis rev. 1.0 for direct drive)
#define PAN_GEAR_RATIO            19.826f // Calibrated Average (110.145 steps/deg, 39,652 steps/rev)
#define TILT_GEAR_RATIO           10.668f // Calibrated Average (59.267 steps/deg, 21,336 steps/rev)

// Calculated Steps per Degree
// steps_per_deg = (steps_per_rev * microsteps * gear_ratio) / 360.0
#define PAN_STEPS_PER_DEG         ((PAN_MOTOR_STEPS_PER_REV * PAN_MICROSTEPS * PAN_GEAR_RATIO) / 360.0f)
#define TILT_STEPS_PER_DEG        ((TILT_MOTOR_STEPS_PER_REV * TILT_MICROSTEPS * TILT_GEAR_RATIO) / 360.0f)
```

### Unit Test Verification
Before flashing physical hardware, run the C++ unit test suite to ensure motion profiles and soft limit clamping remain 100% compliant:
```bash
npm run test:firmware
```
Expected output:
```
========================================
Starting Pan-Tilt Firmware Unit Tests
========================================
[TEST] Running MotionController tests...
  ✓ Position move reached: Pan=45.0045°, Tilt=30°
  ✓ Soft limits successfully clamped tilt to 90°
  ✓ Jog and smooth deceleration verified
  ✓ Emergency stop verified
[TEST] Running CommandParser tests...
  ✓ PING -> PONG acknowledged
  ✓ MOVE 15.5 -10.0 parsed and executed
  ✓ JOG command executed
  ✓ STOP command executed
  ✓ GET STATUS returned valid telemetry
  ✓ ZERO command reset coordinates
========================================
ALL FIRMWARE TESTS PASSED SUCCESSFULLY! ✓
========================================
```

### Compiling & Flashing via avrdude
Ensure serial terminals/browsers are disconnected, then run:
```bash
./upload_firmware.sh
```
Or manually via Arduino CLI:
```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/PanTiltController -e
avrdude -v -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 -D -U flash:w:build/firmware/PanTiltController.ino.hex:i
```

---

## 5. Post-Reboot Resumption Checklist

When restarting this computer to apply system updates, resume your tracking environment with this 3-step checklist:

1. **Verify Serial Device Recognition**:
   ```bash
   ls -l /dev/ttyACM*
   ```
   *(If permission is denied: `sudo usermod -a -G dialout $USER` and log back in).*

2. **Start the Web Dashboard**:
   ```bash
   cd /workspaces/anzym_altaz_ws
   python3 -m http.server 8080 --directory dashboard
   ```

3. **Connect & Zero Hardware**:
   - Open `http://localhost:8080` in Google Chrome or Microsoft Edge.
   - Click **Connect Serial** $\to$ select the Arduino Uno port.
   - Click **`⚡ Sync to IMU`** or **`🎯 Calibrate (0°, 0°)`** to initialize coordinates.
   - Launch **`📐 2-Point Calibration Studio`** at any time to inspect or re-tune axis scales.
