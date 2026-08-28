# Dual-Axis Pan-Tilt (Alt-Azimuth) Stepper Controller

A professional dual-axis stepper motor controller and real-time dashboard for Pan-Tilt (Azimuth-Elevation) tracking and positioning systems. Built for Arduino Uno interfacing with **AutomationDirect SureStep STP-DRV-6575** microstepping drives.

---

## 🌟 Key Features

- **Firmware (Arduino C++)**:
  - Smooth trapezoidal acceleration and velocity profiling.
  - Open-loop step position tracking with engineering degree ($^\circ$) unit conversion.
  - Continuous velocity jogging (interactive joystick control).
  - Configurable soft travel limits (prevent cable wrap / mechanical collisions).
  - Limit switch homing state machine with debounce.
  - Extensible sensor hooks for I2C IMU (MPU6050, BNO055) and auxiliary inputs.
  - 115200 baud ASCII serial protocol with high-frequency (10–20 Hz) telemetry streaming.
- **Dashboard (Web & Web Serial)**:
  - Direct USB Web Serial connection (no backend server required!).
  - Built-in **Simulator Mode** for instant interactive testing without hardware.
  - Interactive **3D Gimbal Visualizer** (Three.js) showing live physical orientation, compass heading, and target reticle.
  - **2D Virtual Joystick** for smooth velocity jogging.
  - **Precision Step Nudge D-Pad** ($0.1^\circ, 0.5^\circ, 1.0^\circ, 5.0^\circ, 10.0^\circ$).
  - **Absolute Target Slew** with sliders and direct degree inputs.
  - **Waypoint & Preset Manager** with persistent browser storage.
  - **Integrated Serial Terminal** with command history.
  - Instant Emergency Stop (`E-STOP`).

---

## 🔌 Hardware Wiring Guide (STP-DRV-6575 & Arduino Uno)

### Pin Mapping Table

| Arduino Uno Pin | Function | STP-DRV-6575 Terminal | Notes |
|---|---|---|---|
| `D2` | Pan STEP (Pulse) | **STEP+** (Drive 1 - Pan) | 5V Step Pulse (Active High) |
| `D3` | Pan DIR (Direction) | **DIR+** (Drive 1 - Pan) | High = CW, Low = CCW |
| `D4` | Pan EN (Enable) | **EN+** (Drive 1 - Pan) | High = Energized, Low = Free |
| `D5` | Tilt STEP (Pulse) | **STEP+** (Drive 2 - Tilt) | 5V Step Pulse (Active High) |
| `D6` | Tilt DIR (Direction) | **DIR+** (Drive 2 - Tilt) | High = Up, Low = Down |
| `D7` | Tilt EN (Enable) | **EN+** (Drive 2 - Tilt) | High = Energized, Low = Free |
| `GND` | Common Ground | **STEP-**, **DIR-**, **EN-** (Both Drives) | Common Cathode wiring |
| `D8` | Pan Limit Switch | Switch NO $\to$ `D8`, Common $\to$ `GND` | Internal pull-up enabled |
| `D9` | Tilt Limit Switch | Switch NO $\to$ `D9`, Common $\to$ `GND` | Internal pull-up enabled |
| `A4` (SDA) | I2C Data | Future IMU / Sensors SDA | Optional |
| `A5` (SCL) | I2C Clock | Future IMU / Sensors SCL | Optional |

### STP-DRV-6575 DIP Switch Settings
- **Current (Amps)**: Set switches according to your stepper motor rated phase current.
- **Microstepping**: Recommended **8x microstepping** (1600 steps/rev) or **16x microstepping** (3200 steps/rev).
- Match the microstepping in [`firmware/PanTiltController/Config.h`](file:///workspaces/anzym_altaz_ws/firmware/PanTiltController/Config.h) (`PAN_MICROSTEPS` and `TILT_MICROSTEPS`).

---

## 🚀 Getting Started

### 1. Flashing the Arduino Firmware
1. Open the Arduino IDE.
2. Open [`firmware/PanTiltController/PanTiltController.ino`](file:///workspaces/anzym_altaz_ws/firmware/PanTiltController/PanTiltController.ino).
3. Select **Board**: `Arduino Uno` and choose your serial port.
4. Click **Upload**.

### 2. Launching the Dashboard
The dashboard runs entirely in the browser using the **Web Serial API** (Google Chrome, Microsoft Edge, Opera).

To run locally:
```bash
# Option A: Python simple HTTP server
python3 -m http.server 8080 --directory dashboard

# Option B: Node.js npx serve
npx serve dashboard
```
Then open `http://localhost:8080` in Chrome/Edge.

- Click **"Connect Serial"** to connect to your Arduino Uno.
- Or click **"Simulator Mode"** to test all 3D visuals, joystick controls, presets, and commands immediately without physical hardware!

---

## 📡 Serial Command Reference

Baud Rate: `115200` | Line Ending: `\n` or `\r\n`

| Command | Example | Description |
|---|---|---|
| `MOVE <pan> <tilt>` | `MOVE 45.0 15.0` | Move to absolute angles in degrees |
| `MOVEREL <d_pan> <d_tilt>` | `MOVEREL 1.0 -2.5` | Move relative by delta degrees |
| `JOG <p_spd> <t_spd>` | `JOG 20.0 -10.0` | Continuous velocity jog in deg/s (`JOG 0 0` to stop) |
| `STOP` | `STOP` | Decelerate to smooth stop |
| `ESTOP` | `ESTOP` | Immediate emergency stop and disable |
| `ENABLE` / `DISABLE` | `ENABLE` | Energize or de-energize stepper coils |
| `ZERO` | `ZERO` | Reset current position as $(0^\circ, 0^\circ)$ |
| `SETPOS <pan> <tilt>` | `SETPOS 0 45` | Set custom coordinate reference |
| `HOME [ALL\|PAN\|TILT]` | `HOME ALL` | Execute limit switch homing routine |
| `SET SPEED <p> <t>` | `SET SPEED 60 40` | Set max velocity in deg/s |
| `SET ACCEL <p> <t>` | `SET ACCEL 120 80` | Set acceleration in $\text{deg/s}^2$ |
| `SET LIMITS <ON\|OFF>` | `SET LIMITS ON` | Toggle soft travel limits |
| `GET POS` | `GET POS` | Returns `POS P=<pan> T=<tilt>` |
| `GET STATUS` | `GET STATUS` | Returns telemetry packet |
| `PING` | `PING` | Returns `PONG` |

### Telemetry Packet Format (Broadcast at 10 Hz)
```
STATUS P=12.50 T=-4.20 TP=12.50 TT=-4.20 SP=0.0 ST=0.0 MV=0 EN=1 LP=0 LT=0
```
- `P`, `T`: Current Pan and Tilt angles ($^\circ$)
- `TP`, `TT`: Target Pan and Tilt angles ($^\circ$)
- `SP`, `ST`: Current speed in deg/s
- `MV`: Moving flag (1 = active, 0 = idle)
- `EN`: Drive enable flag (1 = enabled, 0 = disabled)
- `LP`, `LT`: Pan and Tilt limit switch states (1 = triggered)

---

## 📐 Angular Scale & Precision Calibration Guide

When tuning your pan-tilt platform to match the commanded degrees ($^\circ$) with 1:1 physical accuracy, use the following formulas:

### 1. Empirical Measurement Calibration Formula
If you command a move and measure the actual physical rotation using a digital angle gauge, protractor, or index mark:

$$\mathbf{S_{\text{new}} = S_{\text{current}} \times \left( \frac{\theta_{\text{commanded}}}{\theta_{\text{measured}}} \right)}$$

* $\mathbf{S_{\text{current}}}$: Current Steps/Degree setting in dashboard (e.g. `111.111`).
* $\mathbf{\theta_{\text{commanded}}}$: Commanded target angle in degrees (e.g. `90.0°` or `360.0°`).
* $\mathbf{\theta_{\text{measured}}}$: Actual physical angle rotated as measured on the platform.
* $\mathbf{S_{\text{new}}}$: Exact calibrated Steps/Degree to enter into the dashboard or `Config.h`.

#### Calibration Example:
1. Current setting: **$111.111\text{ steps/deg}$**.
2. You click **"Test 90° Pan"** ($\theta_{\text{commanded}} = 90^\circ$).
3. You measure the physical platform rotated **$86.5^\circ$** ($\theta_{\text{measured}} = 86.5^\circ$).
4. Calculate:
   $$S_{\text{new}} = 111.111 \times \left(\frac{90.0}{86.5}\right) = \mathbf{115.606\text{ steps/degree}}$$
5. Enter **`115.606`** into the **Pan (Steps/°)** box in the dashboard and click **Apply Scale**.

---

### 2. Theoretical Mechanical Formula (From Gear Ratios)

$$\mathbf{\text{Steps per Degree} = \frac{\text{Motor Steps/Rev} \times \text{Microstepping} \times \text{Gear Ratio}}{360^\circ}}$$

For our configuration:
* **Wantai Motor**: $200\text{ full steps / rev}$ ($1.8^\circ/\text{step}$).
* **STP-DRV-6575 Drive**: $10\times\text{ microstepping}$ ($2,000\text{ steps / motor rev}$).

$$\mathbf{\text{Steps per Degree} = \frac{2,000 \times \text{Gear Ratio}}{360^\circ} = \frac{50}{9} \times \text{Gear Ratio} \approx 5.55556 \times \text{Gear Ratio}}$$

| Gear Ratio | Steps per Degree ($^\circ$) | Microsteps per 90° Quarter Turn |
|:---:|:---:|:---:|
| **1:1** (Direct Drive) | `5.556` | 500 steps |
| **5:1** (e.g. 16T to 80T belt) | `27.778` | 2,500 steps |
| **10:1** (e.g. 16T to 160T belt) | `55.556` | 5,000 steps |
| **16:1** | `88.889` | 8,000 steps |
| **18:1** | `100.000` | 9,000 steps |
| **20:1** | `111.111` | 10,000 steps |
| **24:1** | `133.333` | 12,000 steps |
| **30:1** | `166.667` | 15,000 steps |
| **36:1** | `200.000` | 18,000 steps |

---

### 3. High-Precision 360° Full-Turn Calibration Method
1. Place a piece of tape on the rotating platform and stationary base with a fine pen line aligned at $0^\circ$.
2. Command a full $360.0^\circ$ rotation:
   * Send `MOVE 360 0` in the serial console (or `MOVEREL 360 0`).
3. If the line finishes short or past the mark, note the angle difference and apply the empirical formula. Over $360^\circ$, errors are multiplied by 4, giving sub-0.05° precision!

---

## 🧪 Testing and Verification
Unit tests for the motion profiling algorithms, soft limits, coordinate transformations, and command parser can be executed locally:
```bash
g++ -std=c++17 -I./tests -I./firmware/PanTiltController tests/test_firmware.cpp firmware/PanTiltController/MotionController.cpp firmware/PanTiltController/SensorManager.cpp firmware/PanTiltController/CommandParser.cpp -o test_runner && ./test_runner
```

