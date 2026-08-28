/**
 * app.js
 * 
 * Main application coordinator for Pan-Tilt Controller Dashboard.
 * Integrates Web Serial, 3D Visualizer, Joystick Physics, D-Pad, Presets, and Console.
 */

document.addEventListener('DOMContentLoaded', () => {
    // --- Instances ---
    const serial = window.serialController;
    const visualizer = new window.GimbalVisualizer('gl-canvas', 'visualizer-container');
    visualizer.init();

    // --- State ---
    let currentStepSize = 1.0;
    let drivesEnabled = true;
    let softLimitsEnabled = true;
    let telemetryCount = 0;
    let lastTelemetryCalc = Date.now();
    let commandHistory = [];
    let historyIndex = -1;

    // Default Presets
    const DEFAULT_PRESETS = [
        { id: '1', name: 'Home Origin', pan: 0.0, tilt: 0.0 },
        { id: '2', name: 'North Horizon', pan: 0.0, tilt: 0.0 },
        { id: '3', name: 'East Sky', pan: 90.0, tilt: 45.0 },
        { id: '4', name: 'Zenith', pan: 0.0, tilt: 90.0 },
        { id: '5', name: 'West Horizon', pan: -90.0, tilt: 0.0 }
    ];
    let presets = JSON.parse(localStorage.getItem('pan_tilt_presets')) || DEFAULT_PRESETS;

    // --- DOM Elements ---
    const btnConnect = document.getElementById('btn-connect');
    const connectText = document.getElementById('connect-text');
    const btnMockMode = document.getElementById('btn-mock-mode');
    const baudSelect = document.getElementById('baud-select');
    const connStatus = document.getElementById('conn-status');
    const driveStatus = document.getElementById('drive-status');
    const motionStatus = document.getElementById('motion-status');
    const btnEstop = document.getElementById('btn-estop');

    // Telemetry displays
    const valPanDeg = document.getElementById('val-pan-deg');
    const valTiltDeg = document.getElementById('val-tilt-deg');
    const valTgtPan = document.getElementById('val-tgt-pan');
    const valTgtTilt = document.getElementById('val-tgt-tilt');
    const valSpdPan = document.getElementById('val-spd-pan');
    const valSpdTilt = document.getElementById('val-spd-tilt');
    const valTelemetryRate = document.getElementById('val-telemetry-rate');
    const panLimitPill = document.getElementById('pan-limit-pill');
    const tiltLimitPill = document.getElementById('tilt-limit-pill');
    const limitsPill = document.getElementById('limits-pill');

    // Absolute Target Inputs
    const sliderTargetPan = document.getElementById('slider-target-pan');
    const inputTargetPan = document.getElementById('input-target-pan');
    const sliderTargetTilt = document.getElementById('slider-target-tilt');
    const inputTargetTilt = document.getElementById('input-target-tilt');
    const btnSlewTarget = document.getElementById('btn-slew-target');
    const btnStopMotion = document.getElementById('btn-stop-motion');

    // Terminal
    const terminalOutput = document.getElementById('terminal-output');
    const terminalForm = document.getElementById('terminal-form');
    const terminalInput = document.getElementById('terminal-input');
    const btnClearTerm = document.getElementById('btn-clear-term');
    const chkAutoscroll = document.getElementById('chk-autoscroll');

    // Motion & Control Actions
    const btnToggleEnable = document.getElementById('btn-toggle-enable');
    const btnZeroAxes = document.getElementById('btn-zero-axes');
    const btnResetCam = document.getElementById('btn-reset-cam');
    const btnCenterView = document.getElementById('btn-center-view');

    // Settings
    const inputPanMaxSpd = document.getElementById('input-pan-max-spd');
    const inputTiltMaxSpd = document.getElementById('input-tilt-max-spd');
    const inputPanAccel = document.getElementById('input-pan-accel');
    const inputTiltAccel = document.getElementById('input-tilt-accel');
    const btnApplySettings = document.getElementById('btn-apply-settings');
    const btnToggleLimits = document.getElementById('btn-toggle-limits');
    const btnHomeAll = document.getElementById('btn-home-all');

    // Presets
    const presetsList = document.getElementById('presets-list');
    const btnSaveCurrentPreset = document.getElementById('btn-save-current-preset');

    // IMU Elements
    const valImuPitch = document.getElementById('val-imu-pitch');
    const valImuYaw = document.getElementById('val-imu-yaw');
    const valImuRoll = document.getElementById('val-imu-roll');
    const valImuCardinal = document.getElementById('val-imu-cardinal');
    const imuStatusBadge = document.getElementById('imu-status-badge');
    const pillCalSys = document.getElementById('pill-cal-sys');
    const pillCalGyr = document.getElementById('pill-cal-gyr');
    const pillCalAcc = document.getElementById('pill-cal-acc');
    const pillCalMag = document.getElementById('pill-cal-mag');
    const btnSyncImu = document.getElementById('btn-sync-imu');
    const selectImuRemap = document.getElementById('select-imu-remap');
    let lastRemappedImu = null;

    function getCardinalDirection(deg) {
        const d = ((deg % 360) + 360) % 360;
        const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
        const index = Math.round(d / 22.5) % 16;
        return directions[index];
    }

    // --- Helper: Terminal Logging ---
    function logTerminal(text, type = 'rx') {
        const line = document.createElement('div');
        line.className = `term-line ${type}`;
        const time = new Date().toLocaleTimeString();
        line.textContent = `[${time}] ${text}`;
        terminalOutput.appendChild(line);

        if (chkAutoscroll.checked) {
            terminalOutput.scrollTop = terminalOutput.scrollHeight;
        }

        // Limit terminal history to 200 lines
        while (terminalOutput.children.length > 200) {
            terminalOutput.removeChild(terminalOutput.firstChild);
        }
    }

    // --- Serial Callbacks ---
    serial.onConnect = (info) => {
        if (info.mock) {
            connStatus.textContent = 'Simulator';
            connStatus.className = 'badge badge-sim';
            connectText.textContent = 'Disconnect';
            btnMockMode.classList.add('btn-primary');
            logTerminal('[SIM] Running high-fidelity Virtual Stepper Controller', 'sys');
        } else {
            connStatus.textContent = 'Connected';
            connStatus.className = 'badge badge-connected';
            connectText.textContent = 'Disconnect';
            logTerminal('[SYS] Connected to Hardware Serial Port', 'sys');
        }
    };

    serial.onDisconnect = () => {
        connStatus.textContent = 'Disconnected';
        connStatus.className = 'badge badge-disconnected';
        connectText.textContent = 'Connect Serial';
        btnMockMode.classList.remove('btn-primary');
        motionStatus.textContent = 'State: Idle';
        motionStatus.className = 'badge badge-idle';
        if (imuStatusBadge) {
            imuStatusBadge.textContent = 'Offline';
            imuStatusBadge.className = 'badge-sm badge-inactive';
        }
        logTerminal('[SYS] Disconnected', 'sys');
    };

    serial.onData = (line) => {
        if (!line.startsWith('STATUS ')) {
            logTerminal(line, 'rx');
        }
    };

    serial.onStatus = (status) => {
        // Update Telemetry rate
        telemetryCount++;
        const now = Date.now();
        if (now - lastTelemetryCalc >= 1000) {
            valTelemetryRate.textContent = telemetryCount;
            telemetryCount = 0;
            lastTelemetryCalc = now;
        }

        // Update Digits
        valPanDeg.textContent = status.pan.toFixed(2);
        valTiltDeg.textContent = status.tilt.toFixed(2);
        valTgtPan.textContent = status.targetPan.toFixed(2);
        valTgtTilt.textContent = status.targetTilt.toFixed(2);
        valSpdPan.textContent = status.speedPan.toFixed(1);
        valSpdTilt.textContent = status.speedTilt.toFixed(1);

        // Update Motion Status Badge
        if (status.moving) {
            motionStatus.textContent = 'State: Moving';
            motionStatus.className = 'badge badge-moving';
        } else {
            motionStatus.textContent = 'State: Idle';
            motionStatus.className = 'badge badge-idle';
        }

        // Update Drive Enable status
        drivesEnabled = status.enabled;
        driveStatus.textContent = status.enabled ? 'Drives: ON' : 'Drives: OFF';
        driveStatus.className = status.enabled ? 'badge badge-connected' : 'badge badge-disconnected';
        btnToggleEnable.textContent = status.enabled ? 'Disable Drives' : 'Enable Drives';

        // Limit switches
        panLimitPill.className = status.limitPan ? 'pill pill-triggered' : 'pill pill-off';
        tiltLimitPill.className = status.limitTilt ? 'pill pill-triggered' : 'pill pill-off';

        // IMU Telemetry Updates
        if (status.imuAvailable && valImuPitch) {
            imuStatusBadge.textContent = 'Active (9-DOF)';
            imuStatusBadge.className = 'badge-sm badge-active';

            let pitch = status.imuPitch;
            let roll = status.imuRoll;
            let yaw = status.imuYaw;

            const remapMode = selectImuRemap ? selectImuRemap.value : 'rot90_180_neg';

            switch (remapMode) {
                case 'rot90_180_neg':
                    // Under-base sideways: 180° flipped heading, negative roll for tilt
                    pitch = -status.imuRoll;
                    roll = -status.imuPitch;
                    yaw = (status.imuYaw + 180) % 360;
                    break;
                case 'rot90_180_pos':
                    // Under-base sideways: 180° flipped heading, positive roll for tilt
                    pitch = status.imuRoll;
                    roll = status.imuPitch;
                    yaw = (status.imuYaw + 180) % 360;
                    break;
                case 'rot90_0_pos':
                    // Under-base sideways: +0° heading, positive roll for tilt
                    pitch = status.imuRoll;
                    roll = status.imuPitch;
                    yaw = status.imuYaw;
                    break;
                case 'rot90_0_neg':
                    // Under-base sideways: +0° heading, negative roll for tilt
                    pitch = -status.imuRoll;
                    roll = status.imuPitch;
                    yaw = status.imuYaw;
                    break;
                case 'rot90_90':
                    // Under-base sideways: +90° heading, negative roll for tilt
                    pitch = -status.imuRoll;
                    roll = status.imuPitch;
                    yaw = (status.imuYaw + 90) % 360;
                    break;
                case 'rot90_270':
                    // Under-base sideways: +270° heading, negative roll for tilt
                    pitch = -status.imuRoll;
                    roll = status.imuPitch;
                    yaw = (status.imuYaw + 270) % 360;
                    break;
                case 'flat_180':
                    // Flat mounted: 180° flipped heading, negative pitch for tilt
                    pitch = -status.imuPitch;
                    roll = -status.imuRoll;
                    yaw = (status.imuYaw + 180) % 360;
                    break;
                case 'flat_0':
                default:
                    pitch = status.imuPitch;
                    roll = status.imuRoll;
                    yaw = status.imuYaw;
                    break;
            }

            if (yaw < 0) yaw += 360;
            if (yaw >= 360) yaw -= 360;

            lastRemappedImu = { pitch, roll, yaw };

            valImuPitch.textContent = pitch.toFixed(1);
            valImuYaw.textContent = yaw.toFixed(1);
            valImuRoll.textContent = roll.toFixed(1);

            if (valImuCardinal) {
                valImuCardinal.textContent = getCardinalDirection(yaw);
            }

            pillCalSys.textContent = `SYS: ${status.calSys}`;
            pillCalSys.className = `cal-pill cal-${status.calSys}`;
            pillCalGyr.textContent = `GYR: ${status.calGyro}`;
            pillCalGyr.className = `cal-pill cal-${status.calGyro}`;
            pillCalAcc.textContent = `ACC: ${status.calAccel}`;
            pillCalAcc.className = `cal-pill cal-${status.calAccel}`;
            pillCalMag.textContent = `MAG: ${status.calMag}`;
            pillCalMag.className = `cal-pill cal-${status.calMag}`;
        } else if (imuStatusBadge) {
            imuStatusBadge.textContent = 'Offline';
            imuStatusBadge.className = 'badge-sm badge-inactive';
        }

        // Update 3D visualizer
        visualizer.updateAngles(status.pan, status.tilt, status.targetPan, status.targetTilt);
    };

    serial.onError = (err) => {
        logTerminal(`[ERROR] ${err.message || err}`, 'err');
    };

    // --- Command Dispatcher ---
    async function send(cmd) {
        try {
            logTerminal(cmd, 'tx');
            await serial.sendCommand(cmd);
        } catch (e) {
            logTerminal(`Failed to send: ${e.message}`, 'err');
        }
    }

    // --- Connect / Disconnect / Mock ---
    btnConnect.addEventListener('click', async () => {
        if (serial.isConnected) {
            await serial.disconnect();
        } else {
            try {
                const baud = baudSelect.value;
                await serial.connect(baud);
            } catch (err) {
                console.error(err);
            }
        }
    });

    btnMockMode.addEventListener('click', () => {
        if (serial.isMockMode) {
            serial.disconnect();
        } else {
            serial.startSimulator();
        }
    });

    btnEstop.addEventListener('click', () => {
        send('ESTOP');
    });

    // --- 2D Virtual Joystick ---
    const joyBase = document.getElementById('joystick-base');
    const joyStick = document.getElementById('joystick-stick');
    let isDraggingJoy = false;
    let joyInterval = null;
    let joyX = 0; // -1 to 1
    let joyY = 0; // -1 to 1
    const MAX_JOY_RADIUS = 55; // Pixels

    function handleJoyStart(e) {
        e.preventDefault();
        isDraggingJoy = true;
        handleJoyMove(e);

        if (!joyInterval) {
            joyInterval = setInterval(() => {
                if (isDraggingJoy && (Math.abs(joyX) > 0.05 || Math.abs(joyY) > 0.05)) {
                    const maxPan = parseFloat(inputPanMaxSpd.value) || 45;
                    const maxTilt = parseFloat(inputTiltMaxSpd.value) || 30;
                    const panSpd = (joyX * maxPan).toFixed(1);
                    const tiltSpd = (joyY * maxTilt).toFixed(1);
                    send(`JOG ${panSpd} ${tiltSpd}`);
                }
            }, 80); // Send JOG packet every 80ms
        }
    }

    function handleJoyMove(e) {
        if (!isDraggingJoy) return;
        const rect = joyBase.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        const clientX = e.touches ? e.touches[0].clientX : e.clientX;
        const clientY = e.touches ? e.touches[0].clientY : e.clientY;

        let dx = clientX - centerX;
        let dy = clientY - centerY;

        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > MAX_JOY_RADIUS) {
            dx = (dx / dist) * MAX_JOY_RADIUS;
            dy = (dy / dist) * MAX_JOY_RADIUS;
        }

        joyStick.style.transform = `translate(${dx}px, ${dy}px)`;

        joyX = dx / MAX_JOY_RADIUS;
        joyY = -dy / MAX_JOY_RADIUS; // Invert Y so up is positive tilt
    }

    function handleJoyEnd() {
        if (!isDraggingJoy) return;
        isDraggingJoy = false;
        joyX = 0;
        joyY = 0;
        joyStick.style.transform = `translate(0px, 0px)`;

        if (joyInterval) {
            clearInterval(joyInterval);
            joyInterval = null;
        }
        send('STOP');
    }

    joyBase.addEventListener('mousedown', handleJoyStart);
    window.addEventListener('mousemove', handleJoyMove);
    window.addEventListener('mouseup', handleJoyEnd);

    joyBase.addEventListener('touchstart', handleJoyStart, { passive: false });
    window.addEventListener('touchmove', handleJoyMove, { passive: false });
    window.addEventListener('touchend', handleJoyEnd);

    // --- Step Nudge D-Pad ---
    document.querySelectorAll('.step-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.step-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            currentStepSize = parseFloat(btn.dataset.step) || 1.0;
        });
    });

    document.getElementById('btn-nudge-up').addEventListener('click', () => {
        send(`MOVEREL 0 ${currentStepSize}`);
    });
    document.getElementById('btn-nudge-down').addEventListener('click', () => {
        send(`MOVEREL 0 -${currentStepSize}`);
    });
    document.getElementById('btn-nudge-left').addEventListener('click', () => {
        send(`MOVEREL -${currentStepSize} 0`);
    });
    document.getElementById('btn-nudge-right').addEventListener('click', () => {
        send(`MOVEREL ${currentStepSize} 0`);
    });
    document.getElementById('btn-nudge-center').addEventListener('click', () => {
        send('MOVE 0 0');
    });

    // --- Absolute Slew Targets ---
    sliderTargetPan.addEventListener('input', () => {
        inputTargetPan.value = parseFloat(sliderTargetPan.value).toFixed(1);
    });
    inputTargetPan.addEventListener('input', () => {
        sliderTargetPan.value = inputTargetPan.value;
    });

    sliderTargetTilt.addEventListener('input', () => {
        inputTargetTilt.value = parseFloat(sliderTargetTilt.value).toFixed(1);
    });
    inputTargetTilt.addEventListener('input', () => {
        sliderTargetTilt.value = inputTargetTilt.value;
    });

    btnSlewTarget.addEventListener('click', () => {
        const p = parseFloat(inputTargetPan.value) || 0;
        const t = parseFloat(inputTargetTilt.value) || 0;
        send(`MOVE ${p} ${t}`);
    });

    btnStopMotion.addEventListener('click', () => {
        send('STOP');
    });

    // --- Actions & Buttons ---
    btnToggleEnable.addEventListener('click', () => {
        if (drivesEnabled) {
            send('DISABLE');
        } else {
            send('ENABLE');
        }
    });

    // --- Calibrate / Zero Action ---
    function calibrateZero() {
        send('ZERO');
        valPanDeg.textContent = '0.00';
        valTiltDeg.textContent = '0.00';
        valTgtPan.textContent = '0.00';
        valTgtTilt.textContent = '0.00';
        sliderTargetPan.value = 0;
        inputTargetPan.value = '0.0';
        sliderTargetTilt.value = 0;
        inputTargetTilt.value = '0.0';
        visualizer.updateAngles(0, 0, 0, 0);
        logTerminal('[CALIBRATION] Calibrated Home Reference: Origin set to (0.00°, 0.00°). 3D Visualizer realigned.', 'sys');
    }

    const btnCalibrateZero = document.getElementById('btn-calibrate-zero');
    const btnQuickCalibrate = document.getElementById('btn-quick-calibrate');
    if (btnCalibrateZero) btnCalibrateZero.addEventListener('click', calibrateZero);
    if (btnQuickCalibrate) btnQuickCalibrate.addEventListener('click', calibrateZero);
    if (btnZeroAxes) btnZeroAxes.addEventListener('click', calibrateZero);

    btnResetCam.addEventListener('click', () => visualizer.resetCamera());
    btnCenterView.addEventListener('click', () => send('MOVE 0 0'));

    // Settings
    btnApplySettings.addEventListener('click', () => {
        const spdP = inputPanMaxSpd.value;
        const spdT = inputTiltMaxSpd.value;
        const accP = inputPanAccel.value;
        const accT = inputTiltAccel.value;
        send(`SET SPEED ${spdP} ${spdT}`);
        send(`SET ACCEL ${accP} ${accT}`);
    });

    // --- Live Calibration & Alignment ---
    const inputPanStepsDeg = document.getElementById('input-pan-steps-deg');
    const inputTiltStepsDeg = document.getElementById('input-tilt-steps-deg');
    const chkInvertPan = document.getElementById('chk-invert-pan');
    const chkInvertTilt = document.getElementById('chk-invert-tilt');
    const btnApplyCalibration = document.getElementById('btn-apply-calibration');
    const btnTest90Pan = document.getElementById('btn-test-90-pan');
    const btnTest45Tilt = document.getElementById('btn-test-45-tilt');

    if (btnApplyCalibration) {
        btnApplyCalibration.addEventListener('click', () => {
            const panScale = parseFloat(inputPanStepsDeg.value) || 5.556;
            const tiltScale = parseFloat(inputTiltStepsDeg.value) || 5.556;
            const invPan = chkInvertPan.checked ? 1 : 0;
            const invTilt = chkInvertTilt.checked ? 1 : 0;

            send(`SET SCALE ${panScale} ${tiltScale}`);
            send(`SET INVERT ${invPan} ${invTilt}`);
            logTerminal(`[CALIBRATION] Applied: Pan=${panScale} steps/°, Tilt=${tiltScale} steps/°, InvPan=${invPan}, InvTilt=${invTilt}`, 'sys');
        });
    }

    if (btnTest90Pan) {
        btnTest90Pan.addEventListener('click', () => {
            logTerminal('[CALIBRATION] Executing 90° Pan Scale Test (Pan=90°)...', 'sys');
            send('MOVE P=90');
        });
    }

    if (btnTest45Tilt) {
        btnTest45Tilt.addEventListener('click', () => {
            logTerminal('[CALIBRATION] Executing 45° Tilt Scale Test (Tilt=45°)...', 'sys');
            send('MOVE T=45');
        });
    }

    if (btnSyncImu) {
        btnSyncImu.addEventListener('click', () => {
            if (lastRemappedImu) {
                let panTarget = lastRemappedImu.yaw;
                if (panTarget > 180) {
                    panTarget = panTarget - 360;
                }
                logTerminal(`[IMU] Synchronizing stepper coordinates to IMU: Pan=${panTarget.toFixed(1)}° (${lastRemappedImu.yaw.toFixed(1)}°), Tilt=${lastRemappedImu.pitch.toFixed(1)}°`, 'sys');
                send(`SETPOS ${panTarget.toFixed(1)} ${lastRemappedImu.pitch.toFixed(1)}`);
            } else {
                send('SYNC IMU');
            }
        });
    }

    btnToggleLimits.addEventListener('click', () => {
        softLimitsEnabled = !softLimitsEnabled;
        limitsPill.textContent = softLimitsEnabled ? 'Soft Limits ON' : 'Soft Limits OFF';
        limitsPill.className = softLimitsEnabled ? 'pill pill-active' : 'pill pill-off';
        send(`SET LIMITS ${softLimitsEnabled ? 'ON' : 'OFF'}`);
    });

    btnHomeAll.addEventListener('click', () => {
        send('HOME ALL');
    });

    // --- Presets / Waypoints ---
    function renderPresets() {
        presetsList.innerHTML = '';
        presets.forEach(p => {
            const item = document.createElement('div');
            item.className = 'preset-item';
            item.innerHTML = `
                <div>
                    <div class="preset-name">${p.name}</div>
                    <div class="preset-coords">P: ${p.pan.toFixed(1)}° | T: ${p.tilt.toFixed(1)}°</div>
                </div>
                <div class="preset-actions">
                    <button class="btn-sm btn-action btn-slew-preset" data-pan="${p.pan}" data-tilt="${p.tilt}">Slew</button>
                    <button class="btn-sm btn-ghost btn-del-preset" data-id="${p.id}">✕</button>
                </div>
            `;
            presetsList.appendChild(item);
        });

        document.querySelectorAll('.btn-slew-preset').forEach(btn => {
            btn.addEventListener('click', () => {
                const pan = btn.dataset.pan;
                const tilt = btn.dataset.tilt;
                sliderTargetPan.value = pan;
                inputTargetPan.value = pan;
                sliderTargetTilt.value = tilt;
                inputTargetTilt.value = tilt;
                send(`MOVE ${pan} ${tilt}`);
            });
        });

        document.querySelectorAll('.btn-del-preset').forEach(btn => {
            btn.addEventListener('click', () => {
                const id = btn.dataset.id;
                presets = presets.filter(item => item.id !== id);
                localStorage.setItem('pan_tilt_presets', JSON.stringify(presets));
                renderPresets();
            });
        });
    }

    btnSaveCurrentPreset.addEventListener('click', () => {
        const name = prompt('Enter name for current position preset:');
        if (name && name.trim()) {
            const pan = parseFloat(valPanDeg.textContent) || 0;
            const tilt = parseFloat(valTiltDeg.textContent) || 0;
            presets.push({
                id: Date.now().toString(),
                name: name.trim(),
                pan,
                tilt
            });
            localStorage.setItem('pan_tilt_presets', JSON.stringify(presets));
            renderPresets();
        }
    });

    renderPresets();

    // --- Serial Console Form ---
    terminalForm.addEventListener('submit', (e) => {
        e.preventDefault();
        const cmd = terminalInput.value.trim();
        if (cmd) {
            commandHistory.push(cmd);
            historyIndex = commandHistory.length;
            send(cmd);
            terminalInput.value = '';
        }
    });

    terminalInput.addEventListener('keydown', (e) => {
        if (e.key === 'ArrowUp') {
            e.preventDefault();
            if (historyIndex > 0) {
                historyIndex--;
                terminalInput.value = commandHistory[historyIndex];
            }
        } else if (e.key === 'ArrowDown') {
            e.preventDefault();
            if (historyIndex < commandHistory.length - 1) {
                historyIndex++;
                terminalInput.value = commandHistory[historyIndex];
            } else {
                historyIndex = commandHistory.length;
                terminalInput.value = '';
            }
        }
    });

    btnClearTerm.addEventListener('click', () => {
        terminalOutput.innerHTML = '';
    });

    // Auto-start in Simulator mode if opened for quick preview
    logTerminal('[SYSTEM] Ready. Click "Simulator Mode" to test without hardware, or "Connect Serial" for physical Arduino.', 'sys');
});

