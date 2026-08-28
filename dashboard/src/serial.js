/**
 * serial.js
 * 
 * Handles Web Serial API communications with Arduino Uno / STP-DRV-6575 controller.
 * Also includes a full-fidelity Virtual Controller Simulator for offline testing.
 */

class SerialController {
    constructor() {
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.readableStreamClosed = null;
        this.writableStreamClosed = null;
        this.isConnected = false;
        this.isMockMode = false;

        // Callbacks
        this.onConnect = () => {};
        this.onDisconnect = () => {};
        this.onData = (line) => {};
        this.onStatus = (status) => {};
        this.onError = (err) => {};

        // Simulator instance
        this.mockSimulator = new MockControllerSimulator(this);
    }

    isWebSerialSupported() {
        return 'serial' in navigator;
    }

    async connect(baudRate = 115200) {
        if (!this.isWebSerialSupported()) {
            throw new Error("Web Serial API is not supported in this browser. Please use Google Chrome, Microsoft Edge, or Opera.");
        }

        try {
            this.port = await navigator.serial.requestPort();
            await this.port.open({ baudRate: parseInt(baudRate, 10) });

            const textDecoder = new TextDecoderStream();
            this.readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
            this.reader = textDecoder.readable.getReader();

            const textEncoder = new TextEncoderStream();
            this.writableStreamClosed = textEncoder.readable.pipeTo(this.port.writable);
            this.writer = textEncoder.writable.getWriter();

            this.isConnected = true;
            this.isMockMode = false;
            this.onConnect({ port: this.port, mock: false });

            this.readLoop();
        } catch (err) {
            this.onError(err);
            throw err;
        }
    }

    async readLoop() {
        let buffer = '';
        try {
            while (this.isConnected && this.reader) {
                const { value, done } = await this.reader.read();
                if (done) break;
                if (value) {
                    buffer += value;
                    const lines = buffer.split(/\r?\n/);
                    buffer = lines.pop(); // Keep partial line in buffer

                    for (const line of lines) {
                        const trimmed = line.trim();
                        if (trimmed.length > 0) {
                            this.handleIncomingLine(trimmed);
                        }
                    }
                }
            }
        } catch (err) {
            if (this.isConnected) {
                this.onError(err);
                this.disconnect();
            }
        }
    }

    handleIncomingLine(line) {
        this.onData(line);

        // Check if telemetry packet
        if (line.startsWith("STATUS ")) {
            const status = this.parseStatusPacket(line);
            if (status) {
                this.onStatus(status);
            }
        }
    }

    parseStatusPacket(line) {
        // Line format: STATUS P=0.00 T=0.00 TP=0.00 TT=0.00 SP=0.0 ST=0.0 MV=0 EN=1 LP=0 LT=0
        try {
            const tokens = line.substring(7).trim().split(/\s+/);
            const status = {
                pan: 0,
                tilt: 0,
                targetPan: 0,
                targetTilt: 0,
                speedPan: 0,
                speedTilt: 0,
                moving: false,
                enabled: true,
                limitPan: false,
                limitTilt: false,
                timestamp: Date.now()
            };

            for (const token of tokens) {
                const [key, val] = token.split('=');
                if (!key || val === undefined) continue;

                switch (key.toUpperCase()) {
                    case 'P':  status.pan = parseFloat(val); break;
                    case 'T':  status.tilt = parseFloat(val); break;
                    case 'TP': status.targetPan = parseFloat(val); break;
                    case 'TT': status.targetTilt = parseFloat(val); break;
                    case 'SP': status.speedPan = parseFloat(val); break;
                    case 'ST': status.speedTilt = parseFloat(val); break;
                    case 'MV': status.moving = (val === '1' || val === 'true'); break;
                    case 'EN': status.enabled = (val === '1' || val === 'true'); break;
                    case 'LP': status.limitPan = (val === '1'); break;
                    case 'LT': status.limitTilt = (val === '1'); break;
                }
            }
            return status;
        } catch (e) {
            console.error("Failed to parse status packet:", e);
            return null;
        }
    }

    async sendCommand(cmd) {
        const cleanCmd = cmd.trim();
        if (!cleanCmd) return;

        if (this.isMockMode) {
            this.mockSimulator.processCommand(cleanCmd);
            return;
        }

        if (!this.isConnected || !this.writer) {
            throw new Error("Serial port not connected");
        }

        await this.writer.write(cleanCmd + "\n");
    }

    async disconnect() {
        if (this.isMockMode) {
            this.mockSimulator.stop();
            this.isMockMode = false;
            this.isConnected = false;
            this.onDisconnect();
            return;
        }

        this.isConnected = false;
        try {
            if (this.reader) {
                await this.reader.cancel();
                await this.readableStreamClosed.catch(() => {});
                this.reader = null;
            }
            if (this.writer) {
                await this.writer.close();
                await this.writableStreamClosed.catch(() => {});
                this.writer = null;
            }
            if (this.port) {
                await this.port.close();
                this.port = null;
            }
        } catch (e) {
            console.warn("Error during serial disconnect:", e);
        } finally {
            this.onDisconnect();
        }
    }

    startSimulator() {
        if (this.isConnected) {
            this.disconnect();
        }
        this.isMockMode = true;
        this.isConnected = true;
        this.mockSimulator.start();
        this.onConnect({ mock: true });
    }
}

/**
 * High-fidelity virtual controller simulator for browser testing.
 */
class MockControllerSimulator {
    constructor(parent) {
        this.parent = parent;
        this.timer = null;

        // Virtual State
        this.pan = 0.0;
        this.tilt = 0.0;
        this.targetPan = 0.0;
        this.targetTilt = 0.0;
        this.speedPan = 0.0;
        this.speedTilt = 0.0;
        this.maxSpeedPan = 45.0;
        this.maxSpeedTilt = 30.0;
        this.accelPan = 90.0;
        this.accelTilt = 60.0;

        this.jogPan = 0.0;
        this.jogTilt = 0.0;
        this.mode = 'IDLE'; // 'IDLE', 'POSITION', 'JOG'
        this.enabled = true;

        this.softLimits = true;
        this.panMin = -180.0;
        this.panMax = 180.0;
        this.tiltMin = -45.0;
        this.tiltMax = 90.0;

        this.lastTime = Date.now();
    }

    start() {
        this.stop();
        this.lastTime = Date.now();
        this.parent.onData("[SIM] Virtual Arduino Uno connected with STP-DRV-6575 drives.");
        this.parent.onData("=== Pan-Tilt Controller Initialized ===");
        this.parent.onData("Pan steps/deg: 4.444 | Tilt steps/deg: 4.444");

        this.timer = setInterval(() => this.updateLoop(), 50); // 20 Hz update
    }

    stop() {
        if (this.timer) {
            clearInterval(this.timer);
            this.timer = null;
        }
    }

    updateLoop() {
        const now = Date.now();
        const dt = (now - this.lastTime) / 1000.0;
        this.lastTime = now;

        if (this.enabled) {
            if (this.mode === 'POSITION') {
                const dPan = this.targetPan - this.pan;
                const dTilt = this.targetTilt - this.tilt;

                if (Math.abs(dPan) > 0.05) {
                    const stepP = Math.sign(dPan) * Math.min(Math.abs(dPan), this.maxSpeedPan * dt);
                    this.pan += stepP;
                    this.speedPan = Math.sign(dPan) * this.maxSpeedPan;
                } else {
                    this.pan = this.targetPan;
                    this.speedPan = 0;
                }

                if (Math.abs(dTilt) > 0.05) {
                    const stepT = Math.sign(dTilt) * Math.min(Math.abs(dTilt), this.maxSpeedTilt * dt);
                    this.tilt += stepT;
                    this.speedTilt = Math.sign(dTilt) * this.maxSpeedTilt;
                } else {
                    this.tilt = this.targetTilt;
                    this.speedTilt = 0;
                }

                if (Math.abs(dPan) <= 0.05 && Math.abs(dTilt) <= 0.05) {
                    this.mode = 'IDLE';
                }
            } else if (this.mode === 'JOG') {
                this.pan += this.jogPan * dt;
                this.tilt += this.jogTilt * dt;
                this.speedPan = this.jogPan;
                this.speedTilt = this.jogTilt;

                if (this.softLimits) {
                    this.pan = Math.max(this.panMin, Math.min(this.panMax, this.pan));
                    this.tilt = Math.max(this.tiltMin, Math.min(this.tiltMax, this.tilt));
                }

                this.targetPan = this.pan;
                this.targetTilt = this.tilt;

                if (Math.abs(this.jogPan) < 0.01 && Math.abs(this.jogTilt) < 0.01) {
                    this.mode = 'IDLE';
                }
            } else {
                this.speedPan = 0;
                this.speedTilt = 0;
            }
        }

        // Emit telemetry packet
        const isMoving = (this.mode !== 'IDLE');
        const line = `STATUS P=${this.pan.toFixed(2)} T=${this.tilt.toFixed(2)} TP=${this.targetPan.toFixed(2)} TT=${this.targetTilt.toFixed(2)} SP=${this.speedPan.toFixed(1)} ST=${this.speedTilt.toFixed(1)} MV=${isMoving ? 1 : 0} EN=${this.enabled ? 1 : 0} LP=0 LT=0`;
        this.parent.handleIncomingLine(line);
    }

    processCommand(cmd) {
        this.parent.onData(`> ${cmd}`);
        const parts = cmd.trim().split(/\s+/);
        const verb = parts[0].toUpperCase();

        if (verb === 'PING') {
            this.parent.onData("PONG");
        } else if (verb === 'MOVE' || verb === 'GOTO') {
            let p = parseFloat(parts[1]);
            let t = parseFloat(parts[2]);
            if (this.softLimits) {
                p = Math.max(this.panMin, Math.min(this.panMax, p));
                t = Math.max(this.tiltMin, Math.min(this.tiltMax, t));
            }
            this.targetPan = p;
            this.targetTilt = t;
            this.mode = 'POSITION';
            this.parent.onData("OK");
        } else if (verb === 'MOVEREL') {
            const dp = parseFloat(parts[1]) || 0;
            const dt = parseFloat(parts[2]) || 0;
            let targetP = this.pan + dp;
            let targetT = this.tilt + dt;
            if (this.softLimits) {
                targetP = Math.max(this.panMin, Math.min(this.panMax, targetP));
                targetT = Math.max(this.tiltMin, Math.min(this.tiltMax, targetT));
            }
            this.targetPan = targetP;
            this.targetTilt = targetT;
            this.mode = 'POSITION';
            this.parent.onData("OK");
        } else if (verb === 'JOG') {
            this.jogPan = parseFloat(parts[1]) || 0;
            this.jogTilt = parseFloat(parts[2]) || 0;
            this.mode = (Math.abs(this.jogPan) > 0.01 || Math.abs(this.jogTilt) > 0.01) ? 'JOG' : 'IDLE';
            this.parent.onData("OK");
        } else if (verb === 'STOP') {
            this.mode = 'IDLE';
            this.jogPan = 0;
            this.jogTilt = 0;
            this.targetPan = this.pan;
            this.targetTilt = this.tilt;
            this.parent.onData("OK");
        } else if (verb === 'ESTOP') {
            this.mode = 'IDLE';
            this.enabled = false;
            this.parent.onData("OK ESTOP_ACTIVE");
        } else if (verb === 'ENABLE') {
            this.enabled = true;
            this.parent.onData("OK");
        } else if (verb === 'DISABLE') {
            this.enabled = false;
            this.mode = 'IDLE';
            this.parent.onData("OK");
        } else if (verb === 'ZERO') {
            this.pan = 0;
            this.tilt = 0;
            this.targetPan = 0;
            this.targetTilt = 0;
            this.mode = 'IDLE';
            this.parent.onData("OK");
        } else if (verb === 'HOME') {
            this.pan = 0;
            this.tilt = 0;
            this.targetPan = 0;
            this.targetTilt = 0;
            this.mode = 'IDLE';
            this.parent.onData("OK HOMING_COMPLETE");
        } else if (verb === 'GET') {
            const sub = (parts[1] || '').toUpperCase();
            if (sub === 'POS') {
                this.parent.onData(`POS P=${this.pan.toFixed(2)} T=${this.tilt.toFixed(2)}`);
            } else {
                this.parent.onData(`STATUS P=${this.pan.toFixed(2)} T=${this.tilt.toFixed(2)} TP=${this.targetPan.toFixed(2)} TT=${this.targetTilt.toFixed(2)} SP=${this.speedPan.toFixed(1)} ST=${this.speedTilt.toFixed(1)} MV=${this.mode !== 'IDLE' ? 1 : 0} EN=${this.enabled ? 1 : 0} LP=0 LT=0`);
            }
        } else if (verb === 'SET') {
            this.parent.onData("OK");
        } else {
            this.parent.onData(`OK`);
        }
    }
}

window.serialController = new SerialController();

