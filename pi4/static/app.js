/**
 * BNO08X IMU Aerospace Telemetry Cockpit & ESP32-S3 Real-Time Control Engine
 * Supports Axis Inversion (Yaw/Pitch/Roll/Swap), Anti-Flicker Visual Hz Decoupling,
 * Three.js 3D Attitude Rendering, and Smoothed Telemetry Diagnostics.
 */

let ws = null;
let isPaused = false;
let tareYaw = 0;
let tarePitch = 0;
let tareRoll = 0;

// Axis Inversions & Swapping
let invYaw = false;
let invPitch = false;
let invRoll = false;
let swapRP = false;

// Telemetry & Rate Diagnostics
let lastPacketTime = 0;
let packetTimes = [];
let smoothedHz = 200.0;
let smoothedLatency = 1.0;
let lastDiagUpdateTime = 0;

// Anti-Flicker Visual Refresh Rate (Hz)
let visualHz = 20;
let lastUiRenderTime = 0;

// Live Telemetry Cache (Updated at Full Hardware Rate, e.g. 200Hz)
let telem = {
    roll: 0, pitch: 0, yaw: 0,
    qw: 1, qx: 0, qy: 0, qz: 0,
    ax: 0, ay: 0, az: 9.81,
    gx: 0, gy: 0, gz: 0,
    ts: 0
};

// Waveform History Buffer
const MAX_HISTORY = 120;
let historyRoll = new Array(MAX_HISTORY).fill(0);
let historyPitch = new Array(MAX_HISTORY).fill(0);
let historyYaw = new Array(MAX_HISTORY).fill(0);

// ==========================================
// 1. THREE.JS 3D ATTITUDE VIEWER
// ==========================================
let scene, camera, renderer, imuMesh, gridHelper, axesHelper;

function initThree() {
    const container = document.getElementById('canvas-3d');
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(0, 3.5, 6.5);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.75);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0x00f0ff, 1.2);
    dirLight.position.set(5, 10, 7);
    scene.add(dirLight);

    const backLight = new THREE.DirectionalLight(0x9d4edd, 0.8);
    backLight.position.set(-5, -5, -5);
    scene.add(backLight);

    // Grid Floor
    gridHelper = new THREE.GridHelper(10, 20, 0x00f0ff, 0x1f293d);
    gridHelper.position.y = -1.5;
    scene.add(gridHelper);

    // 3D IMU Board + Craft Object
    imuMesh = new THREE.Group();

    // 1. Circuit Board Base
    const boardGeo = new THREE.BoxGeometry(2.4, 0.12, 3.2);
    const boardMat = new THREE.MeshStandardMaterial({
        color: 0x0c2838,
        metalness: 0.6,
        roughness: 0.3
    });
    const board = new THREE.Mesh(boardGeo, boardMat);
    imuMesh.add(board);

    // 2. Chip (BNO080 IC)
    const chipGeo = new THREE.BoxGeometry(0.8, 0.08, 0.8);
    const chipMat = new THREE.MeshStandardMaterial({
        color: 0x181818,
        metalness: 0.8,
        roughness: 0.2
    });
    const chip = new THREE.Mesh(chipGeo, chipMat);
    chip.position.set(0, 0.09, 0.2);
    imuMesh.add(chip);

    // 3. Direction Arrow (Nose Cone)
    const coneGeo = new THREE.ConeGeometry(0.35, 0.8, 16);
    const coneMat = new THREE.MeshStandardMaterial({
        color: 0x00f0ff,
        emissive: 0x0070f3,
        emissiveIntensity: 0.6
    });
    const noseCone = new THREE.Mesh(coneGeo, coneMat);
    noseCone.rotation.x = -Math.PI / 2;
    noseCone.position.set(0, 0.15, -1.8);
    imuMesh.add(noseCone);

    // 4. Wings / Side Fins
    const wingGeo = new THREE.BoxGeometry(4.0, 0.04, 0.9);
    const wingMat = new THREE.MeshStandardMaterial({
        color: 0x00e5ff,
        transparent: true,
        opacity: 0.4
    });
    const wings = new THREE.Mesh(wingGeo, wingMat);
    wings.position.set(0, 0.05, 0.2);
    imuMesh.add(wings);

    // 5. Coordinate Axes
    axesHelper = new THREE.AxesHelper(1.8);
    imuMesh.add(axesHelper);

    scene.add(imuMesh);

    // Resize Handler
    window.addEventListener('resize', () => {
        const w = container.clientWidth;
        const h = container.clientHeight;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    });

    animateThree();
}

function getProcessedEuler() {
    let r = telem.roll;
    let p = telem.pitch;
    let y = telem.yaw;

    // Swap Roll and Pitch if enabled
    if (swapRP) {
        let temp = r;
        r = p;
        p = temp;
    }

    // Apply Inversions
    if (invRoll) r = -r;
    if (invPitch) p = -p;
    if (invYaw) y = -y;

    // Apply Tare
    y = y - tareYaw;
    while (y > 180) y -= 360;
    while (y < -180) y += 360;

    return { roll: r, pitch: p, yaw: y };
}

function animateThree() {
    requestAnimationFrame(animateThree);

    if (imuMesh && !isPaused) {
        const e = getProcessedEuler();
        
        // Convert Euler degrees to radians for Three.js (Order: Y-X-Z)
        const radR = e.roll * (Math.PI / 180);
        const radP = e.pitch * (Math.PI / 180);
        const radY = e.yaw * (Math.PI / 180);

        const euler = new THREE.Euler(radP, -radY, -radR, 'YXZ');
        const targetQuat = new THREE.Quaternion().setFromEuler(euler);

        imuMesh.quaternion.slerp(targetQuat, 0.4);
    }

    renderer.render(scene, camera);

    // Render decoupled visual UI elements based on visualHz
    const now = performance.now();
    if (now - lastUiRenderTime >= (1000.0 / visualHz)) {
        lastUiRenderTime = now;
        updateVisualUIDOM();
    }
}

// ==========================================
// 2. WEBSOCKET TELEMETRY CLIENT
// ==========================================
function connectWebSocket() {
    const connPill = document.getElementById('conn-status');
    const connText = document.getElementById('conn-text');
    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host || '10.43.8.42:8000';
    const wsUrl = `${proto}//${host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        connPill.className = 'metric-pill connection-pill connected';
        connText.textContent = 'ONLINE';
    };

    ws.onmessage = (event) => {
        if (isPaused) return;

        const now = performance.now();
        const rawLatency = lastPacketTime > 0 ? (now - lastPacketTime) : 5.0;
        lastPacketTime = now;

        // Accurate rolling 1-second packet counter for stream rate
        packetTimes.push(now);
        while (packetTimes.length > 0 && packetTimes[0] < now - 1000) {
            packetTimes.shift();
        }

        try {
            const data = JSON.parse(event.data);
            
            // Cache full 200 Hz telemetry instantly in memory
            telem.roll = data.roll || 0;
            telem.pitch = data.pitch || 0;
            telem.yaw = data.yaw || 0;

            telem.qw = data.qw !== undefined ? data.qw : 1.0;
            telem.qx = data.qx || 0;
            telem.qy = data.qy || 0;
            telem.qz = data.qz || 0;

            telem.ax = data.ax || 0;
            telem.ay = data.ay || 0;
            telem.az = data.az !== undefined ? data.az : 9.81;

            telem.gx = data.gx || 0;
            telem.gy = data.gy || 0;
            telem.gz = data.gz || 0;
            telem.ts = data.ts || 0;

            // Push to background waveform buffer at full stream speed
            const e = getProcessedEuler();
            historyRoll.push(e.roll);
            historyRoll.shift();
            historyPitch.push(e.pitch);
            historyPitch.shift();
            historyYaw.push(e.yaw);
            historyYaw.shift();

            // Smooth diagnostics (EMA low-pass filter)
            const liveHz = packetTimes.length;
            smoothedHz = (smoothedHz * 0.85) + (liveHz * 0.15);
            smoothedLatency = (smoothedLatency * 0.85) + (rawLatency * 0.15);

        } catch (e) {
            console.error("Packet parse error:", e);
        }
    };

    ws.onclose = () => {
        connPill.className = 'metric-pill connection-pill disconnected';
        connText.textContent = 'RECONNECTING';
        setTimeout(connectWebSocket, 1500);
    };

    ws.onerror = (err) => {
        ws.close();
    };
}

function sendWsCommand(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(obj));
    }
}

// ==========================================
// 3. DECOUPLED ANTI-FLICKER UI UPDATER
// ==========================================
function updateVisualUIDOM() {
    const e = getProcessedEuler();

    // 1. Update Smoothed Top Nav Metrics
    document.getElementById('val-hz').innerHTML = `${Math.round(smoothedHz)} <small>Hz</small>`;
    document.getElementById('val-latency').innerHTML = `${Math.max(1, Math.round(smoothedLatency))} <small>ms</small>`;

    // 2. Numerical Euler Display
    document.getElementById('val-roll').textContent = `${e.roll >= 0 ? '+' : ''}${e.roll.toFixed(2)}°`;
    document.getElementById('val-pitch').textContent = `${e.pitch >= 0 ? '+' : ''}${e.pitch.toFixed(2)}°`;
    document.getElementById('val-yaw').textContent = `${e.yaw >= 0 ? '+' : ''}${e.yaw.toFixed(2)}°`;

    document.getElementById('badge-roll').textContent = `R: ${e.roll >= 0 ? '+' : ''}${e.roll.toFixed(1)}°`;
    document.getElementById('badge-pitch').textContent = `P: ${e.pitch >= 0 ? '+' : ''}${e.pitch.toFixed(1)}°`;
    document.getElementById('badge-yaw').textContent = `Y: ${e.yaw >= 0 ? '+' : ''}${e.yaw.toFixed(1)}°`;

    // 3. Numerical Accel & Gyro & Quat Display
    document.getElementById('val-qw').textContent = `${telem.qw >= 0 ? '+' : ''}${telem.qw.toFixed(3)}`;
    document.getElementById('val-qx').textContent = `${telem.qx >= 0 ? '+' : ''}${telem.qx.toFixed(3)}`;
    document.getElementById('val-qy').textContent = `${telem.qy >= 0 ? '+' : ''}${telem.qy.toFixed(3)}`;
    document.getElementById('val-qz').textContent = `${telem.qz >= 0 ? '+' : ''}${telem.qz.toFixed(3)}`;

    document.getElementById('val-ax').textContent = `${telem.ax >= 0 ? '+' : ''}${telem.ax.toFixed(2)}`;
    document.getElementById('val-ay').textContent = `${telem.ay >= 0 ? '+' : ''}${telem.ay.toFixed(2)}`;
    document.getElementById('val-az').textContent = `${telem.az >= 0 ? '+' : ''}${telem.az.toFixed(2)}`;

    document.getElementById('val-gx').textContent = `${telem.gx >= 0 ? '+' : ''}${telem.gx.toFixed(1)}`;
    document.getElementById('val-gy').textContent = `${telem.gy >= 0 ? '+' : ''}${telem.gy.toFixed(1)}`;
    document.getElementById('val-gz').textContent = `${telem.gz >= 0 ? '+' : ''}${telem.gz.toFixed(1)}`;

    // 4. Update Artificial Horizon Instrument
    const horizonSky = document.getElementById('horizon-sky');
    if (horizonSky) {
        const pitchClamped = Math.max(-45, Math.min(45, e.pitch));
        horizonSky.style.transform = `rotate(${-e.roll}deg) translateY(${pitchClamped * 0.8}px)`;
    }

    // 5. Update Compass Heading Instrument
    const compassDial = document.getElementById('compass-dial');
    const compassDeg = document.getElementById('compass-deg');
    if (compassDial) {
        compassDial.style.transform = `rotate(${-e.yaw}deg)`;
        const heading360 = (e.yaw < 0 ? e.yaw + 360 : e.yaw).toFixed(0);
        compassDeg.textContent = `${String(heading360).padStart(3, '0')}°`;
    }

    // 6. Update G-Force Meter
    const gDot = document.getElementById('g-dot');
    const gTotal = document.getElementById('g-total');
    if (gDot) {
        const gx = Math.max(-20, Math.min(20, telem.ax * 2));
        const gy = Math.max(-20, Math.min(20, telem.ay * 2));
        gDot.style.transform = `translate(calc(-50% + ${gx}px), calc(-50% + ${gy}px))`;
        const totalG = (Math.sqrt(telem.ax*telem.ax + telem.ay*telem.ay + telem.az*telem.az) / 9.81).toFixed(2);
        gTotal.textContent = `${totalG} G`;
    }
}

// ==========================================
// 4. REAL-TIME WAVEFORM CANVAS RENDERER
// ==========================================
let canvas, ctx;

function initWaveform() {
    canvas = document.getElementById('waveform-canvas');
    if (!canvas) return;
    ctx = canvas.getContext('2d');

    function resizeCanvas() {
        canvas.width = canvas.parentElement.clientWidth;
        canvas.height = canvas.parentElement.clientHeight;
    }
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    function drawWaveform() {
        requestAnimationFrame(drawWaveform);
        if (!ctx) return;

        const w = canvas.width;
        const h = canvas.height;
        const midY = h / 2;

        ctx.clearRect(0, 0, w, h);

        // Center zero line
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(0, midY);
        ctx.lineTo(w, midY);
        ctx.stroke();
        ctx.setLineDash([]);

        function drawLine(data, color) {
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            const step = w / (MAX_HISTORY - 1);
            for (let i = 0; i < MAX_HISTORY; i++) {
                const y = midY - (data[i] / 180.0) * (midY - 10);
                if (i === 0) ctx.moveTo(0, y);
                else ctx.lineTo(i * step, y);
            }
            ctx.stroke();
        }

        // Draw Roll (Magenta), Pitch (Emerald), Yaw (Cyan)
        drawLine(historyRoll, '#ff007a');
        drawLine(historyPitch, '#00ff88');
        drawLine(historyYaw, '#00f0ff');
    }

    drawWaveform();
}

// ==========================================
// 5. UI CONTROLS & EVENT HANDLERS
// ==========================================
function initControls() {
    // 1. Hardware Tare
    async function triggerTare() {
        tareYaw = (invYaw ? -telem.yaw : telem.yaw);
        sendWsCommand({ action: "tare" });
        try {
            await fetch('/api/tare', { method: 'POST' });
        } catch (e) {
            console.error("Tare failed:", e);
        }
    }

    const btnTare = document.getElementById('btn-tare');
    const btnTareCmd = document.getElementById('btn-tare-cmd');
    if (btnTare) btnTare.addEventListener('click', triggerTare);
    if (btnTareCmd) btnTareCmd.addEventListener('click', triggerTare);

    // 2. Axis Inversions & Swapping
    const btnInvYaw = document.getElementById('btn-inv-yaw');
    const btnInvPitch = document.getElementById('btn-inv-pitch');
    const btnInvRoll = document.getElementById('btn-inv-roll');
    const btnSwapRP = document.getElementById('btn-swap-rp');
    const btnResetInv = document.getElementById('btn-reset-inversion');

    if (btnInvYaw) {
        btnInvYaw.addEventListener('click', () => {
            invYaw = !invYaw;
            btnInvYaw.classList.toggle('active', invYaw);
        });
    }

    if (btnInvPitch) {
        btnInvPitch.addEventListener('click', () => {
            invPitch = !invPitch;
            btnInvPitch.classList.toggle('active', invPitch);
        });
    }

    if (btnInvRoll) {
        btnInvRoll.addEventListener('click', () => {
            invRoll = !invRoll;
            btnInvRoll.classList.toggle('active', invRoll);
        });
    }

    if (btnSwapRP) {
        btnSwapRP.addEventListener('click', () => {
            swapRP = !swapRP;
            btnSwapRP.classList.toggle('active', swapRP);
        });
    }

    if (btnResetInv) {
        btnResetInv.addEventListener('click', () => {
            invYaw = false;
            invPitch = false;
            invRoll = false;
            swapRP = false;
            if (btnInvYaw) btnInvYaw.classList.remove('active');
            if (btnInvPitch) btnInvPitch.classList.remove('active');
            if (btnInvRoll) btnInvRoll.classList.remove('active');
            if (btnSwapRP) btnSwapRP.classList.remove('active');
        });
    }

    // 3. Visual Refresh Rate Slider & Presets
    const visualSlider = document.getElementById('visual-hz-slider');
    const visualDisplay = document.getElementById('visual-hz-display');
    const visualPresets = document.querySelectorAll('.btn-visual-preset');

    function setVisualHz(hz) {
        visualHz = parseInt(hz);
        if (visualSlider) visualSlider.value = visualHz;
        if (visualDisplay) visualDisplay.textContent = `${visualHz} Hz`;
        visualPresets.forEach(btn => {
            btn.classList.toggle('active', parseInt(btn.dataset.vhz) === visualHz);
        });
    }

    if (visualSlider) {
        visualSlider.addEventListener('input', (e) => {
            setVisualHz(e.target.value);
        });
    }

    visualPresets.forEach(btn => {
        btn.addEventListener('click', () => {
            setVisualHz(btn.dataset.vhz);
        });
    });

    // 4. Reset Camera View
    document.getElementById('btn-reset-view').addEventListener('click', () => {
        camera.position.set(0, 3.5, 6.5);
        camera.lookAt(0, 0, 0);
    });

    // 5. Pause / Resume GUI
    const btnPause = document.getElementById('btn-pause');
    const pauseIcon = document.getElementById('pause-icon');
    const pauseText = document.getElementById('pause-text');
    btnPause.addEventListener('click', () => {
        isPaused = !isPaused;
        pauseIcon.textContent = isPaused ? '▶️' : '⏸️';
        pauseText.textContent = isPaused ? 'Resume GUI' : 'Pause GUI';
        btnPause.className = isPaused ? 'btn btn-primary' : 'btn btn-secondary';
    });

    // 6. Soft Reset IMU on ESP32
    document.getElementById('btn-soft-reset').addEventListener('click', async () => {
        if (confirm("Send soft reset command to BNO08X IMU on ESP32?")) {
            sendWsCommand({ action: "reset" });
            try {
                await fetch('/api/reset', { method: 'POST' });
            } catch (e) {
                console.error("Reset request failed:", e);
            }
        }
    });

    // 7. Hardware Rate Presets (50Hz, 100Hz, 200Hz)
    const rateDisplay = document.getElementById('rate-display');
    const presetBtns = document.querySelectorAll('.btn-preset');

    async function setRate(hz, cmd) {
        rateDisplay.textContent = `${hz} Hz`;
        presetBtns.forEach(btn => {
            btn.classList.toggle('active', parseInt(btn.dataset.rate) === parseInt(hz));
        });

        sendWsCommand({ action: "set_rate", hz: hz, cmd: cmd });
        try {
            await fetch(`/api/set_rate?hz=${hz}`, { method: 'POST' });
        } catch (e) {
            console.error("Rate update failed:", e);
        }
    }

    presetBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            setRate(btn.dataset.rate, btn.dataset.cmd);
        });
    });

    // 8. Sensor Fusion Mode Selector (1: 6-DOF, 2: 9-DOF, 3: Filtered, 5: All)
    const modeBtns = document.querySelectorAll('.btn-mode');
    async function setMode(mode) {
        modeBtns.forEach(btn => {
            btn.classList.toggle('active', btn.dataset.mode === mode);
        });

        sendWsCommand({ action: "set_mode", mode: mode });
        try {
            await fetch(`/api/set_mode?mode=${mode}`, { method: 'POST' });
        } catch (e) {
            console.error("Mode update failed:", e);
        }
    }

    modeBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            setMode(btn.dataset.mode);
        });
    });

    // 9. Copy Telemetry Snapshot to Clipboard
    const btnCopy = document.getElementById('btn-copy-telem');
    if (btnCopy) {
        btnCopy.addEventListener('click', () => {
            const e = getProcessedEuler();
            const text = `📊 Live 9-DOF Measurements
EULER ANGLES
ROLL ${e.roll >= 0 ? '+' : ''}${e.roll.toFixed(2)}°
PITCH ${e.pitch >= 0 ? '+' : ''}${e.pitch.toFixed(2)}°
YAW ${e.yaw >= 0 ? '+' : ''}${e.yaw.toFixed(2)}°
ACCELEROMETER (m/s²)
ACC X ${telem.ax >= 0 ? '+' : ''}${telem.ax.toFixed(2)}
ACC Y ${telem.ay >= 0 ? '+' : ''}${telem.ay.toFixed(2)}
ACC Z ${telem.az >= 0 ? '+' : ''}${telem.az.toFixed(2)}
GYROSCOPE (°/s)
GYR X ${telem.gx >= 0 ? '+' : ''}${telem.gx.toFixed(1)}
GYR Y ${telem.gy >= 0 ? '+' : ''}${telem.gy.toFixed(1)}
GYR Z ${telem.gz >= 0 ? '+' : ''}${telem.gz.toFixed(1)}
UNIT QUATERNION
QW ${telem.qw >= 0 ? '+' : ''}${telem.qw.toFixed(3)}
QX ${telem.qx >= 0 ? '+' : ''}${telem.qx.toFixed(3)}
QY ${telem.qy >= 0 ? '+' : ''}${telem.qy.toFixed(3)}
QZ ${telem.qz >= 0 ? '+' : ''}${telem.qz.toFixed(3)}`;

            navigator.clipboard.writeText(text).then(() => {
                const orig = btnCopy.textContent;
                btnCopy.textContent = '✅ Copied!';
                btnCopy.style.background = '#00ff88';
                btnCopy.style.color = '#000';
                setTimeout(() => {
                    btnCopy.textContent = orig;
                    btnCopy.style.background = '';
                    btnCopy.style.color = '';
                }, 1500);
            }).catch(err => {
                console.error("Clipboard error:", err);
            });
        });
    }
}

// Initialize on DOM Ready
window.addEventListener('DOMContentLoaded', () => {
    initThree();
    initWaveform();
    initControls();
    connectWebSocket();
});
