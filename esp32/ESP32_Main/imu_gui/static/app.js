/**
 * BNO08X IMU Real-Time 3D Attitude Visualizer & Control Engine
 */

let ws = null;
let isPaused = false;
let tareYaw = 0;
let lastPacketTime = 0;
let packetTimes = [];

// Live Telemetry Cache
let telem = {
    roll: 0, pitch: 0, yaw: 0,
    qw: 1, qx: 0, qy: 0, qz: 0,
    ax: 0, ay: 0, az: 0,
    gx: 0, gy: 0, gz: 0
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
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
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

    // 3. Direction Arrow (Forward / Pitch Indicator)
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

    // 5. Axes Helper
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

function animateThree() {
    requestAnimationFrame(animateThree);

    if (imuMesh && !isPaused) {
        // Apply quaternion directly from BNO08x (converting ENU to Three.js coordinate system)
        // Three.js: X=Right, Y=Up, Z=Back
        // BNO08x Quat: qw, qx, qy, qz
        const q = new THREE.Quaternion(telem.qx, telem.qz, -telem.qy, telem.qw);
        
        // Apply Tare Offset on Yaw
        if (tareYaw !== 0) {
            const tareQuat = new THREE.Quaternion();
            tareQuat.setFromAxisAngle(new THREE.Vector3(0, 1, 0), -tareYaw * (Math.PI / 180));
            q.premultiply(tareQuat);
        }

        imuMesh.quaternion.slerp(q, 0.5);
    }

    renderer.render(scene, camera);
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
        const interval = lastPacketTime > 0 ? Math.round(now - lastPacketTime) : 10;
        lastPacketTime = now;

        // Accurate rolling 1-second packet counter
        packetTimes.push(now);
        while (packetTimes.length > 0 && packetTimes[0] < now - 1000) {
            packetTimes.shift();
        }
        
        const liveHz = packetTimes.length;
        document.getElementById('val-hz').innerHTML = `${liveHz} <small>Hz</small>`;

        try {
            const data = JSON.parse(event.data);
            handleTelemetry(data, interval);
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

function handleTelemetry(data, latency) {
    telem.roll = data.roll || 0;
    telem.pitch = data.pitch || 0;
    telem.yaw = data.yaw || 0;

    telem.qw = data.qw !== undefined ? data.qw : 1.0;
    telem.qx = data.qx || 0;
    telem.qy = data.qy || 0;
    telem.qz = data.qz || 0;

    telem.ax = data.ax || 0;
    telem.ay = data.ay || 0;
    telem.az = data.az || 0;

    telem.gx = data.gx || 0;
    telem.gy = data.gy || 0;
    telem.gz = data.gz || 0;

    // Adjusted Yaw with Tare
    let adjYaw = telem.yaw - tareYaw;
    while (adjYaw > 180) adjYaw -= 360;
    while (adjYaw < -180) adjYaw += 360;

    // Update Numerical DOM Values
    document.getElementById('val-latency').innerHTML = `${Math.min(latency, 999)} <small>ms</small>`;
    document.getElementById('val-roll').textContent = `${telem.roll >= 0 ? '+' : ''}${telem.roll.toFixed(2)}°`;
    document.getElementById('val-pitch').textContent = `${telem.pitch >= 0 ? '+' : ''}${telem.pitch.toFixed(2)}°`;
    document.getElementById('val-yaw').textContent = `${adjYaw >= 0 ? '+' : ''}${adjYaw.toFixed(2)}°`;

    document.getElementById('badge-roll').textContent = `R: ${telem.roll.toFixed(1)}°`;
    document.getElementById('badge-pitch').textContent = `P: ${telem.pitch.toFixed(1)}°`;
    document.getElementById('badge-yaw').textContent = `Y: ${adjYaw.toFixed(1)}°`;

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

    // Update Artificial Horizon
    const horizonSky = document.getElementById('horizon-sky');
    if (horizonSky) {
        const pitchClamped = Math.max(-45, Math.min(45, telem.pitch));
        horizonSky.style.transform = `rotate(${-telem.roll}deg) translateY(${pitchClamped * 0.8}px)`;
    }

    // Update Compass Dial
    const compassDial = document.getElementById('compass-dial');
    const compassDeg = document.getElementById('compass-deg');
    if (compassDial) {
        compassDial.style.transform = `rotate(${-adjYaw}deg)`;
        const heading360 = (adjYaw < 0 ? adjYaw + 360 : adjYaw).toFixed(0);
        compassDeg.textContent = `${String(heading360).padStart(3, '0')}°`;
    }

    // Update G-Force Meter
    const gDot = document.getElementById('g-dot');
    const gTotal = document.getElementById('g-total');
    if (gDot) {
        const gx = Math.max(-20, Math.min(20, telem.ax * 2));
        const gy = Math.max(-20, Math.min(20, telem.ay * 2));
        gDot.style.transform = `translate(calc(-50% + ${gx}px), calc(-50% + ${gy}px))`;
        const totalG = (Math.sqrt(telem.ax*telem.ax + telem.ay*telem.ay + telem.az*telem.az) / 9.81).toFixed(2);
        gTotal.textContent = `${totalG} G`;
    }

    // Append to Waveform History
    historyRoll.push(telem.roll);
    historyRoll.shift();
    historyPitch.push(telem.pitch);
    historyPitch.shift();
    historyYaw.push(adjYaw);
    historyYaw.shift();
}

// ==========================================
// 3. REAL-TIME WAVEFORM CANVAS RENDERER
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
                // Scale: -180° to +180° mapped to height
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
// 4. UI CONTROLS & SETTINGS EVENT HANDLERS
// ==========================================
function initControls() {
    // Tare / Zero Heading
    document.getElementById('btn-tare').addEventListener('click', () => {
        tareYaw = telem.yaw;
    });

    // Reset View
    document.getElementById('btn-reset-view').addEventListener('click', () => {
        camera.position.set(0, 3.5, 6.5);
        camera.lookAt(0, 0, 0);
    });

    // Pause / Resume
    const btnPause = document.getElementById('btn-pause');
    const pauseIcon = document.getElementById('pause-icon');
    const pauseText = document.getElementById('pause-text');
    btnPause.addEventListener('click', () => {
        isPaused = !isPaused;
        pauseIcon.textContent = isPaused ? '▶️' : '⏸️';
        pauseText.textContent = isPaused ? 'Resume Stream' : 'Pause Stream';
        btnPause.className = isPaused ? 'btn btn-primary' : 'btn btn-secondary';
    });

    // Hard Reset Sensor
    document.getElementById('btn-hard-reset').addEventListener('click', async () => {
        if (confirm("Send hardware reset signal to BNO08X?")) {
            try {
                await fetch('/api/reset', { method: 'POST' });
            } catch (e) {
                console.error("Reset request failed:", e);
            }
        }
    });

    // Rate Slider & Presets
    const rateSlider = document.getElementById('rate-slider');
    const rateDisplay = document.getElementById('rate-display');
    const presetBtns = document.querySelectorAll('.btn-preset');

    async function setRate(hz) {
        rateSlider.value = hz;
        rateDisplay.textContent = `${hz} Hz`;
        presetBtns.forEach(btn => {
            btn.classList.toggle('active', parseInt(btn.dataset.rate) === parseInt(hz));
        });

        try {
            await fetch(`/api/set_rate?hz=${hz}`, { method: 'POST' });
        } catch (e) {
            console.error("Rate update failed:", e);
        }
    }

    rateSlider.addEventListener('input', (e) => {
        rateDisplay.textContent = `${e.target.value} Hz`;
    });

    rateSlider.addEventListener('change', (e) => {
        setRate(e.target.value);
    });

    presetBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            setRate(btn.dataset.rate);
        });
    });

    // Sensor Toggles
    const toggles = [
        { id: 'toggle-game-rot', feature: 'game_rot' },
        { id: 'toggle-accel', feature: 'accel' },
        { id: 'toggle-gyro', feature: 'gyro' },
        { id: 'toggle-rot-vector', feature: 'rot_vector' }
    ];

    toggles.forEach(t => {
        const el = document.getElementById(t.id);
        if (el) {
            el.addEventListener('change', async () => {
                try {
                    await fetch(`/api/toggle_feature?feature=${t.feature}&enabled=${el.checked}`, { method: 'POST' });
                } catch (e) {
                    console.error("Toggle feature failed:", e);
                }
            });
        }
    });
}

// Initialize on DOM Ready
window.addEventListener('DOMContentLoaded', () => {
    initThree();
    initWaveform();
    initControls();
    connectWebSocket();
});
