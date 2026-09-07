/**
 * Real-Time 2D SLAM Canvas Engine for Google Cartographer
 * Ultra-fast Base64 binary decoding + live LiDAR sweep + Sensor Health Diagnostics
 */

let canvas, ctx;
let ws = null;

// Map & Pose State
let mapState = {
    width: 0,
    height: 0,
    resolution: 0.02,
    originX: 0,
    originY: 0,
    offscreenCanvas: null,
    frameCount: 0
};

let robotPose = {
    x: 0,
    y: 0,
    yaw: 0,
    path: []
};

let liveScanPoints = [];

// Viewport Transform (Pan & Zoom)
let view = {
    scale: 6.0, // Pixels per grid cell
    offsetX: 0,
    offsetY: 0,
    isDragging: false,
    dragStartX: 0,
    dragStartY: 0,
    autoCenter: true
};

function initCanvas() {
    canvas = document.getElementById('slam-canvas');
    ctx = canvas.getContext('2d');

    function resize() {
        canvas.width = canvas.parentElement.clientWidth;
        canvas.height = canvas.parentElement.clientHeight;
        if (view.offsetX === 0 && view.offsetY === 0) {
            view.offsetX = canvas.width / 2;
            view.offsetY = canvas.height / 2;
        }
    }
    resize();
    window.addEventListener('resize', resize);

    // Pan & Zoom Event Listeners
    canvas.addEventListener('mousedown', (e) => {
        view.isDragging = true;
        view.dragStartX = e.clientX - view.offsetX;
        view.dragStartY = e.clientY - view.offsetY;
        view.autoCenter = false;
    });

    window.addEventListener('mousemove', (e) => {
        if (view.isDragging) {
            view.offsetX = e.clientX - view.dragStartX;
            view.offsetY = e.clientY - view.dragStartY;
        }
    });

    window.addEventListener('mouseup', () => {
        view.isDragging = false;
    });

    canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
        const mouseX = e.clientX - canvas.getBoundingClientRect().left;
        const mouseY = e.clientY - canvas.getBoundingClientRect().top;

        view.offsetX = mouseX - (mouseX - view.offsetX) * zoomFactor;
        view.offsetY = mouseY - (mouseY - view.offsetY) * zoomFactor;
        view.scale *= zoomFactor;
        view.scale = Math.max(0.5, Math.min(80.0, view.scale));
    });

    // Control Toolbar Buttons
    const debugPanel = document.getElementById('debug-panel');
    const toggleDebugBtn = document.getElementById('btn-toggle-debug');

    toggleDebugBtn.addEventListener('click', () => {
        debugPanel.classList.toggle('hidden');
        toggleDebugBtn.classList.toggle('active', !debugPanel.classList.contains('hidden'));
    });

    document.getElementById('btn-close-debug').addEventListener('click', () => {
        debugPanel.classList.add('hidden');
        toggleDebugBtn.classList.remove('active');
    });

    document.getElementById('btn-recenter').addEventListener('click', () => {
        view.autoCenter = true;
        centerOnRobot();
    });

    document.getElementById('btn-zoom-in').addEventListener('click', () => {
        view.scale *= 1.25;
    });

    document.getElementById('btn-zoom-out').addEventListener('click', () => {
        view.scale *= 0.8;
    });

    document.getElementById('btn-reset-view').addEventListener('click', () => {
        view.scale = 6.0;
        view.offsetX = canvas.width / 2;
        view.offsetY = canvas.height / 2;
        view.autoCenter = true;
    });

    document.getElementById('btn-save-map').addEventListener('click', () => {
        exportMapPNG();
    });

    document.getElementById('btn-reset-map').addEventListener('click', () => {
        if (confirm("Reset current SLAM map and start fresh?")) {
            resetMap();
        }
    });

    renderLoop();
}

function resetMap() {
    fetch('/api/reset_map', { method: 'POST' })
        .then(r => r.json())
        .then(data => {
            console.log("Reset result:", data);
        })
        .catch(err => {
            console.warn("Reset fetch error, sending over WS:", err);
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ action: 'reset_map' }));
            }
        });
    clearLocalMap();
}

function clearLocalMap() {
    mapState.width = 0;
    mapState.height = 0;
    mapState.offscreenCanvas = null;
    mapState.frameCount = 0;
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.yaw = 0;
    robotPose.path = [];
    liveScanPoints = [];
    document.getElementById('val-cells').textContent = '0';
    document.getElementById('val-size').textContent = '0.0m × 0.0m';
    document.getElementById('val-pose').textContent = 'X: 0.00m | Y: 0.00m | Yaw: 0.0°';
    if (canvas) {
        view.offsetX = canvas.width / 2;
        view.offsetY = canvas.height / 2;
    }
}

function centerOnRobot() {
    if (!mapState.width) return;
    const robotCellX = (robotPose.x - mapState.originX) / mapState.resolution;
    const robotCellY = (robotPose.y - mapState.originY) / mapState.resolution;

    view.offsetX = canvas.width / 2 - robotCellX * view.scale;
    view.offsetY = canvas.height / 2 + robotCellY * view.scale;
}

function updateOffscreenMapFromBase64(b64Data) {
    if (!mapState.width || !mapState.height) return;

    if (!mapState.offscreenCanvas) {
        mapState.offscreenCanvas = document.createElement('canvas');
    }
    mapState.offscreenCanvas.width = mapState.width;
    mapState.offscreenCanvas.height = mapState.height;

    const offCtx = mapState.offscreenCanvas.getContext('2d');
    const imgData = offCtx.createImageData(mapState.width, mapState.height);
    const data = imgData.data;

    const binaryString = atob(b64Data);
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
    }

    let totalExplored = 0;

    for (let y = 0; y < mapState.height; y++) {
        for (let x = 0; x < mapState.width; x++) {
            const gridIdx = (mapState.height - 1 - y) * mapState.width + x;
            const val = bytes[gridIdx];
            const pxIdx = (y * mapState.width + x) * 4;

            if (val === 255) {
                // Unknown space: deep slate
                data[pxIdx + 0] = 18;
                data[pxIdx + 1] = 24;
                data[pxIdx + 2] = 36;
                data[pxIdx + 3] = 255;
            } else if (val >= 60) {
                // Occupied Obstacle / Wall: bright neon cyan
                data[pxIdx + 0] = 0;
                data[pxIdx + 1] = 240;
                data[pxIdx + 2] = 255;
                data[pxIdx + 3] = 255;
                totalExplored++;
            } else {
                // Free Space: clean light silver
                data[pxIdx + 0] = 220;
                data[pxIdx + 1] = 230;
                data[pxIdx + 2] = 242;
                data[pxIdx + 3] = 255;
                totalExplored++;
            }
        }
    }

    offCtx.putImageData(imgData, 0, 0);

    mapState.frameCount++;
    document.getElementById('val-cells').textContent = totalExplored.toLocaleString();
    const sizeW = (mapState.width * mapState.resolution).toFixed(1);
    const sizeH = (mapState.height * mapState.resolution).toFixed(1);
    document.getElementById('val-size').textContent = `${sizeW}m × ${sizeH}m (F:${mapState.frameCount})`;
}

function updateTelemetryHUD(t) {
    if (!t) return;

    // 1. LiDAR Telemetry
    if (t.lidar) {
        const l = t.lidar;
        document.getElementById('lidar-hz').textContent = `${l.hz.toFixed(1)} Hz`;
        document.getElementById('lidar-pts').textContent = `${l.points} pts`;
        document.getElementById('lidar-min').textContent = `${l.min_dist.toFixed(2)} m`;
        document.getElementById('lidar-max').textContent = `${l.max_dist.toFixed(2)} m`;

        const badge = document.getElementById('lidar-status-badge');
        const card = document.getElementById('card-lidar');
        const hBar = document.getElementById('lidar-health-bar');
        const hText = document.getElementById('lidar-health-text');

        if (l.status === 'OK') {
            badge.className = 'sensor-badge';
            badge.textContent = 'ONLINE';
            card.className = 'sensor-card ok';
            hBar.className = 'health-indicator';
            hText.textContent = `Streaming 360° laser sweep (${l.hz.toFixed(1)} Hz)`;
        } else if (l.status === 'TIMEOUT') {
            badge.className = 'sensor-badge warn';
            badge.textContent = `TIMEOUT (${l.age_s}s)`;
            card.className = 'sensor-card error';
            hBar.className = 'health-indicator warn';
            hText.textContent = `No packets received for ${l.age_s}s`;
        } else {
            badge.className = 'sensor-badge err';
            badge.textContent = 'OFFLINE';
            card.className = 'sensor-card error';
            hBar.className = 'health-indicator err';
            hText.textContent = 'No serial stream detected on /dev/ttyAMA4';
        }
    }

    // 2. IMU Telemetry
    if (t.imu) {
        const im = t.imu;
        document.getElementById('imu-hz').textContent = `${im.hz.toFixed(1)} Hz`;
        document.getElementById('imu-rpy').textContent = `R: ${im.roll > 0 ? '+' : ''}${im.roll.toFixed(1)}° | P: ${im.pitch > 0 ? '+' : ''}${im.pitch.toFixed(1)}° | Y: ${im.yaw > 0 ? '+' : ''}${im.yaw.toFixed(1)}°`;
        document.getElementById('imu-accel').textContent = `X: ${im.ax.toFixed(2)} | Y: ${im.ay.toFixed(2)} | Z: ${im.az.toFixed(2)} m/s²`;
        document.getElementById('imu-gyro').textContent = `X: ${im.gx.toFixed(1)} | Y: ${im.gy.toFixed(1)} | Z: ${im.gz.toFixed(1)} °/s`;

        const badge = document.getElementById('imu-status-badge');
        const card = document.getElementById('card-imu');
        const hBar = document.getElementById('imu-health-bar');
        const hText = document.getElementById('imu-health-text');

        if (im.status === 'OK') {
            badge.className = 'sensor-badge';
            badge.textContent = 'ONLINE';
            card.className = 'sensor-card ok';
            hBar.className = 'health-indicator';
            hText.textContent = `Hardware fusion stream active (${im.hz.toFixed(0)} Hz)`;
        } else if (im.status === 'TIMEOUT') {
            badge.className = 'sensor-badge warn';
            badge.textContent = `TIMEOUT (${im.age_s}s)`;
            card.className = 'sensor-card error';
            hBar.className = 'health-indicator warn';
            hText.textContent = `No IMU packets received for ${im.age_s}s`;
        } else {
            badge.className = 'sensor-badge err';
            badge.textContent = 'OFFLINE';
            card.className = 'sensor-card error';
            hBar.className = 'health-indicator err';
            hText.textContent = 'I2C 0x4B not responding';
        }
    }
}

function renderLoop() {
    requestAnimationFrame(renderLoop);
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    ctx.translate(view.offsetX, view.offsetY);
    ctx.scale(view.scale, view.scale);

    // 1. Draw 2D Occupancy Grid Map
    if (mapState.offscreenCanvas) {
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(mapState.offscreenCanvas, 0, -mapState.height);
    }

    // 2. Draw Robot Trajectory Path
    if (robotPose.path && robotPose.path.length > 1) {
        ctx.strokeStyle = '#ff007a';
        ctx.lineWidth = 1.5 / view.scale;
        ctx.beginPath();
        for (let i = 0; i < robotPose.path.length; i++) {
            const px = (robotPose.path[i][0] - mapState.originX) / mapState.resolution;
            const py = -(robotPose.path[i][1] - mapState.originY) / mapState.resolution;
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();
    }

    // 3. Draw Robot Position & Live Laser Rays
    const rx = (robotPose.x - mapState.originX) / mapState.resolution;
    const ry = -(robotPose.y - mapState.originY) / mapState.resolution;

    // Draw Live LiDAR Sweep Points (2x bigger, vibrant green)
    if (liveScanPoints && liveScanPoints.length > 0) {
        ctx.fillStyle = '#00ff88';
        const cosYaw = Math.cos(robotPose.yaw);
        const sinYaw = Math.sin(robotPose.yaw);
        const ptSize = Math.max(1.2, 2.6 / view.scale); // 2x bigger
        for (let i = 0; i < liveScanPoints.length; i++) {
            const lx = liveScanPoints[i][0];
            const ly = liveScanPoints[i][1];
            const gx = robotPose.x + (lx * cosYaw - ly * sinYaw);
            const gy = robotPose.y + (lx * sinYaw + ly * cosYaw);

            const px = (gx - mapState.originX) / mapState.resolution;
            const py = -(gy - mapState.originY) / mapState.resolution;

            ctx.fillRect(px - ptSize / 2, py - ptSize / 2, ptSize, ptSize);
        }
    }

    // Draw Robot Body (compact precision dot)
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-robotPose.yaw);

    const rSize = Math.max(1.8, 4.0 / view.scale);
    ctx.fillStyle = '#ff007a';
    ctx.shadowColor = '#ff007a';
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.arc(0, 0, rSize, 0, Math.PI * 2);
    ctx.fill();

    // Heading Arrow
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = Math.max(1.2, 1.8 / view.scale);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(rSize * 2.8, 0);
    ctx.stroke();

    ctx.restore();
    ctx.restore();

    if (view.autoCenter) {
        centerOnRobot();
    }
}

function exportMapPNG() {
    if (!mapState.offscreenCanvas) return;
    const link = document.createElement('a');
    link.download = `cartographer_2d_map_${Date.now()}.png`;
    link.href = mapState.offscreenCanvas.toDataURL();
    link.click();
}

function connectWS() {
    const statusTag = document.getElementById('status-tag');
    const statusText = document.getElementById('status-text');
    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host || '10.43.8.42:8888';
    const url = `${proto}//${host}/ws`;

    ws = new WebSocket(url);

    ws.onopen = () => {
        statusTag.className = 'status-tag connected';
        statusText.textContent = 'ONLINE (CONTINUOUS SLAM)';
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'map_b64') {
                mapState.width = data.width;
                mapState.height = data.height;
                mapState.resolution = data.resolution;
                mapState.originX = data.origin_x;
                mapState.originY = data.origin_y;
                updateOffscreenMapFromBase64(data.grid_b64);
            } else if (data.type === 'scan') {
                liveScanPoints = data.points || [];
            } else if (data.type === 'pose') {
                robotPose.x = data.x;
                robotPose.y = data.y;
                robotPose.yaw = data.yaw;
                robotPose.path = data.path || [];
                document.getElementById('val-pose').textContent = `X: ${data.x.toFixed(2)}m | Y: ${data.y.toFixed(2)}m | Yaw: ${(data.yaw * 57.3).toFixed(1)}°`;
            } else if (data.type === 'telemetry') {
                updateTelemetryHUD(data);
            } else if (data.type === 'reset') {
                clearLocalMap();
            }
        } catch (e) {
            console.error("WS Parse error:", e);
        }
    };

    ws.onclose = () => {
        statusTag.className = 'status-tag';
        statusText.textContent = 'RECONNECTING';
        setTimeout(connectWS, 1500);
    };
}

window.addEventListener('DOMContentLoaded', () => {
    initCanvas();
    connectWS();
});
