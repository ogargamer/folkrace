/**
 * Modular Race Track Studio & Cartographer 2D Map Exporter
 * Fully Parameterized: Customizable Tile Size, Track Width, 100% Seamless Wall Alignment,
 * Exact Mouse-Aligned Obstacle Drawing & Down-Arrow Undo.
 */

// Track & Canvas State
let canvas, ctx;
let scale = 50.0; // Pixels per meter
let offsetX = 0;
let offsetY = 0;
let isDragging = false;
let dragStartX = 0;
let dragStartY = 0;

// Active Tool
let activeTool = 'select'; // 'select', 'rect', 'circle', 'startline', 'eraser'
let drawingItem = null;

// Track Parameters (in meters)
let TILE_SIZE = 1.2;     // 1200mm
let TRACK_WIDTH = 1.0;   // 1000mm
let WALL_MARGIN = 0.1;   // 100mm (computed: (TILE_SIZE - TRACK_WIDTH) / 2)
let WALL_HEIGHT = 0.30;  // 300mm (for 3D Webots World export)

// Track Definition
// Block Types: 'straight', 'turn_left', 'turn_right'
let trackBlocks = [];
let obstacles = []; // { type: 'rect'|'circle', ... }
let startLine = null; // { x, y, heading }

// Obstacle Selection & Editing State
let selectedObstacleIndex = -1;
let hoveredObstacleIndex = -1;
let movingObstacle = null;
let currentEditIndex = -1;

// Robot Initial Spawn Pose (Customizable starting position and heading)
let robotStartPose = {
    x: 0.60,
    y: 0.00,
    yaw_deg: 0.0
};

// Cursor state for adding next block
let cursor = {
    x: 0.0,
    y: 0.0,
    heading: 0 // 0: East (+X), 1: North (+Y), 2: West (-X), 3: South (-Y)
};

// Heading Unit Vectors (East, North, West, South)
const DIR_VECTORS = [
    { x: 1, y: 0 },  // 0: East (+X)
    { x: 0, y: 1 },  // 1: North (+Y)
    { x: -1, y: 0 }, // 2: West (-X)
    { x: 0, y: -1 }  // 3: South (-Y)
];

// Left Normal Vectors (perpendicular to heading)
const LEFT_VECTORS = [
    { x: 0, y: 1 },   // 0: North
    { x: -1, y: 0 },  // 1: West
    { x: 0, y: -1 },  // 2: South
    { x: 1, y: 0 }    // 3: East
];

function init() {
    canvas = document.getElementById('track-canvas');
    ctx = canvas.getContext('2d');

    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    setupEventListeners();
    setupKeyboardShortcuts();
    updateParametersFromInputs();
    updateRobotPoseInputs();
    recenterView();
    render();
}

function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
    render();
}

function getCanvasMousePos(e) {
    const rect = canvas.getBoundingClientRect();
    return {
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
    };
}

function updateParametersFromInputs() {
    const tileSizeInput = parseFloat(document.getElementById('input-tile-size').value) || 1200;
    const trackWidthInput = parseFloat(document.getElementById('input-track-width').value) || 1000;
    const wallHeightEl = document.getElementById('input-wall-height');
    const wallHeightInput = wallHeightEl ? parseFloat(wallHeightEl.value) || 300 : 300;

    TILE_SIZE = Math.max(0.3, tileSizeInput / 1000.0);
    TRACK_WIDTH = Math.max(0.1, Math.min(TILE_SIZE - 0.05, trackWidthInput / 1000.0));
    WALL_MARGIN = (TILE_SIZE - TRACK_WIDTH) / 2.0;
    WALL_HEIGHT = Math.max(0.05, wallHeightInput / 1000.0);

    // Update UI Badges & Labels
    const marginMm = Math.round(WALL_MARGIN * 1000);
    document.getElementById('val-wall-margin').textContent = `${marginMm} mm (${(marginMm / 10).toFixed(1)}cm)`;
    document.getElementById('badge-tile-size').textContent = `${Math.round(TILE_SIZE * 1000)}mm`;
    document.getElementById('label-straight').textContent = `Straight (${TILE_SIZE.toFixed(1)}m)`;

    // Rebuild existing blocks geometry to match new parameters
    rebuildTrackBlocksGeometry();
    updateStats();
    render();
}

function rebuildTrackBlocksGeometry() {
    let curX = 0.0;
    let curY = 0.0;
    let curH = 0;

    for (let i = 0; i < trackBlocks.length; i++) {
        const b = trackBlocks[i];
        b.startX = curX;
        b.startY = curY;
        b.startHeading = curH;

        const fwd = DIR_VECTORS[curH];
        const left = LEFT_VECTORS[curH];

        if (b.type === 'straight') {
            b.endX = curX + fwd.x * TILE_SIZE;
            b.endY = curY + fwd.y * TILE_SIZE;
            b.endHeading = curH;
        } else if (b.type === 'turn_left') {
            const newH = (curH + 1) % 4;
            const rCenter = TILE_SIZE / 2.0;
            b.center = {
                x: curX + left.x * rCenter,
                y: curY + left.y * rCenter
            };
            b.endX = curX + (fwd.x + left.x) * rCenter;
            b.endY = curY + (fwd.y + left.y) * rCenter;
            b.endHeading = newH;
            curH = newH;
        } else if (b.type === 'turn_right') {
            const newH = (curH + 3) % 4;
            const rCenter = TILE_SIZE / 2.0;
            const right = { x: -left.x, y: -left.y };
            b.center = {
                x: curX + right.x * rCenter,
                y: curY + right.y * rCenter
            };
            b.endX = curX + (fwd.x + right.x) * rCenter;
            b.endY = curY + (fwd.y + right.y) * rCenter;
            b.endHeading = newH;
            curH = newH;
        }

        curX = b.endX;
        curY = b.endY;
    }

    cursor.x = curX;
    cursor.y = curY;
    cursor.heading = curH;
}

function recenterView() {
    if (trackBlocks.length === 0) {
        offsetX = canvas.width / 2;
        offsetY = canvas.height / 2;
        scale = 60.0;
        return;
    }

    // Compute bounding box
    let minX = 0, maxX = 0, minY = 0, maxY = 0;
    for (const b of trackBlocks) {
        minX = Math.min(minX, b.startX, b.endX);
        maxX = Math.max(maxX, b.startX, b.endX);
        minY = Math.min(minY, b.startY, b.endY);
        maxY = Math.max(maxY, b.startY, b.endY);
    }

    const spanX = Math.max(4.0, (maxX - minX) + 2.4);
    const spanY = Math.max(4.0, (maxY - minY) + 2.4);
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;

    scale = Math.min(canvas.width / spanX, canvas.height / spanY) * 0.85;
    offsetX = canvas.width / 2 - midX * scale;
    offsetY = canvas.height / 2 + midY * scale;
    render();
}

// -------------------------------------------------------------
// BLOCK PLACEMENT (STRAIGHT, TURN LEFT, TURN RIGHT, UNDO)
// -------------------------------------------------------------

function addStraightBlock() {
    const fwd = DIR_VECTORS[cursor.heading];
    const block = {
        type: 'straight',
        startX: cursor.x,
        startY: cursor.y,
        startHeading: cursor.heading,
        endX: cursor.x + fwd.x * TILE_SIZE,
        endY: cursor.y + fwd.y * TILE_SIZE,
        endHeading: cursor.heading
    };

    trackBlocks.push(block);
    cursor.x = block.endX;
    cursor.y = block.endY;
    cursor.heading = block.endHeading;

    updateStats();
    render();
    showToast(`Added Straight (${TILE_SIZE.toFixed(1)}m)`, 'success');
}

function addTurnLeftBlock() {
    const fwd = DIR_VECTORS[cursor.heading];
    const left = LEFT_VECTORS[cursor.heading];
    const newHeading = (cursor.heading + 1) % 4;
    const rCenter = TILE_SIZE / 2.0;

    const center = {
        x: cursor.x + left.x * rCenter,
        y: cursor.y + left.y * rCenter
    };

    const block = {
        type: 'turn_left',
        startX: cursor.x,
        startY: cursor.y,
        startHeading: cursor.heading,
        endX: cursor.x + (fwd.x + left.x) * rCenter,
        endY: cursor.y + (fwd.y + left.y) * rCenter,
        endHeading: newHeading,
        center: center
    };

    trackBlocks.push(block);
    cursor.x = block.endX;
    cursor.y = block.endY;
    cursor.heading = block.endHeading;

    updateStats();
    render();
    showToast('Added Turn Left (90°)', 'success');
}

function addTurnRightBlock() {
    const fwd = DIR_VECTORS[cursor.heading];
    const left = LEFT_VECTORS[cursor.heading];
    const right = { x: -left.x, y: -left.y };
    const newHeading = (cursor.heading + 3) % 4;
    const rCenter = TILE_SIZE / 2.0;

    const center = {
        x: cursor.x + right.x * rCenter,
        y: cursor.y + right.y * rCenter
    };

    const block = {
        type: 'turn_right',
        startX: cursor.x,
        startY: cursor.y,
        startHeading: cursor.heading,
        endX: cursor.x + (fwd.x + right.x) * rCenter,
        endY: cursor.y + (fwd.y + right.y) * rCenter,
        endHeading: newHeading,
        center: center
    };

    trackBlocks.push(block);
    cursor.x = block.endX;
    cursor.y = block.endY;
    cursor.heading = block.endHeading;

    updateStats();
    render();
    showToast('Added Turn Right (90°)', 'success');
}

function undoPreviousBlock() {
    if (trackBlocks.length === 0) {
        showToast('No blocks to delete', 'error');
        return;
    }

    const removed = trackBlocks.pop();
    cursor.x = removed.startX;
    cursor.y = removed.startY;
    cursor.heading = removed.startHeading;

    updateStats();
    render();
    showToast('Deleted previous block (Undo)', 'success');
}

function clearTrack() {
    if (confirm('Clear entire track layout?')) {
        trackBlocks = [];
        obstacles = [];
        startLine = null;
        cursor.x = 0;
        cursor.y = 0;
        cursor.heading = 0;
        recenterView();
        updateStats();
        render();
        showToast('Track cleared', 'success');
    }
}

function updateStats() {
    document.getElementById('stat-blocks').textContent = trackBlocks.length;
    
    let totalLen = 0;
    const rCenter = TILE_SIZE / 2.0;
    for (const b of trackBlocks) {
        if (b.type === 'straight') {
            totalLen += TILE_SIZE;
        } else {
            totalLen += (Math.PI * rCenter) / 2.0;
        }
    }
    document.getElementById('stat-length').textContent = `${totalLen.toFixed(1)} m`;

    const closureEl = document.getElementById('stat-closure');
    if (trackBlocks.length >= 4) {
        const dist = Math.hypot(cursor.x - 0, cursor.y - 0);
        if (dist < 0.15 && cursor.heading === 0) {
            closureEl.textContent = 'CLOSED LOOP ✓';
            closureEl.style.color = 'var(--accent-green)';
        } else {
            closureEl.textContent = `Gap: ${dist.toFixed(2)}m`;
            closureEl.style.color = 'var(--accent-amber)';
        }
    } else {
        closureEl.textContent = 'Open Track';
        closureEl.style.color = 'var(--text-muted)';
    }
}

// -------------------------------------------------------------
// COORDINATE CONVERSIONS
// -------------------------------------------------------------

function worldToScreen(wx, wy) {
    return {
        x: offsetX + wx * scale,
        y: offsetY - wy * scale
    };
}

function screenToWorld(sx, sy) {
    return {
        x: (sx - offsetX) / scale,
        y: -(sy - offsetY) / scale
    };
}

// -------------------------------------------------------------
// RENDERING ENGINE (100% SEAMLESS ALIGNMENT)
// -------------------------------------------------------------

function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    drawGrid();
    drawTrack();
    drawObstacles();
    drawPlacementCursor();

    if (drawingItem) {
        drawTempItem(drawingItem);
    }
}

function drawGrid() {
    const minW = screenToWorld(0, canvas.height);
    const maxW = screenToWorld(canvas.width, 0);

    const startX = Math.floor(minW.x / TILE_SIZE) * TILE_SIZE;
    const endX = Math.ceil(maxW.x / TILE_SIZE) * TILE_SIZE;
    const startY = Math.floor(minW.y / TILE_SIZE) * TILE_SIZE;
    const endY = Math.ceil(maxW.y / TILE_SIZE) * TILE_SIZE;

    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';

    for (let x = startX; x <= endX; x += TILE_SIZE) {
        const p1 = worldToScreen(x, minW.y);
        const p2 = worldToScreen(x, maxW.y);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
    }

    for (let y = startY; y <= endY; y += TILE_SIZE) {
        const p1 = worldToScreen(minW.x, y);
        const p2 = worldToScreen(maxW.x, y);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
    }

    // Origin Axes
    const origin = worldToScreen(0, 0);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.25)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(origin.x - 20, origin.y);
    ctx.lineTo(origin.x + 20, origin.y);
    ctx.moveTo(origin.x, origin.y - 20);
    ctx.lineTo(origin.x, origin.y + 20);
    ctx.stroke();
}

function drawTrack() {
    for (let i = 0; i < trackBlocks.length; i++) {
        const b = trackBlocks[i];
        if (b.type === 'straight') {
            drawStraightTile(b);
        } else if (b.type === 'turn_left' || b.type === 'turn_right') {
            drawCurveTile(b);
        }
    }
}

function drawStraightTile(b) {
    const left = LEFT_VECTORS[b.startHeading];

    const pStart = worldToScreen(b.startX, b.startY);
    const pEnd = worldToScreen(b.endX, b.endY);

    // Mathematical wall center offset from centerline
    const wallOffset = (TILE_SIZE + TRACK_WIDTH) / 4.0;

    // 1. Drivable Road Surface
    ctx.save();
    ctx.strokeStyle = '#1a2233';
    ctx.lineWidth = TRACK_WIDTH * scale;
    ctx.lineCap = 'butt';
    ctx.beginPath();
    ctx.moveTo(pStart.x, pStart.y);
    ctx.lineTo(pEnd.x, pEnd.y);
    ctx.stroke();

    // Center dashed line
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.25)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    ctx.moveTo(pStart.x, pStart.y);
    ctx.lineTo(pEnd.x, pEnd.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // 2. Left Wall Barrier
    const leftWStart = worldToScreen(b.startX + left.x * wallOffset, b.startY + left.y * wallOffset);
    const leftWEnd = worldToScreen(b.endX + left.x * wallOffset, b.endY + left.y * wallOffset);
    ctx.strokeStyle = '#ff0055';
    ctx.lineWidth = WALL_MARGIN * scale;
    ctx.beginPath();
    ctx.moveTo(leftWStart.x, leftWStart.y);
    ctx.lineTo(leftWEnd.x, leftWEnd.y);
    ctx.stroke();

    // 3. Right Wall Barrier
    const rightWStart = worldToScreen(b.startX - left.x * wallOffset, b.startY - left.y * wallOffset);
    const rightWEnd = worldToScreen(b.endX - left.x * wallOffset, b.endY - left.y * wallOffset);
    ctx.strokeStyle = '#ff0055';
    ctx.lineWidth = WALL_MARGIN * scale;
    ctx.beginPath();
    ctx.moveTo(rightWStart.x, rightWStart.y);
    ctx.lineTo(rightWEnd.x, rightWEnd.y);
    ctx.stroke();

    ctx.restore();
}

function drawCurveTile(b) {
    const isLeft = b.type === 'turn_left';
    const cScreen = worldToScreen(b.center.x, b.center.y);

    const startAngle = Math.atan2(-(b.startY - b.center.y), b.startX - b.center.x);
    const endAngle = Math.atan2(-(b.endY - b.center.y), b.endX - b.center.x);

    // Exact radii matching the straight tile
    const rCenter = TILE_SIZE / 2.0;
    const rInnerWall = (TILE_SIZE - TRACK_WIDTH) / 4.0;
    const rOuterWall = TILE_SIZE - (TILE_SIZE - TRACK_WIDTH) / 4.0;

    ctx.save();

    // 1. Drivable Road Surface (Radius rCenter, Width TRACK_WIDTH)
    ctx.strokeStyle = '#1a2233';
    ctx.lineWidth = TRACK_WIDTH * scale;
    ctx.beginPath();
    ctx.arc(cScreen.x, cScreen.y, rCenter * scale, startAngle, endAngle, isLeft);
    ctx.stroke();

    // Center dashed line
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.25)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    ctx.arc(cScreen.x, cScreen.y, rCenter * scale, startAngle, endAngle, isLeft);
    ctx.stroke();
    ctx.setLineDash([]);

    // 2. Inner Wall Arc (Radius rInnerWall, Thickness WALL_MARGIN)
    ctx.strokeStyle = '#ff0055';
    ctx.lineWidth = WALL_MARGIN * scale;
    ctx.beginPath();
    ctx.arc(cScreen.x, cScreen.y, rInnerWall * scale, startAngle, endAngle, isLeft);
    ctx.stroke();

    // 3. Outer Wall Arc (Radius rOuterWall, Thickness WALL_MARGIN)
    ctx.strokeStyle = '#ff0055';
    ctx.lineWidth = WALL_MARGIN * scale;
    ctx.beginPath();
    ctx.arc(cScreen.x, cScreen.y, rOuterWall * scale, startAngle, endAngle, isLeft);
    ctx.stroke();

    ctx.restore();
}

function updateRobotPoseInputs() {
    document.getElementById('input-robot-x').value = robotStartPose.x.toFixed(2);
    document.getElementById('input-robot-y').value = robotStartPose.y.toFixed(2);
    document.getElementById('input-robot-yaw').value = Math.round(robotStartPose.yaw_deg);
    document.getElementById('label-robot-yaw').textContent = `${Math.round(robotStartPose.yaw_deg)}°`;
}

function snapRobotToStart() {
    if (trackBlocks.length > 0) {
        const b = trackBlocks[0];
        robotStartPose.x = (b.startX + b.endX) / 2.0;
        robotStartPose.y = (b.startY + b.endY) / 2.0;
        robotStartPose.yaw_deg = b.startHeading * 90.0;
    } else {
        robotStartPose.x = TILE_SIZE / 2.0;
        robotStartPose.y = 0.0;
        robotStartPose.yaw_deg = 0.0;
    }
    updateRobotPoseInputs();
    render();
    showToast('Snapped Robot to Track Start', 'success');
}

function findObstacleAt(wx, wy) {
    for (let i = obstacles.length - 1; i >= 0; i--) {
        const obs = obstacles[i];
        if (obs.type === 'circle') {
            const d = Math.hypot(wx - obs.cx, wy - obs.cy);
            if (d <= obs.radius + 0.06) return i;
        } else if (obs.type === 'rect') {
            const minX = Math.min(obs.x1, obs.x2) - 0.04;
            const maxX = Math.max(obs.x1, obs.x2) + 0.04;
            const minY = Math.min(obs.y1, obs.y2) - 0.04;
            const maxY = Math.max(obs.y1, obs.y2) + 0.04;
            if (wx >= minX && wx <= maxX && wy >= minY && wy <= maxY) return i;
        }
    }
    return -1;
}

function drawObstacles() {
    for (let i = 0; i < obstacles.length; i++) {
        const obs = obstacles[i];
        const isHovered = (i === hoveredObstacleIndex);
        const isSelected = (i === selectedObstacleIndex);

        if (obs.type === 'rect') {
            const p1 = worldToScreen(Math.min(obs.x1, obs.x2), Math.max(obs.y1, obs.y2));
            const w = Math.abs(obs.x2 - obs.x1) * scale;
            const h = Math.abs(obs.y2 - obs.y1) * scale;

            ctx.save();
            ctx.fillStyle = isSelected ? 'rgba(0, 240, 255, 0.7)' : (isHovered ? 'rgba(255, 215, 0, 0.9)' : 'rgba(255, 184, 0, 0.85)');
            ctx.strokeStyle = isSelected ? '#00f0ff' : (isHovered ? '#fff' : '#ffb800');
            ctx.lineWidth = (isHovered || isSelected) ? 2.5 : 1.5;
            ctx.fillRect(p1.x, p1.y, w, h);
            ctx.strokeRect(p1.x, p1.y, w, h);

            if (isHovered || isSelected) {
                // Dimensions tag
                const widthMm = Math.round(Math.abs(obs.x2 - obs.x1) * 1000);
                const heightMm = Math.round(Math.abs(obs.y2 - obs.y1) * 1000);
                ctx.font = '10px "JetBrains Mono", monospace';
                ctx.fillStyle = '#00f0ff';
                ctx.textAlign = 'center';
                ctx.fillText(`${widthMm}mm × ${heightMm}mm (Right-click to edit)`, p1.x + w / 2, p1.y - 6);
            }
            ctx.restore();

        } else if (obs.type === 'circle') {
            const center = worldToScreen(obs.cx, obs.cy);
            const r = obs.radius * scale;

            ctx.save();
            ctx.fillStyle = isSelected ? 'rgba(0, 240, 255, 0.7)' : (isHovered ? 'rgba(255, 215, 0, 0.9)' : 'rgba(255, 184, 0, 0.85)');
            ctx.strokeStyle = isSelected ? '#00f0ff' : (isHovered ? '#fff' : '#ffb800');
            ctx.lineWidth = (isHovered || isSelected) ? 2.5 : 1.5;
            ctx.beginPath();
            ctx.arc(center.x, center.y, r, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();

            if (isHovered || isSelected) {
                // Diameter tag
                const diamMm = Math.round(obs.radius * 2 * 1000);
                ctx.font = '10px "JetBrains Mono", monospace';
                ctx.fillStyle = '#00f0ff';
                ctx.textAlign = 'center';
                ctx.fillText(`Ø ${diamMm}mm (Right-click to edit)`, center.x, center.y - r - 6);
            }
            ctx.restore();
        }
    }

    if (startLine) {
        const p = worldToScreen(startLine.x, startLine.y);
        ctx.save();
        ctx.translate(p.x, p.y);
        ctx.rotate(-startLine.heading * (Math.PI / 2));
        ctx.fillStyle = '#00ff88';
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.fillRect(-4, -(TRACK_WIDTH * scale) / 2, 8, TRACK_WIDTH * scale);
        ctx.strokeRect(-4, -(TRACK_WIDTH * scale) / 2, 8, TRACK_WIDTH * scale);
        ctx.restore();
    }

    // Always Draw Robot Initial Spawn Pose
    drawRobotStartPose();
}

function drawRobotStartPose() {
    const p = worldToScreen(robotStartPose.x, robotStartPose.y);
    const rad = -robotStartPose.yaw_deg * (Math.PI / 180.0);
    const rSize = Math.max(12, 0.22 * scale); // ~22cm robot footprint

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(rad);

    // Glowing Chassis
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 14;
    ctx.fillStyle = '#00f0ff';
    ctx.beginPath();
    ctx.arc(0, 0, rSize, 0, Math.PI * 2);
    ctx.fill();

    // Dark Inner Core
    ctx.fillStyle = '#07090e';
    ctx.beginPath();
    ctx.arc(0, 0, rSize * 0.72, 0, Math.PI * 2);
    ctx.fill();

    // Directional Pointer Arrow
    ctx.strokeStyle = '#ff007a';
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(rSize * 1.9, 0);
    ctx.stroke();

    // Arrowhead
    ctx.fillStyle = '#ff007a';
    ctx.beginPath();
    ctx.moveTo(rSize * 1.9, 0);
    ctx.lineTo(rSize * 1.35, -5.5);
    ctx.lineTo(rSize * 1.35, 5.5);
    ctx.closePath();
    ctx.fill();

    // Wheels representation
    ctx.fillStyle = '#ffb800';
    ctx.fillRect(-rSize * 0.7, -rSize - 3, rSize * 1.4, 5);
    ctx.fillRect(-rSize * 0.7, rSize - 2, rSize * 1.4, 5);

    ctx.restore();

    // Coordinate Label Tag
    ctx.save();
    ctx.font = '11px "JetBrains Mono", monospace';
    ctx.fillStyle = '#00f0ff';
    ctx.textAlign = 'center';
    ctx.fillText(`ROBOT START (${robotStartPose.x.toFixed(2)}m, ${robotStartPose.y.toFixed(2)}m, ${Math.round(robotStartPose.yaw_deg)}°)`, p.x, p.y - rSize - 10);
    ctx.restore();
}

function drawPlacementCursor() {
    const p = worldToScreen(cursor.x, cursor.y);
    const fwd = DIR_VECTORS[cursor.heading];

    ctx.save();
    ctx.fillStyle = '#00f0ff';
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 12;

    // Pulsing Cursor Point
    ctx.beginPath();
    ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
    ctx.fill();

    // Forward Heading Arrow
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x + fwd.x * 24, p.y - fwd.y * 24);
    ctx.stroke();

    ctx.restore();
}

function drawTempItem(item) {
    if (item.type === 'rect') {
        const p1 = worldToScreen(Math.min(item.x1, item.x2), Math.max(item.y1, item.y2));
        const w = Math.abs(item.x2 - item.x1) * scale;
        const h = Math.abs(item.y2 - item.y1) * scale;

        ctx.fillStyle = 'rgba(255, 184, 0, 0.4)';
        ctx.strokeStyle = '#ffb800';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        ctx.fillRect(p1.x, p1.y, w, h);
        ctx.strokeRect(p1.x, p1.y, w, h);
        ctx.setLineDash([]);
    } else if (item.type === 'circle') {
        const center = worldToScreen(item.cx, item.cy);
        const r = item.radius * scale;

        ctx.fillStyle = 'rgba(255, 184, 0, 0.4)';
        ctx.strokeStyle = '#ffb800';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(center.x, center.y, r, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.setLineDash([]);
    }
}

// -------------------------------------------------------------
// EVENT LISTENERS & EXACT CANVAS MOUSE COORDINATES
// -------------------------------------------------------------

function setupEventListeners() {
    // Parameter Inputs
    document.getElementById('input-tile-size').addEventListener('input', updateParametersFromInputs);
    document.getElementById('input-track-width').addEventListener('input', updateParametersFromInputs);
    const wallHeightEl = document.getElementById('input-wall-height');
    if (wallHeightEl) wallHeightEl.addEventListener('input', updateParametersFromInputs);

    // Robot Initial Pose Inputs
    document.getElementById('input-robot-x').addEventListener('input', (e) => {
        robotStartPose.x = parseFloat(e.target.value) || 0.0;
        render();
    });
    document.getElementById('input-robot-y').addEventListener('input', (e) => {
        robotStartPose.y = parseFloat(e.target.value) || 0.0;
        render();
    });
    document.getElementById('input-robot-yaw').addEventListener('input', (e) => {
        robotStartPose.yaw_deg = parseFloat(e.target.value) || 0.0;
        document.getElementById('label-robot-yaw').textContent = `${Math.round(robotStartPose.yaw_deg)}°`;
        render();
    });
    document.getElementById('btn-snap-robot').addEventListener('click', snapRobotToStart);

    // Block Buttons
    document.getElementById('btn-add-straight').addEventListener('click', addStraightBlock);
    document.getElementById('btn-add-left').addEventListener('click', addTurnLeftBlock);
    document.getElementById('btn-add-right').addEventListener('click', addTurnRightBlock);
    document.getElementById('btn-undo-block').addEventListener('click', undoPreviousBlock);
    document.getElementById('btn-clear').addEventListener('click', clearTrack);

    // Load JSON Button & File Picker
    const btnLoadJson = document.getElementById('btn-load-json');
    const inputLoadJson = document.getElementById('input-load-json');
    if (btnLoadJson && inputLoadJson) {
        btnLoadJson.addEventListener('click', () => inputLoadJson.click());
        inputLoadJson.addEventListener('change', onJsonFileSelected);
    }

    // Zoom & Recenter
    document.getElementById('btn-zoom-in').addEventListener('click', () => { scale *= 1.2; render(); });
    document.getElementById('btn-zoom-out').addEventListener('click', () => { scale /= 1.2; render(); });
    document.getElementById('btn-recenter').addEventListener('click', recenterView);

    // Tool Selector
    document.querySelectorAll('.btn-tool').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.btn-tool').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            activeTool = btn.dataset.tool;
            if (activeTool === 'select') canvas.style.cursor = 'grab';
            else if (activeTool === 'move') canvas.style.cursor = 'default';
            else canvas.style.cursor = 'crosshair';
        });
    });

    // Exporters
    document.getElementById('btn-export-png').addEventListener('click', exportMapPNG);
    document.getElementById('btn-export-yaml').addEventListener('click', exportMapYAML);
    document.getElementById('btn-save-json').addEventListener('click', exportTrackJSON);
    
    // Webots Exporters (Header & Sidebar)
    const btnWebots = document.getElementById('btn-export-webots');
    if (btnWebots) btnWebots.addEventListener('click', exportWebotsWorld);
    const btnWebotsSide = document.getElementById('btn-export-webots-side');
    if (btnWebotsSide) btnWebotsSide.addEventListener('click', exportWebotsWorld);

    document.getElementById('btn-deploy-pi').addEventListener('click', deployMapToPi);

    // Mouse Interaction with getCanvasMousePos() for exact alignment
    canvas.addEventListener('mousedown', onMouseDown);
    canvas.addEventListener('mousemove', onMouseMove);
    window.addEventListener('mouseup', onMouseUp);
    canvas.addEventListener('wheel', onWheel);
    canvas.addEventListener('contextmenu', onContextMenu);

    // Modal Action Buttons
    document.getElementById('modal-btn-close').addEventListener('click', closeObstacleDimensionModal);
    document.getElementById('modal-btn-cancel').addEventListener('click', closeObstacleDimensionModal);
    document.getElementById('modal-btn-save').addEventListener('click', saveObstacleDimensions);
    document.getElementById('modal-btn-delete').addEventListener('click', deleteObstacleFromModal);
}

function setupKeyboardShortcuts() {
    window.addEventListener('keydown', (e) => {
        const modal = document.getElementById('obstacle-modal');
        if (modal.style.display === 'flex') {
            if (e.key === 'Escape') {
                closeObstacleDimensionModal();
            } else if (e.key === 'Enter') {
                saveObstacleDimensions();
            }
            return;
        }

        if (e.target.tagName === 'INPUT') return;

        if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') {
            addStraightBlock();
        } else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') {
            addTurnLeftBlock();
        } else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') {
            addTurnRightBlock();
        } else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown' || e.key === 'Backspace' || e.key === 'Delete') {
            undoPreviousBlock();
        }
    });
}

function onMouseDown(e) {
    if (e.button === 2) return; // Right-click is handled by onContextMenu

    const m = getCanvasMousePos(e);
    const wPos = screenToWorld(m.x, m.y);

    if (activeTool === 'move') {
        const idx = findObstacleAt(wPos.x, wPos.y);
        if (idx !== -1) {
            selectedObstacleIndex = idx;
            const obs = obstacles[idx];
            movingObstacle = {
                index: idx,
                startMouseW: { x: wPos.x, y: wPos.y },
                orig: JSON.parse(JSON.stringify(obs))
            };
            canvas.style.cursor = 'grabbing';
            render();
            return;
        } else {
            // Pan if clicked on empty canvas
            isDragging = true;
            dragStartX = m.x - offsetX;
            dragStartY = m.y - offsetY;
            canvas.style.cursor = 'grabbing';
            return;
        }
    }

    if (activeTool === 'select' || e.button === 1 || (e.button === 0 && e.shiftKey)) {
        isDragging = true;
        dragStartX = m.x - offsetX;
        dragStartY = m.y - offsetY;
        canvas.style.cursor = 'grabbing';
    } else if (activeTool === 'robot') {
        robotStartPose.x = wPos.x;
        robotStartPose.y = wPos.y;
        updateRobotPoseInputs();
        render();
        showToast('Robot position updated (drag to set heading)', 'success');
    } else if (activeTool === 'rect') {
        drawingItem = {
            type: 'rect',
            x1: wPos.x,
            y1: wPos.y,
            x2: wPos.x,
            y2: wPos.y
        };
    } else if (activeTool === 'circle') {
        drawingItem = {
            type: 'circle',
            cx: wPos.x,
            cy: wPos.y,
            radius: 0.05
        };
    } else if (activeTool === 'startline') {
        startLine = {
            x: wPos.x,
            y: wPos.y,
            heading: cursor.heading
        };
        showToast('Start Line placed', 'success');
        render();
    } else if (activeTool === 'eraser') {
        eraseClosestObstacle(wPos.x, wPos.y);
    }
}

function onMouseMove(e) {
    const m = getCanvasMousePos(e);

    if (movingObstacle) {
        const wPos = screenToWorld(m.x, m.y);
        const dx = wPos.x - movingObstacle.startMouseW.x;
        const dy = wPos.y - movingObstacle.startMouseW.y;
        const obs = obstacles[movingObstacle.index];

        if (obs.type === 'rect') {
            obs.x1 = movingObstacle.orig.x1 + dx;
            obs.x2 = movingObstacle.orig.x2 + dx;
            obs.y1 = movingObstacle.orig.y1 + dy;
            obs.y2 = movingObstacle.orig.y2 + dy;
        } else if (obs.type === 'circle') {
            obs.cx = movingObstacle.orig.cx + dx;
            obs.cy = movingObstacle.orig.cy + dy;
        }
        render();
        return;
    }

    if (isDragging) {
        offsetX = m.x - dragStartX;
        offsetY = m.y - dragStartY;
        render();
        return;
    }

    if (activeTool === 'move') {
        const wPos = screenToWorld(m.x, m.y);
        const hIdx = findObstacleAt(wPos.x, wPos.y);
        if (hIdx !== hoveredObstacleIndex) {
            hoveredObstacleIndex = hIdx;
            canvas.style.cursor = (hIdx !== -1) ? 'grab' : 'default';
            render();
        }
        return;
    }

    if (activeTool === 'robot' && e.buttons === 1) {
        const wPos = screenToWorld(m.x, m.y);
        const angleRad = Math.atan2(wPos.y - robotStartPose.y, wPos.x - robotStartPose.x);
        robotStartPose.yaw_deg = angleRad * (180.0 / Math.PI);
        updateRobotPoseInputs();
        render();
        return;
    }

    if (drawingItem) {
        const wPos = screenToWorld(m.x, m.y);
        if (drawingItem.type === 'rect') {
            drawingItem.x2 = wPos.x;
            drawingItem.y2 = wPos.y;
        } else if (drawingItem.type === 'circle') {
            drawingItem.radius = Math.hypot(wPos.x - drawingItem.cx, wPos.y - drawingItem.cy);
        }
        render();
    }
}

function onMouseUp(e) {
    if (movingObstacle) {
        movingObstacle = null;
        canvas.style.cursor = (activeTool === 'move') ? 'grab' : 'crosshair';
        showToast('Moved obstacle', 'success');
        render();
        return;
    }

    if (isDragging) {
        isDragging = false;
        canvas.style.cursor = (activeTool === 'select') ? 'grab' : ((activeTool === 'move') ? 'default' : 'crosshair');
    }

    if (drawingItem) {
        if (drawingItem.type === 'rect') {
            if (Math.abs(drawingItem.x2 - drawingItem.x1) > 0.05 && Math.abs(drawingItem.y2 - drawingItem.y1) > 0.05) {
                obstacles.push(drawingItem);
                showToast('Rectangle added (Right-click to edit size)', 'success');
            }
        } else if (drawingItem.type === 'circle') {
            if (drawingItem.radius > 0.05) {
                obstacles.push(drawingItem);
                showToast('Circle pillar added (Right-click to edit size)', 'success');
            }
        }
        drawingItem = null;
        render();
    }
}

function onContextMenu(e) {
    e.preventDefault();
    const m = getCanvasMousePos(e);
    const wPos = screenToWorld(m.x, m.y);

    const idx = findObstacleAt(wPos.x, wPos.y);
    if (idx !== -1) {
        selectedObstacleIndex = idx;
        render();
        openObstacleDimensionModal(idx);
    }
}

function openObstacleDimensionModal(idx) {
    currentEditIndex = idx;
    const obs = obstacles[idx];
    const modal = document.getElementById('obstacle-modal');
    const modalBody = document.getElementById('modal-body');
    const modalTitle = document.getElementById('modal-title');

    if (obs.type === 'rect') {
        const curW = Math.round(Math.abs(obs.x2 - obs.x1) * 1000);
        const curH = Math.round(Math.abs(obs.y2 - obs.y1) * 1000);
        const curCx = ((obs.x1 + obs.x2) / 2.0).toFixed(3);
        const curCy = ((obs.y1 + obs.y2) / 2.0).toFixed(3);

        modalTitle.innerHTML = '<span>⬛</span> Rectangle Dimensions';
        modalBody.innerHTML = `
            <div class="modal-row">
                <div class="modal-field">
                    <label>Width (mm)</label>
                    <input type="number" id="dim-rect-w" value="${curW}" step="10" min="20" max="5000" autofocus>
                </div>
                <div class="modal-field">
                    <label>Height (mm)</label>
                    <input type="number" id="dim-rect-h" value="${curH}" step="10" min="20" max="5000">
                </div>
            </div>
            <div class="modal-row">
                <div class="modal-field">
                    <label>Center X (m)</label>
                    <input type="number" id="dim-cx" value="${curCx}" step="0.05">
                </div>
                <div class="modal-field">
                    <label>Center Y (m)</label>
                    <input type="number" id="dim-cy" value="${curCy}" step="0.05">
                </div>
            </div>
        `;
    } else if (obs.type === 'circle') {
        const curDiam = Math.round(obs.radius * 2 * 1000);
        const curCx = obs.cx.toFixed(3);
        const curCy = obs.cy.toFixed(3);

        modalTitle.innerHTML = '<span>⚪</span> Circle Pillar Dimensions';
        modalBody.innerHTML = `
            <div class="modal-field">
                <label>Diameter (mm)</label>
                <input type="number" id="dim-circle-d" value="${curDiam}" step="10" min="20" max="5000" autofocus>
            </div>
            <div class="modal-row">
                <div class="modal-field">
                    <label>Center X (m)</label>
                    <input type="number" id="dim-cx" value="${curCx}" step="0.05">
                </div>
                <div class="modal-field">
                    <label>Center Y (m)</label>
                    <input type="number" id="dim-cy" value="${curCy}" step="0.05">
                </div>
            </div>
        `;
    }

    modal.style.display = 'flex';
    setTimeout(() => {
        const firstInput = modalBody.querySelector('input');
        if (firstInput) {
            firstInput.focus();
            firstInput.select();
        }
    }, 50);
}

function closeObstacleDimensionModal() {
    document.getElementById('obstacle-modal').style.display = 'none';
    currentEditIndex = -1;
    render();
}

function saveObstacleDimensions() {
    if (currentEditIndex < 0 || currentEditIndex >= obstacles.length) {
        closeObstacleDimensionModal();
        return;
    }

    const obs = obstacles[currentEditIndex];
    const cx = parseFloat(document.getElementById('dim-cx').value) || 0.0;
    const cy = parseFloat(document.getElementById('dim-cy').value) || 0.0;

    if (obs.type === 'rect') {
        const w = (parseFloat(document.getElementById('dim-rect-w').value) || 200) / 1000.0;
        const h = (parseFloat(document.getElementById('dim-rect-h').value) || 200) / 1000.0;
        obs.x1 = cx - w / 2.0;
        obs.x2 = cx + w / 2.0;
        obs.y1 = cy - h / 2.0;
        obs.y2 = cy + h / 2.0;
    } else if (obs.type === 'circle') {
        const diam = (parseFloat(document.getElementById('dim-circle-d').value) || 200) / 1000.0;
        obs.cx = cx;
        obs.cy = cy;
        obs.radius = Math.max(0.01, diam / 2.0);
    }

    closeObstacleDimensionModal();
    render();
    showToast('Updated obstacle dimensions', 'success');
}

function deleteObstacleFromModal() {
    if (currentEditIndex >= 0 && currentEditIndex < obstacles.length) {
        obstacles.splice(currentEditIndex, 1);
        closeObstacleDimensionModal();
        render();
        showToast('Obstacle deleted', 'success');
    }
}

function onWheel(e) {
    e.preventDefault();
    const m = getCanvasMousePos(e);
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    
    // Zoom centered around exact mouse point
    const mouseW = screenToWorld(m.x, m.y);
    scale *= zoomFactor;
    offsetX = m.x - mouseW.x * scale;
    offsetY = m.y + mouseW.y * scale;
    render();
}

function eraseClosestObstacle(wx, wy) {
    let bestIdx = -1;
    let bestDist = 0.5;

    for (let i = 0; i < obstacles.length; i++) {
        const obs = obstacles[i];
        let d = 999;
        if (obs.type === 'circle') {
            d = Math.hypot(wx - obs.cx, wy - obs.cy);
        } else if (obs.type === 'rect') {
            const cx = (obs.x1 + obs.x2) / 2;
            const cy = (obs.y1 + obs.y2) / 2;
            d = Math.hypot(wx - cx, wy - cy);
        }

        if (d < bestDist) {
            bestDist = d;
            bestIdx = i;
        }
    }

    if (bestIdx !== -1) {
        obstacles.splice(bestIdx, 1);
        showToast('Obstacle erased', 'success');
        render();
    }
}

// -------------------------------------------------------------
// ROS 2 OCCUPANCY GRID GENERATION (2cm / 0.02m Resolution)
// -------------------------------------------------------------

function generateOccupancyGrid() {
    if (trackBlocks.length === 0) {
        alert('Please build a track first before exporting!');
        return null;
    }

    let minX = 0, maxX = 0, minY = 0, maxY = 0;
    for (const b of trackBlocks) {
        minX = Math.min(minX, b.startX, b.endX);
        maxX = Math.max(maxX, b.startX, b.endX);
        minY = Math.min(minY, b.startY, b.endY);
        maxY = Math.max(maxY, b.startY, b.endY);
    }
    for (const obs of obstacles) {
        if (obs.type === 'rect') {
            minX = Math.min(minX, obs.x1, obs.x2);
            maxX = Math.max(maxX, obs.x1, obs.x2);
            minY = Math.min(minY, obs.y1, obs.y2);
            maxY = Math.max(maxY, obs.y1, obs.y2);
        } else if (obs.type === 'circle') {
            minX = Math.min(minX, obs.cx - obs.radius);
            maxX = Math.max(maxX, obs.cx + obs.radius);
            minY = Math.min(minY, obs.cy - obs.radius);
            maxY = Math.max(maxY, obs.cy + obs.radius);
        }
    }

    const PADDING = 1.0;
    minX -= PADDING;
    maxX += PADDING;
    minY -= PADDING;
    maxY += PADDING;

    const RESOLUTION = 0.02; // 2cm
    const gridWidth = Math.ceil((maxX - minX) / RESOLUTION);
    const gridHeight = Math.ceil((maxY - minY) / RESOLUTION);

    const offscreen = document.createElement('canvas');
    offscreen.width = gridWidth;
    offscreen.height = gridHeight;
    const octx = offscreen.getContext('2d');

    // Fill entire grid with Unknown space (Gray 205)
    octx.fillStyle = '#cdcdcd';
    octx.fillRect(0, 0, gridWidth, gridHeight);

    function mapWorldToGrid(wx, wy) {
        return {
            gx: (wx - minX) / RESOLUTION,
            gy: gridHeight - (wy - minY) / RESOLUTION
        };
    }

    const wallOffset = (TILE_SIZE + TRACK_WIDTH) / 4.0;
    const rCenter = TILE_SIZE / 2.0;
    const rInnerWall = (TILE_SIZE - TRACK_WIDTH) / 4.0;
    const rOuterWall = TILE_SIZE - (TILE_SIZE - TRACK_WIDTH) / 4.0;

    // Step A: Rasterize Free Space (Drivable corridor)
    octx.fillStyle = '#ffffff';
    octx.strokeStyle = '#ffffff';
    octx.lineCap = 'round';
    octx.lineWidth = TRACK_WIDTH / RESOLUTION;

    for (const b of trackBlocks) {
        if (b.type === 'straight') {
            const p1 = mapWorldToGrid(b.startX, b.startY);
            const p2 = mapWorldToGrid(b.endX, b.endY);
            octx.beginPath();
            octx.moveTo(p1.gx, p1.gy);
            octx.lineTo(p2.gx, p2.gy);
            octx.stroke();
        } else {
            const c = mapWorldToGrid(b.center.x, b.center.y);
            const isLeft = b.type === 'turn_left';
            const sAngle = Math.atan2(-(b.startY - b.center.y), b.startX - b.center.x);
            const eAngle = Math.atan2(-(b.endY - b.center.y), b.endX - b.center.x);
            octx.beginPath();
            octx.arc(c.gx, c.gy, rCenter / RESOLUTION, sAngle, eAngle, isLeft);
            octx.stroke();
        }
    }

    // Step B: Rasterize Solid Barrier Walls (Black 0)
    octx.fillStyle = '#000000';
    octx.strokeStyle = '#000000';
    octx.lineWidth = WALL_MARGIN / RESOLUTION;

    for (const b of trackBlocks) {
        const left = LEFT_VECTORS[b.startHeading];
        if (b.type === 'straight') {
            // Left Wall
            const l1 = mapWorldToGrid(b.startX + left.x * wallOffset, b.startY + left.y * wallOffset);
            const l2 = mapWorldToGrid(b.endX + left.x * wallOffset, b.endY + left.y * wallOffset);
            octx.beginPath();
            octx.moveTo(l1.gx, l1.gy);
            octx.lineTo(l2.gx, l2.gy);
            octx.stroke();

            // Right Wall
            const r1 = mapWorldToGrid(b.startX - left.x * wallOffset, b.startY - left.y * wallOffset);
            const r2 = mapWorldToGrid(b.endX - left.x * wallOffset, b.endY - left.y * wallOffset);
            octx.beginPath();
            octx.moveTo(r1.gx, r1.gy);
            octx.lineTo(r2.gx, r2.gy);
            octx.stroke();
        } else {
            const c = mapWorldToGrid(b.center.x, b.center.y);
            const isLeft = b.type === 'turn_left';
            const sAngle = Math.atan2(-(b.startY - b.center.y), b.startX - b.center.x);
            const eAngle = Math.atan2(-(b.endY - b.center.y), b.endX - b.center.x);

            // Inner Wall
            octx.beginPath();
            octx.arc(c.gx, c.gy, rInnerWall / RESOLUTION, sAngle, eAngle, isLeft);
            octx.stroke();

            // Outer Wall
            octx.beginPath();
            octx.arc(c.gx, c.gy, rOuterWall / RESOLUTION, sAngle, eAngle, isLeft);
            octx.stroke();
        }
    }

    // Step C: Rasterize Custom Obstacles (Black 0)
    for (const obs of obstacles) {
        if (obs.type === 'rect') {
            const p1 = mapWorldToGrid(Math.min(obs.x1, obs.x2), Math.max(obs.y1, obs.y2));
            const w = Math.abs(obs.x2 - obs.x1) / RESOLUTION;
            const h = Math.abs(obs.y2 - obs.y1) / RESOLUTION;
            octx.fillRect(p1.gx, p1.gy, w, h);
        } else if (obs.type === 'circle') {
            const c = mapWorldToGrid(obs.cx, obs.cy);
            octx.beginPath();
            octx.arc(c.gx, c.gy, obs.radius / RESOLUTION, 0, Math.PI * 2);
            octx.fill();
        }
    }

    return {
        canvas: offscreen,
        width: gridWidth,
        height: gridHeight,
        resolution: RESOLUTION,
        originX: minX,
        originY: minY
    };
}

// -------------------------------------------------------------
// EXPORTERS & 1-CLICK PI 5 DEPLOYMENT (WITH INITIAL POSE)
// -------------------------------------------------------------

function exportMapPNG() {
    const grid = generateOccupancyGrid();
    if (!grid) return;

    const link = document.createElement('a');
    link.download = `track_map_${Date.now()}.png`;
    link.href = grid.canvas.toDataURL('image/png');
    link.click();
    showToast('Exported Map PNG', 'success');
}

function exportMapYAML() {
    const grid = generateOccupancyGrid();
    if (!grid) return;

    const yawRad = (robotStartPose.yaw_deg * Math.PI / 180.0).toFixed(4);
    const yamlContent = `image: track_map.png
resolution: ${grid.resolution}
origin: [${grid.originX.toFixed(3)}, ${grid.originY.toFixed(3)}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# Initial Robot Spawn Pose (ROS 2 / Cartographer / Nav2 standard)
initial_pose:
  x: ${robotStartPose.x.toFixed(3)}
  y: ${robotStartPose.y.toFixed(3)}
  z: 0.0
  yaw: ${yawRad}
`;

    const blob = new Blob([yamlContent], { type: 'text/yaml' });
    const link = document.createElement('a');
    link.download = `track_map.yaml`;
    link.href = URL.createObjectURL(blob);
    link.click();
    showToast('Exported ROS map.yaml (with Initial Pose)', 'success');
}

function exportTrackJSON() {
    const data = {
        parameters: {
            tileSize: TILE_SIZE,
            trackWidth: TRACK_WIDTH,
            wallMargin: WALL_MARGIN,
            wallHeight: WALL_HEIGHT
        },
        initial_pose: {
            x: robotStartPose.x,
            y: robotStartPose.y,
            yaw_deg: robotStartPose.yaw_deg,
            yaw_rad: robotStartPose.yaw_deg * Math.PI / 180.0
        },
        blocks: trackBlocks,
        obstacles: obstacles,
        startLine: startLine,
        timestamp: Date.now()
    };

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const link = document.createElement('a');
    link.download = `track_layout_${Date.now()}.json`;
    link.href = URL.createObjectURL(blob);
    link.click();
    showToast('Saved Track JSON (with Initial Pose)', 'success');
}

// -------------------------------------------------------------
// TRACK JSON IMPORTER (LOAD SAVED TRACK)
// -------------------------------------------------------------

function onJsonFileSelected(e) {
    const file = e.target.files[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (event) => {
        try {
            const data = JSON.parse(event.target.result);
            loadTrackJSONData(data);
            showToast('Track layout loaded successfully!', 'success');
        } catch (err) {
            alert('Failed to parse JSON track layout file: ' + err.message);
            showToast('Invalid JSON file', 'error');
        }
        // Reset file input so same file can be reloaded if needed
        e.target.value = '';
    };
    reader.readAsText(file);
}

function loadTrackJSONData(data) {
    if (!data) return;

    // 1. Restore Parameters if present
    if (data.parameters) {
        if (data.parameters.tileSize) {
            TILE_SIZE = parseFloat(data.parameters.tileSize) || 1.2;
            const inputTile = document.getElementById('input-tile-size');
            if (inputTile) inputTile.value = Math.round(TILE_SIZE * 1000);
        }
        if (data.parameters.trackWidth) {
            TRACK_WIDTH = parseFloat(data.parameters.trackWidth) || 1.0;
            const inputWidth = document.getElementById('input-track-width');
            if (inputWidth) inputWidth.value = Math.round(TRACK_WIDTH * 1000);
        }
        if (data.parameters.wallHeight) {
            WALL_HEIGHT = parseFloat(data.parameters.wallHeight) || 0.30;
            const inputWallH = document.getElementById('input-wall-height');
            if (inputWallH) inputWallH.value = Math.round(WALL_HEIGHT * 1000);
        }
        WALL_MARGIN = (TILE_SIZE - TRACK_WIDTH) / 2.0;
    }

    // 2. Restore Blocks
    if (Array.isArray(data.blocks)) {
        trackBlocks = JSON.parse(JSON.stringify(data.blocks));
    } else {
        trackBlocks = [];
    }

    // 3. Restore Obstacles
    if (Array.isArray(data.obstacles)) {
        obstacles = JSON.parse(JSON.stringify(data.obstacles));
    } else {
        obstacles = [];
    }

    // 4. Restore Start Line
    if (data.startLine) {
        startLine = JSON.parse(JSON.stringify(data.startLine));
    } else {
        startLine = null;
    }

    // 5. Restore Initial Robot Pose
    if (data.initial_pose) {
        robotStartPose.x = parseFloat(data.initial_pose.x) || 0.60;
        robotStartPose.y = parseFloat(data.initial_pose.y) || 0.00;
        robotStartPose.yaw_deg = parseFloat(data.initial_pose.yaw_deg) || 0.0;
    }

    updateParametersFromInputs();
    updateRobotPoseInputs();
    rebuildTrackBlocksGeometry();
    recenterView();
    updateStats();
    render();
}

// -------------------------------------------------------------
// WEBOTS 3D WORLD EXPORTER (STANDARD ENU COORDINATE SYSTEM: Z IS UP)
// -------------------------------------------------------------

function buildTrackWallPoints(stepSize = 0.04) {
    const leftWall = [];
    const rightWall = [];
    const centerline = [];
    const halfWidth = TRACK_WIDTH / 2.0;

    for (const b of trackBlocks) {
        const curH = b.startHeading;
        const fwd = DIR_VECTORS[curH];
        const left = LEFT_VECTORS[curH];

        if (b.type === 'straight') {
            const steps = Math.max(8, Math.ceil(TILE_SIZE / stepSize));
            for (let i = 1; i <= steps; i++) {
                const t = i / steps;
                const px = b.startX + fwd.x * TILE_SIZE * t;
                const py = b.startY + fwd.y * TILE_SIZE * t;
                leftWall.push({ x: px + left.x * halfWidth, y: py + left.y * halfWidth });
                rightWall.push({ x: px - left.x * halfWidth, y: py - left.y * halfWidth });
                centerline.push({ x: px, y: py });
            }
        } else if (b.type === 'turn_left') {
            const rCenter = TILE_SIZE / 2.0;
            const c = b.center;
            const startAngle = Math.atan2(b.startY - c.y, b.startX - c.x);
            const arcLen = (Math.PI / 2.0) * rCenter;
            const steps = Math.max(12, Math.ceil(arcLen / stepSize));

            for (let i = 1; i <= steps; i++) {
                const angle = startAngle + (Math.PI / 2.0) * (i / steps);
                const cosA = Math.cos(angle);
                const sinA = Math.sin(angle);
                leftWall.push({
                    x: c.x + (rCenter - halfWidth) * cosA,
                    y: c.y + (rCenter - halfWidth) * sinA
                });
                rightWall.push({
                    x: c.x + (rCenter + halfWidth) * cosA,
                    y: c.y + (rCenter + halfWidth) * sinA
                });
                centerline.push({
                    x: c.x + rCenter * cosA,
                    y: c.y + rCenter * sinA
                });
            }
        } else if (b.type === 'turn_right') {
            const rCenter = TILE_SIZE / 2.0;
            const c = b.center;
            const startAngle = Math.atan2(b.startY - c.y, b.startX - c.x);
            const arcLen = (Math.PI / 2.0) * rCenter;
            const steps = Math.max(12, Math.ceil(arcLen / stepSize));

            for (let i = 1; i <= steps; i++) {
                const angle = startAngle - (Math.PI / 2.0) * (i / steps);
                const cosA = Math.cos(angle);
                const sinA = Math.sin(angle);
                leftWall.push({
                    x: c.x + (rCenter + halfWidth) * cosA,
                    y: c.y + (rCenter + halfWidth) * sinA
                });
                rightWall.push({
                    x: c.x + (rCenter - halfWidth) * cosA,
                    y: c.y + (rCenter - halfWidth) * sinA
                });
                centerline.push({
                    x: c.x + rCenter * cosA,
                    y: c.y + rCenter * sinA
                });
            }
        }
    }

    return { leftWall, rightWall, centerline };
}

function buildIndexedFaceSetWall(pts, name, colorStr, wallHeight) {
    const N = pts.length;
    if (N < 2) return '';

    // Check if the track forms a closed loop
    const dist = Math.hypot(pts[0].x - pts[N - 1].x, pts[0].y - pts[N - 1].y);
    const isClosed = dist < 0.20;

    // ENU Coordinate System: (x, y) are horizontal floor coordinates, Z is height (0 to wallHeight)
    const coordLines = [];
    // Bottom vertices (z = 0.0000)
    for (let i = 0; i < N; i++) {
        coordLines.push(`${pts[i].x.toFixed(4)} ${pts[i].y.toFixed(4)} 0.0000`);
    }
    // Top vertices (z = wallHeight)
    for (let i = 0; i < N; i++) {
        coordLines.push(`${pts[i].x.toFixed(4)} ${pts[i].y.toFixed(4)} ${wallHeight.toFixed(4)}`);
    }
    const coordStr = coordLines.join(', ');

    // Generate quad face indices with two-sided normal support
    const indexLines = [];
    const maxI = isClosed ? N : N - 1;
    for (let i = 0; i < maxI; i++) {
        const next = (i + 1) % N;
        // Quad face front: (bottom_i, bottom_next, top_next, top_i)
        indexLines.push(`${i}, ${next}, ${next + N}, ${i + N}, -1`);
        // Quad face reverse for 2-sided LiDAR collision
        indexLines.push(`${i}, ${i + N}, ${next + N}, ${next}, -1`);
    }

    const indexStr = indexLines.join(', ');

    return `Solid {
  name "${name}"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor ${colorStr}
        roughness 0.5
        metalness 0.1
      }
      geometry IndexedFaceSet {
        coord Coordinate {
          point [ ${coordStr} ]
        }
        coordIndex [ ${indexStr} ]
      }
    }
  ]
  boundingObject IndexedFaceSet {
    coord Coordinate {
      point [ ${coordStr} ]
    }
    coordIndex [ ${indexStr} ]
  }
}`;
}

function generateWebotsWBT() {
    if (trackBlocks.length === 0) {
        alert('Please build a track first before exporting to Webots!');
        return null;
    }

    const wallHeight = WALL_HEIGHT;
    const { leftWall, rightWall } = buildTrackWallPoints(0.04);

    // Compute bounding box for floor arena and camera viewpoint
    let minX = robotStartPose.x - 2.0;
    let maxX = robotStartPose.x + 2.0;
    let minY = robotStartPose.y - 2.0;
    let maxY = robotStartPose.y + 2.0;

    for (const p of leftWall) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
    }
    for (const p of rightWall) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
    }
    for (const obs of obstacles) {
        if (obs.type === 'rect') {
            minX = Math.min(minX, obs.x1, obs.x2);
            maxX = Math.max(maxX, obs.x1, obs.x2);
            minY = Math.min(minY, obs.y1, obs.y2);
            maxY = Math.max(maxY, obs.y1, obs.y2);
        } else if (obs.type === 'circle') {
            minX = Math.min(minX, obs.cx - obs.radius);
            maxX = Math.max(maxX, obs.cx + obs.radius);
            minY = Math.min(minY, obs.cy - obs.radius);
            maxY = Math.max(maxY, obs.cy + obs.radius);
        }
    }

    const spanX = Math.max(5.0, (maxX - minX));
    const spanY = Math.max(5.0, (maxY - minY));
    const midX = (minX + maxX) / 2.0;
    const midY = (minY + maxY) / 2.0;

    const arenaW = Math.max(16, Math.ceil(spanX + 8));
    const arenaH = Math.max(16, Math.ceil(spanY + 8));
    const viewDist = Math.max(7.0, Math.max(spanX, spanY) * 1.35);

    const yawRad = robotStartPose.yaw_deg * (Math.PI / 180.0);

    const lines = [
        '#VRML_SIM R2025a utf8',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"',
        '',
        'WorldInfo {',
        '  info [',
        '    "Da Winna Modular Race Track Studio World",',
        '    "Coordinate System: ENU (East-North-Up Standard, Z is Elevation)"',
        '  ]',
        '  title "Da Winna Track World"',
        '  basicTimeStep 10',
        '  FPS 60',
        '  coordinateSystem "ENU"',
        '}',
        'Viewpoint {',
        '  orientation 0.65 0.35 0.67 1.85',
        `  position ${(midX).toFixed(2)} ${(midY - viewDist * 0.55).toFixed(2)} ${(viewDist * 0.95).toFixed(2)}`,
        '}',
        'TexturedBackground {',
        '  texture "dusk"',
        '}',
        'TexturedBackgroundLight {',
        '  luminosity 1.2',
        '}',
        'RectangleArena {',
        `  floorSize ${arenaW} ${arenaH}`,
        '  floorTileSize 1 1',
        '  floorAppearance PBRAppearance {',
        '    baseColor 0.08 0.10 0.15',
        '    roughness 0.8',
        '  }',
        '}',
        ''
    ];

    // Left & Right Watertight Boundary Walls
    lines.push(buildIndexedFaceSetWall(leftWall, "track_left_wall", "0.95 0.15 0.35", wallHeight));
    lines.push(buildIndexedFaceSetWall(rightWall, "track_right_wall", "0.20 0.60 0.95", wallHeight));

    // Custom Obstacles (Pillars & Boxes)
    for (let idx = 0; idx < obstacles.length; idx++) {
        const obs = obstacles[idx];
        if (obs.type === 'circle') {
            lines.push(`Solid {
  name "slalom_pillar_${idx}"
  translation ${obs.cx.toFixed(4)} ${obs.cy.toFixed(4)} ${(wallHeight / 2.0).toFixed(4)}
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1.0 0.75 0.0
        roughness 0.25
        metalness 0.1
      }
      geometry Cylinder {
        height ${wallHeight.toFixed(4)}
        radius ${obs.radius.toFixed(4)}
      }
    }
  ]
  boundingObject Cylinder {
    height ${wallHeight.toFixed(4)}
    radius ${obs.radius.toFixed(4)}
  }
}`);
        } else if (obs.type === 'rect') {
            const cx = (obs.x1 + obs.x2) / 2.0;
            const cy = (obs.y1 + obs.y2) / 2.0;
            const w = Math.abs(obs.x2 - obs.x1);
            const h = Math.abs(obs.y2 - obs.y1);
            lines.push(`Solid {
  name "obstacle_box_${idx}"
  translation ${cx.toFixed(4)} ${cy.toFixed(4)} ${(wallHeight / 2.0).toFixed(4)}
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1.0 0.45 0.0
        roughness 0.35
      }
      geometry Box {
        size ${w.toFixed(4)} ${h.toFixed(4)} ${wallHeight.toFixed(4)}
      }
    }
  ]
  boundingObject Box {
    size ${w.toFixed(4)} ${h.toFixed(4)} ${wallHeight.toFixed(4)}
  }
}`);
        }
    }

    // Start / Finish Line Floor Visual
    if (startLine || trackBlocks.length > 0) {
        const sX = startLine ? startLine.x : trackBlocks[0].startX;
        const sY = startLine ? startLine.y : trackBlocks[0].startY;
        const sH = startLine ? startLine.heading : trackBlocks[0].startHeading;
        const sYaw = sH * (Math.PI / 2.0);

        lines.push(`Transform {
  translation ${sX.toFixed(4)} ${sY.toFixed(4)} 0.002
  rotation 0 0 1 ${sYaw.toFixed(4)}
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.0 1.0 0.55
        roughness 0.4
      }
      geometry Box {
        size 0.08 ${TRACK_WIDTH.toFixed(4)} 0.003
      }
    }
  ]
}`);
    }

    // Autonomous Robot with 2D Planar LiDAR (ENU Coordinate System)
    lines.push(`Robot {
  name "da_winna_robot"
  translation ${robotStartPose.x.toFixed(4)} ${robotStartPose.y.toFixed(4)} 0.05
  rotation 0 0 1 ${yawRad.toFixed(4)}
  children [
    # Golden Yellow Carbon Chassis
    Shape {
      appearance PBRAppearance {
        baseColor 1.0 0.85 0.0
        metalness 0.6
        roughness 0.2
      }
      geometry Box {
        size 0.20 0.15 0.05
      }
    }
    # Front Directional Indicator
    Transform {
      translation 0.09 0.0 0.015
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 1.0 0.0 0.45
            roughness 0.2
          }
          geometry Box {
            size 0.03 0.08 0.02
          }
        }
      ]
    }
    # MS200k 2D Planar LiDAR (Zero-Noise / Horizontal scan in XY plane)
    Solid {
      translation 0.0 0.0 0.04
      children [
        Lidar {
          name "ms200k_lidar"
          fieldOfView 6.2831853
          horizontalResolution 450
          numberOfLayers 1
          minRange 0.12
          maxRange 4.0
          noise 0.0
          type "fixed"
        }
      ]
    }
    InertialUnit {
      name "bno08x_imu"
      noise 0.001
    }
  ]
  controller "robot_lidar_collector"
  supervisor TRUE
}`);

    return lines.join('\n');
}

function exportWebotsWorld() {
    const wbtContent = generateWebotsWBT();
    if (!wbtContent) return;

    const blob = new Blob([wbtContent], { type: 'text/plain' });
    const link = document.createElement('a');
    link.download = `track_world_${Date.now()}.wbt`;
    link.href = URL.createObjectURL(blob);
    link.click();
    showToast('Exported Webots .wbt World (ENU Standard)', 'success');
}

async function deployMapToPi() {
    const grid = generateOccupancyGrid();
    if (!grid) return;

    const pngBase64 = grid.canvas.toDataURL('image/png').split(',')[1];
    const yawRad = (robotStartPose.yaw_deg * Math.PI / 180.0).toFixed(4);
    const yamlContent = `image: track_map.png
resolution: ${grid.resolution}
origin: [${grid.originX.toFixed(3)}, ${grid.originY.toFixed(3)}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# Initial Robot Spawn Pose (ROS 2 / Cartographer / Nav2 standard)
initial_pose:
  x: ${robotStartPose.x.toFixed(3)}
  y: ${robotStartPose.y.toFixed(3)}
  z: 0.0
  yaw: ${yawRad}
`;

    const payload = {
        png_b64: pngBase64,
        yaml: yamlContent,
        track_json: JSON.stringify({
            parameters: {
                tileSize: TILE_SIZE,
                trackWidth: TRACK_WIDTH,
                wallMargin: WALL_MARGIN
            },
            initial_pose: {
                x: robotStartPose.x,
                y: robotStartPose.y,
                yaw_deg: robotStartPose.yaw_deg,
                yaw_rad: robotStartPose.yaw_deg * Math.PI / 180.0
            },
            blocks: trackBlocks,
            obstacles: obstacles,
            startLine: startLine
        }, null, 2)
    };

    showToast('Deploying map & initial pose to Raspberry Pi 5...', 'success');

    try {
        const resp = await fetch('/api/deploy_to_pi', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });

        const res = await resp.json();
        if (res.status === 'ok') {
            showToast('✓ ' + res.message, 'success');
        } else {
            showToast('Error: ' + res.message, 'error');
        }
    } catch (err) {
        showToast('Deployment error: ' + err.message, 'error');
    }
}

function showToast(msg, type = 'success') {
    const toast = document.getElementById('toast');
    const toastMsg = document.getElementById('toast-message');
    const toastIcon = document.getElementById('toast-icon');

    toast.className = `toast show ${type}`;
    toastMsg.textContent = msg;
    toastIcon.textContent = type === 'success' ? '✓' : '⚠️';

    setTimeout(() => {
        toast.className = 'toast';
    }, 2800);
}

// Launch on DOM Load
window.addEventListener('DOMContentLoaded', init);
