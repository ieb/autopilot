/**
 * Training Data Viewer
 *
 * WebGL 3D visualization with playback controls for binary training data.
 * Reuses the Pogo 1250 boat model from the IMU testing tool.
 */

// =============================================================================
// Feature Indices (must match data_loader.py)
// =============================================================================

const F = {
    HEADING_ERROR: 0,
    MODE: 1,
    HEADING_RATE: 2,
    ROLL: 3,
    PITCH: 4,
    ROLL_RATE: 5,
    AWA: 6,
    AWA_RATE: 7,
    AWS: 8,
    TWA: 9,
    TWS: 10,
    STW: 11,
    SOG: 12,
    COG_ERROR: 13,
    RUDDER_POS: 14,
    RUDDER_VEL: 15,
    HEADING: 16,
    VMG_UP: 17,
    VMG_DOWN: 18,
    PD_SUGGESTION: 19,
    PLACEHOLDER: 20,
    WAVE_PERIOD: 21,
    LABEL: 22,  // expert rudder (the label column)
};

// Denormalization scales
const SCALE = {
    [F.HEADING_ERROR]: 90,
    [F.MODE]: 1,
    [F.HEADING_RATE]: 30,
    [F.ROLL]: 45,
    [F.PITCH]: 30,
    [F.ROLL_RATE]: 30,
    [F.AWA]: 180,
    [F.AWA_RATE]: 10,
    [F.AWS]: 60,
    [F.TWA]: 180,
    [F.TWS]: 60,
    [F.STW]: 25,
    [F.SOG]: 25,
    [F.COG_ERROR]: 45,
    [F.RUDDER_POS]: 25,
    [F.RUDDER_VEL]: 10,
    [F.HEADING]: 360,
    [F.VMG_UP]: 15,
    [F.VMG_DOWN]: 20,
    [F.PD_SUGGESTION]: 1,
    [F.PLACEHOLDER]: 1,
    [F.WAVE_PERIOD]: 15,
    [F.LABEL]: 25,
};

const COLS = 23;

// =============================================================================
// State
// =============================================================================

let allFrames = null;      // Float32Array of all frame data
let totalFrames = 0;

const playback = {
    playing: false,
    speed: 1.0,
    currentFrame: 0,
    frameDuration: 500,    // ms per frame at 1x (2 Hz)
    accumulator: 0,
    lastTimestamp: 0,
};

// WebGL
let glCanvas = null;
let gl = null;
let compassCanvas = null;
let compassCtx = null;
let animationId = null;
let shaderProgram = null;
let boatBuffers = null;
let riggingBuffers = null;
let rudderBuffers = null;
let axisBuffers = null;

// Camera
let cameraAzimuth = 315;
let cameraElevation = 25;

// Current frame values (denormalized)
const cur = {};

// =============================================================================
// WebGL Utilities
// =============================================================================

function degToRad(deg) {
    return deg * Math.PI / 180;
}

function createShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        console.error('Shader compile error:', gl.getShaderInfoLog(shader));
        gl.deleteShader(shader);
        return null;
    }
    return shader;
}

function createProgram(gl, vertexShader, fragmentShader) {
    const program = gl.createProgram();
    gl.attachShader(program, vertexShader);
    gl.attachShader(program, fragmentShader);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        console.error('Program link error:', gl.getProgramInfoLog(program));
        gl.deleteProgram(program);
        return null;
    }
    return program;
}

// =============================================================================
// Matrix Math (Column-major for WebGL)
// =============================================================================

function mat4Identity() {
    return new Float32Array([
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]);
}

function mat4Multiply(a, b) {
    const result = new Float32Array(16);
    for (let row = 0; row < 4; row++) {
        for (let col = 0; col < 4; col++) {
            result[col * 4 + row] =
                a[0 * 4 + row] * b[col * 4 + 0] +
                a[1 * 4 + row] * b[col * 4 + 1] +
                a[2 * 4 + row] * b[col * 4 + 2] +
                a[3 * 4 + row] * b[col * 4 + 3];
        }
    }
    return result;
}

function mat4RotateX(angle) {
    const c = Math.cos(angle), s = Math.sin(angle);
    return new Float32Array([
        1, 0, 0, 0,
        0, c, s, 0,
        0, -s, c, 0,
        0, 0, 0, 1
    ]);
}

function mat4RotateY(angle) {
    const c = Math.cos(angle), s = Math.sin(angle);
    return new Float32Array([
        c, 0, -s, 0,
        0, 1, 0, 0,
        s, 0, c, 0,
        0, 0, 0, 1
    ]);
}

function mat4RotateZ(angle) {
    const c = Math.cos(angle), s = Math.sin(angle);
    return new Float32Array([
        c, s, 0, 0,
        -s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]);
}

function mat4Perspective(fovY, aspect, near, far) {
    const f = 1.0 / Math.tan(fovY / 2);
    const rangeInv = 1 / (near - far);
    return new Float32Array([
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (near + far) * rangeInv, -1,
        0, 0, near * far * rangeInv * 2, 0
    ]);
}

function mat4Translate(x, y, z) {
    return new Float32Array([
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        x, y, z, 1
    ]);
}

function mat4LookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ, upX, upY, upZ) {
    let fx = targetX - eyeX, fy = targetY - eyeY, fz = targetZ - eyeZ;
    let fLen = Math.sqrt(fx * fx + fy * fy + fz * fz);
    fx /= fLen; fy /= fLen; fz /= fLen;

    let rx = fy * upZ - fz * upY;
    let ry = fz * upX - fx * upZ;
    let rz = fx * upY - fy * upX;
    let rLen = Math.sqrt(rx * rx + ry * ry + rz * rz);
    rx /= rLen; ry /= rLen; rz /= rLen;

    let ux = ry * fz - rz * fy;
    let uy = rz * fx - rx * fz;
    let uz = rx * fy - ry * fx;

    return new Float32Array([
        rx, ux, -fx, 0,
        ry, uy, -fy, 0,
        rz, uz, -fz, 0,
        -(rx * eyeX + ry * eyeY + rz * eyeZ),
        -(ux * eyeX + uy * eyeY + uz * eyeZ),
        (fx * eyeX + fy * eyeY + fz * eyeZ),
        1
    ]);
}

// =============================================================================
// Shaders
// =============================================================================

const vertexShaderSource = `
    attribute vec3 aPosition;
    attribute vec3 aColor;
    uniform mat4 uModelMatrix;
    uniform mat4 uViewMatrix;
    uniform mat4 uProjectionMatrix;
    varying vec3 vColor;
    void main() {
        gl_Position = uProjectionMatrix * uViewMatrix * uModelMatrix * vec4(aPosition, 1.0);
        vColor = aColor;
    }
`;

const fragmentShaderSource = `
    precision mediump float;
    varying vec3 vColor;
    void main() {
        gl_FragColor = vec4(vColor, 1.0);
    }
`;

// =============================================================================
// WebGL Initialization
// =============================================================================

function initWebGL() {
    glCanvas = document.getElementById('boat-canvas');
    gl = glCanvas.getContext('webgl') || glCanvas.getContext('experimental-webgl');
    if (!gl) {
        console.error('WebGL not supported');
        return false;
    }

    const vertexShader = createShader(gl, gl.VERTEX_SHADER, vertexShaderSource);
    const fragmentShader = createShader(gl, gl.FRAGMENT_SHADER, fragmentShaderSource);
    if (!vertexShader || !fragmentShader) return false;

    shaderProgram = createProgram(gl, vertexShader, fragmentShader);
    if (!shaderProgram) return false;

    shaderProgram.aPosition = gl.getAttribLocation(shaderProgram, 'aPosition');
    shaderProgram.aColor = gl.getAttribLocation(shaderProgram, 'aColor');
    shaderProgram.uModelMatrix = gl.getUniformLocation(shaderProgram, 'uModelMatrix');
    shaderProgram.uViewMatrix = gl.getUniformLocation(shaderProgram, 'uViewMatrix');
    shaderProgram.uProjectionMatrix = gl.getUniformLocation(shaderProgram, 'uProjectionMatrix');

    if (typeof createPogo1250Buffers !== 'function') {
        console.error('pogo1250.js not loaded');
        return false;
    }

    boatBuffers = createPogo1250Buffers(gl);
    riggingBuffers = createRiggingBuffers(gl);
    rudderBuffers = createRudderBuffers(gl);

    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    gl.clearColor(0.08, 0.08, 0.12, 1.0);  // dark background

    return true;
}

// =============================================================================
// Rudder Geometry
// =============================================================================

function createRudderBuffers(gl) {
    const S = 0.16;
    // Rudder blade: flat rectangle at stern, extending below waterline
    // Stern Y ~ +0.95 (just inside transom at Y=1.0)
    const y = 0.95;
    const hw = S * 0.08;    // half-width (thin blade)
    const top = S * 0.1;    // just below waterline
    const bot = -S * 1.2;   // extends below
    const depth = S * 0.5;  // fore-aft depth of blade

    // Rudder is drawn as a flat blade centered at (0, y, mid)
    // It will be rotated around Z axis (vertical) in the boat frame
    const positions = new Float32Array([
        // Front face
        -hw, y - depth/2, top,
         hw, y - depth/2, top,
         hw, y - depth/2, bot,
        -hw, y - depth/2, bot,
        // Back face
        -hw, y + depth/2, top,
         hw, y + depth/2, top,
         hw, y + depth/2, bot,
        -hw, y + depth/2, bot,
        // Top face
        -hw, y - depth/2, top,
         hw, y - depth/2, top,
         hw, y + depth/2, top,
        -hw, y + depth/2, top,
        // Bottom face
        -hw, y - depth/2, bot,
         hw, y - depth/2, bot,
         hw, y + depth/2, bot,
        -hw, y + depth/2, bot,
        // Left face
        -hw, y - depth/2, top,
        -hw, y + depth/2, top,
        -hw, y + depth/2, bot,
        -hw, y - depth/2, bot,
        // Right face
         hw, y - depth/2, top,
         hw, y + depth/2, top,
         hw, y + depth/2, bot,
         hw, y - depth/2, bot,
    ]);

    const RUDDER_COL = [0.75, 0.15, 0.15];  // dark red
    const colors = new Float32Array(24 * 3);
    for (let i = 0; i < 24; i++) {
        colors[i * 3] = RUDDER_COL[0];
        colors[i * 3 + 1] = RUDDER_COL[1];
        colors[i * 3 + 2] = RUDDER_COL[2];
    }

    const indices = new Uint16Array([
        0,1,2, 0,2,3,       // front
        4,6,5, 4,7,6,       // back
        8,9,10, 8,10,11,    // top
        12,14,13, 12,15,14, // bottom
        16,17,18, 16,18,19, // left
        20,22,21, 20,23,22, // right
    ]);

    const positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);

    const colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, colors, gl.STATIC_DRAW);

    const indexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indexBuffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, indices, gl.STATIC_DRAW);

    return {
        position: positionBuffer,
        color: colorBuffer,
        indices: indexBuffer,
        numIndices: indices.length,
        pivotY: y,  // pivot point for rotation
    };
}

function createAxisBuffers() {
    const len = 1.2;
    const positions = new Float32Array([
        0,0,0, len,0,0,  // X red
        0,0,0, 0,len,0,  // Y green
        0,0,0, 0,0,len,  // Z blue
    ]);
    const colors = new Float32Array([
        1.0,0.42,0.42, 1.0,0.42,0.42,
        0.29,0.87,0.5, 0.29,0.87,0.5,
        0.38,0.65,0.98, 0.38,0.65,0.98,
    ]);

    const positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);

    const colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, colors, gl.STATIC_DRAW);

    return { position: positionBuffer, color: colorBuffer, numVertices: 6 };
}

// =============================================================================
// 3D Rendering
// =============================================================================

function getFrame(index) {
    if (!allFrames || index < 0 || index >= totalFrames) return null;
    const offset = index * COLS;
    return allFrames.subarray(offset, offset + COLS);
}

function getRaw(feat) {
    if (!allFrames || totalFrames === 0) return 0;
    const frame = getFrame(playback.currentFrame);
    return frame ? frame[feat] : 0;
}

function getDenorm(feat) {
    return getRaw(feat) * (SCALE[feat] || 1);
}

function drawScene() {
    if (!gl || !shaderProgram) return;

    gl.viewport(0, 0, glCanvas.width, glCanvas.height);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(shaderProgram);

    const aspect = glCanvas.width / glCanvas.height;
    const projectionMatrix = mat4Perspective(degToRad(45), aspect, 0.1, 100.0);

    const distance = 5.5;
    const lookAtZ = 0.4;
    const azimuthRad = degToRad(cameraAzimuth);
    const elevationRad = degToRad(cameraElevation);

    const camX = distance * Math.cos(elevationRad) * Math.sin(azimuthRad);
    const camY = distance * Math.cos(elevationRad) * Math.cos(azimuthRad);
    const camZ = lookAtZ + distance * Math.sin(elevationRad);

    const viewMatrix = mat4LookAt(camX, camY, camZ, 0, 0, lookAtZ, 0, 0, 1);

    gl.uniformMatrix4fv(shaderProgram.uProjectionMatrix, false, projectionMatrix);
    gl.uniformMatrix4fv(shaderProgram.uViewMatrix, false, viewMatrix);

    // Get current orientation from frame data
    const heading = getDenorm(F.HEADING);
    const roll = getDenorm(F.ROLL);
    const pitch = getDenorm(F.PITCH);
    const rudderAngle = getDenorm(F.LABEL);

    // Build model matrix: pitch -> roll -> heading (same order as IMU tool)
    let modelMatrix = mat4Identity();
    modelMatrix = mat4Multiply(mat4RotateX(degToRad(-pitch)), modelMatrix);
    modelMatrix = mat4Multiply(mat4RotateY(degToRad(roll)), modelMatrix);
    modelMatrix = mat4Multiply(mat4RotateZ(degToRad(-heading)), modelMatrix);

    // Draw boat hull
    drawMesh(boatBuffers, modelMatrix);

    // Draw rigging
    drawLines(riggingBuffers, modelMatrix);

    // Draw rudder with its own rotation
    // Rudder rotates around the vertical axis (Z in world) at the stern pivot
    // Positive rudder = starboard deflection
    const pivotY = rudderBuffers.pivotY;
    let rudderMatrix = mat4Identity();
    // Translate to pivot, rotate around Z, translate back
    rudderMatrix = mat4Multiply(mat4Translate(0, -pivotY, 0), rudderMatrix);
    rudderMatrix = mat4Multiply(mat4RotateZ(degToRad(-rudderAngle)), rudderMatrix);
    rudderMatrix = mat4Multiply(mat4Translate(0, pivotY, 0), rudderMatrix);
    // Apply boat orientation on top
    rudderMatrix = mat4Multiply(modelMatrix, rudderMatrix);

    drawMesh(rudderBuffers, rudderMatrix);

    // Draw axes
    if (!axisBuffers) axisBuffers = createAxisBuffers();
    drawLines(axisBuffers, modelMatrix);
}

function drawMesh(buffers, modelMatrix) {
    gl.uniformMatrix4fv(shaderProgram.uModelMatrix, false, modelMatrix);

    gl.bindBuffer(gl.ARRAY_BUFFER, buffers.position);
    gl.enableVertexAttribArray(shaderProgram.aPosition);
    gl.vertexAttribPointer(shaderProgram.aPosition, 3, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, buffers.color);
    gl.enableVertexAttribArray(shaderProgram.aColor);
    gl.vertexAttribPointer(shaderProgram.aColor, 3, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, buffers.indices);
    gl.drawElements(gl.TRIANGLES, buffers.numIndices, gl.UNSIGNED_SHORT, 0);
}

function drawLines(buffers, modelMatrix) {
    gl.uniformMatrix4fv(shaderProgram.uModelMatrix, false, modelMatrix);

    gl.bindBuffer(gl.ARRAY_BUFFER, buffers.position);
    gl.enableVertexAttribArray(shaderProgram.aPosition);
    gl.vertexAttribPointer(shaderProgram.aPosition, 3, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, buffers.color);
    gl.enableVertexAttribArray(shaderProgram.aColor);
    gl.vertexAttribPointer(shaderProgram.aColor, 3, gl.FLOAT, false, 0, 0);

    gl.drawArrays(gl.LINES, 0, buffers.numVertices);
}

// =============================================================================
// Compass (2D Canvas) with Wind Arrows
// =============================================================================

function initCompass() {
    compassCanvas = document.getElementById('compass-canvas');
    compassCtx = compassCanvas.getContext('2d');
}

function drawCompass() {
    if (!compassCtx) return;

    const cx = compassCanvas.width / 2;
    const cy = compassCanvas.height / 2;
    const radius = Math.min(cx, cy) - 5;

    compassCtx.clearRect(0, 0, compassCanvas.width, compassCanvas.height);

    // Background circle
    compassCtx.beginPath();
    compassCtx.arc(cx, cy, radius, 0, Math.PI * 2);
    compassCtx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    compassCtx.fill();
    compassCtx.strokeStyle = '#5a7aaa';
    compassCtx.lineWidth = 1;
    compassCtx.stroke();

    // Cardinal directions
    const dirs = [
        { label: 'N', angle: 0 },
        { label: 'E', angle: 90 },
        { label: 'S', angle: 180 },
        { label: 'W', angle: 270 }
    ];

    compassCtx.font = '9px sans-serif';
    compassCtx.textAlign = 'center';
    compassCtx.textBaseline = 'middle';

    dirs.forEach(d => {
        const a = degToRad(d.angle - cameraAzimuth - 90);
        const x = cx + Math.cos(a) * (radius - 9);
        const y = cy + Math.sin(a) * (radius - 9);
        compassCtx.fillStyle = d.label === 'N' ? '#e94560' : '#888';
        compassCtx.fillText(d.label, x, y);
    });

    // Heading arrow (red)
    const heading = getDenorm(F.HEADING);
    const headingRad = degToRad(heading - cameraAzimuth);

    compassCtx.save();
    compassCtx.translate(cx, cy);
    compassCtx.rotate(headingRad);

    compassCtx.beginPath();
    compassCtx.moveTo(0, -radius + 16);
    compassCtx.lineTo(-5, 4);
    compassCtx.lineTo(5, 4);
    compassCtx.closePath();
    compassCtx.fillStyle = '#e94560';
    compassCtx.fill();
    compassCtx.restore();

    // AWA arrow (green) — relative to boat heading
    const awa = getDenorm(F.AWA);
    drawWindArrow(cx, cy, radius, heading + awa, '#4ade80', 'AWA');

    // TWA arrow (blue) — relative to boat heading
    const twa = getDenorm(F.TWA);
    drawWindArrow(cx, cy, radius, heading + twa, '#60a5fa', 'TWA');
}

function drawWindArrow(cx, cy, radius, angle, color, label) {
    // angle is absolute (heading + wind angle)
    const rad = degToRad(angle - cameraAzimuth);

    compassCtx.save();
    compassCtx.translate(cx, cy);
    compassCtx.rotate(rad);

    // Small triangle at edge of compass, pointing inward
    const r = radius - 2;
    compassCtx.beginPath();
    compassCtx.moveTo(0, -r);
    compassCtx.lineTo(-3, -r + 8);
    compassCtx.lineTo(3, -r + 8);
    compassCtx.closePath();
    compassCtx.fillStyle = color;
    compassCtx.fill();

    compassCtx.restore();
}

// =============================================================================
// Data Loading
// =============================================================================

async function loadFileList() {
    try {
        const resp = await fetch('/api/files');
        const data = await resp.json();

        const sel = document.getElementById('file-select');
        sel.innerHTML = '';

        if (!data.files || data.files.length === 0) {
            sel.innerHTML = '<option value="">No .bin files found</option>';
            setStatus('No .bin files found in data directory', true);
            return;
        }

        const placeholder = document.createElement('option');
        placeholder.value = '';
        placeholder.textContent = `-- Select file (${data.files.length} available) --`;
        sel.appendChild(placeholder);

        for (const f of data.files) {
            const opt = document.createElement('option');
            opt.value = f.name;
            const dur = f.duration >= 60
                ? `${Math.floor(f.duration / 60)}m${Math.round(f.duration % 60)}s`
                : `${f.duration}s`;
            opt.textContent = `${f.name} (${f.frames} frames, ${dur})`;
            sel.appendChild(opt);
        }

        setStatus('Select a file to begin');
    } catch (e) {
        setStatus('Failed to load file list: ' + e.message, true);
    }
}

async function loadFile(filename) {
    if (!filename) return;

    setStatus('Loading ' + filename + '...');
    try {
        const resp = await fetch('/api/data/' + encodeURIComponent(filename));
        if (!resp.ok) {
            const err = await resp.json();
            setStatus('Error: ' + (err.error || 'Unknown'), true);
            return;
        }

        const cols = parseInt(resp.headers.get('X-Cols')) || COLS;
        const frameCount = parseInt(resp.headers.get('X-Frame-Count')) || 0;

        const buffer = await resp.arrayBuffer();
        allFrames = new Float32Array(buffer);
        totalFrames = allFrames.length / cols;

        // Reset playback
        playback.currentFrame = 0;
        playback.playing = false;
        playback.accumulator = 0;
        updatePlayButton();

        // Update scrub bar
        const scrub = document.getElementById('scrub');
        scrub.max = Math.max(0, totalFrames - 1);
        scrub.value = 0;

        updateFrameInfo();
        updateDataPanels();

        const dur = totalFrames / 2.0;
        const durStr = dur >= 60
            ? `${Math.floor(dur / 60)}m${Math.round(dur % 60)}s`
            : `${dur.toFixed(1)}s`;
        setStatus(`Loaded ${filename}: ${totalFrames} frames, ${durStr}`);

    } catch (e) {
        setStatus('Failed to load file: ' + e.message, true);
    }
}

// =============================================================================
// Playback Engine
// =============================================================================

function animate(timestamp) {
    if (playback.playing && totalFrames > 0) {
        if (playback.lastTimestamp > 0) {
            const dt = timestamp - playback.lastTimestamp;
            playback.accumulator += dt * playback.speed;

            while (playback.accumulator >= playback.frameDuration) {
                playback.accumulator -= playback.frameDuration;
                playback.currentFrame++;
                if (playback.currentFrame >= totalFrames) {
                    playback.currentFrame = totalFrames - 1;
                    playback.playing = false;
                    updatePlayButton();
                    break;
                }
            }
        }
        playback.lastTimestamp = timestamp;

        document.getElementById('scrub').value = playback.currentFrame;
        updateFrameInfo();
        updateDataPanels();
    }

    drawScene();
    drawCompass();
    animationId = requestAnimationFrame(animate);
}

function togglePlay() {
    if (totalFrames === 0) return;
    playback.playing = !playback.playing;
    if (playback.playing) {
        playback.lastTimestamp = 0;
        playback.accumulator = 0;
        // If at end, wrap to start
        if (playback.currentFrame >= totalFrames - 1) {
            playback.currentFrame = 0;
        }
    }
    updatePlayButton();
}

function stepBack() {
    if (playback.currentFrame > 0) {
        playback.playing = false;
        playback.currentFrame--;
        updatePlayButton();
        syncUI();
    }
}

function stepFwd() {
    if (playback.currentFrame < totalFrames - 1) {
        playback.playing = false;
        playback.currentFrame++;
        updatePlayButton();
        syncUI();
    }
}

function jumpStart() {
    playback.playing = false;
    playback.currentFrame = 0;
    updatePlayButton();
    syncUI();
}

function jumpEnd() {
    playback.playing = false;
    playback.currentFrame = Math.max(0, totalFrames - 1);
    updatePlayButton();
    syncUI();
}

function setSpeed(speed) {
    playback.speed = speed;
    document.querySelectorAll('.speed-btn').forEach(btn => {
        btn.classList.toggle('active', parseFloat(btn.dataset.speed) === speed);
    });
}

function updatePlayButton() {
    const btn = document.getElementById('btn-play');
    btn.innerHTML = playback.playing ? '&#10074;&#10074;' : '&#9654;';
    btn.classList.toggle('active', playback.playing);
}

function syncUI() {
    document.getElementById('scrub').value = playback.currentFrame;
    updateFrameInfo();
    updateDataPanels();
}

// =============================================================================
// UI Updates
// =============================================================================

function updateFrameInfo() {
    const frame = playback.currentFrame;
    document.getElementById('frame-info').textContent =
        `Frame ${frame + 1} / ${totalFrames}`;

    const totalSec = frame / 2.0;
    const min = Math.floor(totalSec / 60);
    const sec = (totalSec % 60).toFixed(1);
    document.getElementById('time-info').textContent =
        `${min}:${sec.padStart(4, '0')}`;
}

function updateDataPanels() {
    if (totalFrames === 0) return;

    // Navigation
    setVal('val-heading', getDenorm(F.HEADING), '\u00B0', 1);
    setVal('val-heading-error', getDenorm(F.HEADING_ERROR), '\u00B0', 1, true);
    setVal('val-heading-rate', getDenorm(F.HEADING_RATE), '\u00B0/s', 1, true);
    setMode('val-mode', getRaw(F.MODE));
    setVal('val-cog-error', getDenorm(F.COG_ERROR), '\u00B0', 1, true);

    // Wind
    setVal('val-awa', getDenorm(F.AWA), '\u00B0', 1, true);
    setVal('val-aws', getDenorm(F.AWS), ' kts', 1);
    setVal('val-twa', getDenorm(F.TWA), '\u00B0', 1, true);
    setVal('val-tws', getDenorm(F.TWS), ' kts', 1);
    setVal('val-awa-rate', getDenorm(F.AWA_RATE), '\u00B0/s', 1, true);

    // Speed
    setVal('val-stw', getDenorm(F.STW), ' kts', 1);
    setVal('val-sog', getDenorm(F.SOG), ' kts', 1);
    setVal('val-vmg-up', getDenorm(F.VMG_UP), ' kts', 1);
    setVal('val-vmg-down', getDenorm(F.VMG_DOWN), ' kts', 1);

    // Steering
    setVal('val-rudder', getDenorm(F.LABEL), '\u00B0', 1, true);
    setVal('val-pd', getRaw(F.PD_SUGGESTION), '', 3, true);
    setVal('val-rudder-vel', getDenorm(F.RUDDER_VEL), '\u00B0/s', 1, true);

    // Environment
    setVal('val-roll', getDenorm(F.ROLL), '\u00B0', 1, true);
    setVal('val-pitch', getDenorm(F.PITCH), '\u00B0', 1, true);
    setVal('val-roll-rate', getDenorm(F.ROLL_RATE), '\u00B0/s', 1, true);
    setVal('val-wave-period', getDenorm(F.WAVE_PERIOD), ' s', 1);
}

function setVal(id, value, suffix, decimals, signed) {
    const el = document.getElementById(id);
    if (!el) return;
    const text = value.toFixed(decimals) + suffix;
    el.textContent = text;
    if (signed) {
        el.className = 'value ' + (value > 0.05 ? 'positive' : (value < -0.05 ? 'negative' : ''));
    }
}

function setMode(id, raw) {
    const el = document.getElementById(id);
    if (!el) return;
    if (raw < 0.25) {
        el.innerHTML = '<span class="mode-badge mode-compass">COMPASS</span>';
    } else if (raw < 0.75) {
        el.innerHTML = '<span class="mode-badge mode-awa">AWA</span>';
    } else {
        el.innerHTML = '<span class="mode-badge mode-twa">TWA</span>';
    }
}

function setStatus(msg, isError) {
    const el = document.getElementById('status-bar');
    el.textContent = msg;
    el.className = 'status-bar' + (isError ? ' error' : '');
}

function azimuthToCardinal(deg) {
    const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'N'];
    return directions[Math.round(deg / 45) % 8];
}

// =============================================================================
// Initialization
// =============================================================================

function init() {
    if (!initWebGL()) {
        setStatus('WebGL initialization failed', true);
        return;
    }

    initCompass();

    // File selector
    document.getElementById('file-select').addEventListener('change', (e) => {
        loadFile(e.target.value);
    });

    // Transport controls
    document.getElementById('btn-start').addEventListener('click', jumpStart);
    document.getElementById('btn-back').addEventListener('click', stepBack);
    document.getElementById('btn-play').addEventListener('click', togglePlay);
    document.getElementById('btn-fwd').addEventListener('click', stepFwd);
    document.getElementById('btn-end').addEventListener('click', jumpEnd);

    // Scrub bar
    const scrub = document.getElementById('scrub');
    scrub.addEventListener('input', (e) => {
        playback.currentFrame = parseInt(e.target.value);
        playback.playing = false;
        updatePlayButton();
        updateFrameInfo();
        updateDataPanels();
    });

    // Speed buttons
    document.querySelectorAll('.speed-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            setSpeed(parseFloat(btn.dataset.speed));
        });
    });

    // Camera controls
    const azSlider = document.getElementById('view-azimuth');
    const azValue = document.getElementById('view-azimuth-value');
    azSlider.addEventListener('input', (e) => {
        cameraAzimuth = parseInt(e.target.value);
        azValue.textContent = azimuthToCardinal(cameraAzimuth);
    });
    azValue.textContent = azimuthToCardinal(cameraAzimuth);
    azSlider.value = cameraAzimuth;

    const elSlider = document.getElementById('view-elevation');
    const elValue = document.getElementById('view-elevation-value');
    elSlider.addEventListener('input', (e) => {
        cameraElevation = parseInt(e.target.value);
        elValue.textContent = cameraElevation + '\u00B0';
    });
    elValue.textContent = cameraElevation + '\u00B0';
    elSlider.value = cameraElevation;

    // Keyboard shortcuts
    document.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

        switch (e.code) {
            case 'Space':
                e.preventDefault();
                togglePlay();
                break;
            case 'ArrowLeft':
                e.preventDefault();
                stepBack();
                break;
            case 'ArrowRight':
                e.preventDefault();
                stepFwd();
                break;
            case 'Home':
                e.preventDefault();
                jumpStart();
                break;
            case 'End':
                e.preventDefault();
                jumpEnd();
                break;
        }
    });

    // Load file list
    loadFileList();

    // Start render loop
    animationId = requestAnimationFrame(animate);
}

document.addEventListener('DOMContentLoaded', init);
