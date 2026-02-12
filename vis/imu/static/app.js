/**
 * BNO055 IMU Testing Tool
 * 
 * WebGL-based 3D visualization for real-time IMU orientation display.
 */

// =============================================================================
// State
// =============================================================================

const state = {
    connected: false,
    heading: 0,
    pitch: 0,
    roll: 0,
    yawRate: 0,
    pitchRate: 0,
    rollRate: 0,
    accelX: 0,
    accelY: 0,
    accelZ: 0,
    calSys: 0,
    calGyro: 0,
    calAccel: 0,
    calMag: 0,
    isCalibrated: false,
    messageCount: 0,
    errorCount: 0,
    lastUpdate: 0,
    updateCount: 0,
    dataRate: 0
};

let eventSource = null;
let glCanvas = null;
let gl = null;
let compassCanvas = null;
let compassCtx = null;
let animationId = null;
let shaderProgram = null;
let cubeBuffers = null;

// Camera view settings
let cameraAzimuth = 315;  // Initial view from NW (315 degrees)
let cameraElevation = 35; // Degrees above horizon

// =============================================================================
// WebGL Utilities
// =============================================================================

function degToRad(deg) {
    return deg * Math.PI / 180;
}

/**
 * Create a shader from source
 */
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

/**
 * Create shader program from vertex and fragment shaders
 */
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
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return new Float32Array([
        1, 0, 0, 0,
        0, c, s, 0,
        0, -s, c, 0,
        0, 0, 0, 1
    ]);
}

function mat4RotateY(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return new Float32Array([
        c, 0, -s, 0,
        0, 1, 0, 0,
        s, 0, c, 0,
        0, 0, 0, 1
    ]);
}

function mat4RotateZ(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
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

// =============================================================================
// WebGL Initialization
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

/**
 * Initialize WebGL context and shaders
 */
function initWebGL() {
    glCanvas = document.getElementById('imu-canvas');
    gl = glCanvas.getContext('webgl') || glCanvas.getContext('experimental-webgl');
    
    if (!gl) {
        console.error('WebGL not supported');
        return false;
    }
    
    // Create shaders
    const vertexShader = createShader(gl, gl.VERTEX_SHADER, vertexShaderSource);
    const fragmentShader = createShader(gl, gl.FRAGMENT_SHADER, fragmentShaderSource);
    
    if (!vertexShader || !fragmentShader) {
        return false;
    }
    
    // Create program
    shaderProgram = createProgram(gl, vertexShader, fragmentShader);
    if (!shaderProgram) {
        return false;
    }
    
    // Get attribute and uniform locations
    shaderProgram.aPosition = gl.getAttribLocation(shaderProgram, 'aPosition');
    shaderProgram.aColor = gl.getAttribLocation(shaderProgram, 'aColor');
    shaderProgram.uModelMatrix = gl.getUniformLocation(shaderProgram, 'uModelMatrix');
    shaderProgram.uViewMatrix = gl.getUniformLocation(shaderProgram, 'uViewMatrix');
    shaderProgram.uProjectionMatrix = gl.getUniformLocation(shaderProgram, 'uProjectionMatrix');
    
    // Create cube buffers
    cubeBuffers = createCubeBuffers();
    
    // Enable depth testing
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    
    // Set clear color
    gl.clearColor(0.1, 0.1, 0.18, 1.0);
    
    return true;
}

/**
 * Create buffers for the cube geometry
 */
function createCubeBuffers() {
    // Cube dimensions (representing the IMU board)
    const w = 0.8;  // width (X - forward/back)
    const h = 0.5;  // height (Y - left/right)
    const d = 0.15; // depth (Z - up/down)
    
    // Vertices for each face (each face has 4 vertices, 6 faces = 24 vertices)
    // Position: x, y, z
    const positions = new Float32Array([
        // Top face (Z+) - Blue deck
        -w, -h,  d,   w, -h,  d,   w,  h,  d,  -w,  h,  d,
        // Bottom face (Z-) - Dark
        -w, -h, -d,  -w,  h, -d,   w,  h, -d,   w, -h, -d,
        // Front face (Y-) - Bow, Red
        -w, -h, -d,   w, -h, -d,   w, -h,  d,  -w, -h,  d,
        // Back face (Y+) - Stern, Dark blue
        -w,  h, -d,  -w,  h,  d,   w,  h,  d,   w,  h, -d,
        // Right face (X+) - Starboard, Green
         w, -h, -d,   w,  h, -d,   w,  h,  d,   w, -h,  d,
        // Left face (X-) - Port, Red/purple
        -w, -h, -d,  -w, -h,  d,  -w,  h,  d,  -w,  h, -d,
    ]);
    
    // Colors for each vertex (RGB)
    const colors = new Float32Array([
        // Top - Blue deck
        0.29, 0.56, 0.76,  0.29, 0.56, 0.76,  0.29, 0.56, 0.76,  0.29, 0.56, 0.76,
        // Bottom - Dark
        0.1, 0.15, 0.2,  0.1, 0.15, 0.2,  0.1, 0.15, 0.2,  0.1, 0.15, 0.2,
        // Front (bow) - Red
        0.76, 0.31, 0.31,  0.76, 0.31, 0.31,  0.76, 0.31, 0.31,  0.76, 0.31, 0.31,
        // Back (stern) - Dark blue
        0.16, 0.29, 0.42,  0.16, 0.29, 0.42,  0.16, 0.29, 0.42,  0.16, 0.29, 0.42,
        // Right (starboard) - Green
        0.23, 0.48, 0.35,  0.23, 0.48, 0.35,  0.23, 0.48, 0.35,  0.23, 0.48, 0.35,
        // Left (port) - Red/purple
        0.54, 0.23, 0.35,  0.54, 0.23, 0.35,  0.54, 0.23, 0.35,  0.54, 0.23, 0.35,
    ]);
    
    // Indices for triangles (2 triangles per face, 6 faces = 12 triangles = 36 indices)
    const indices = new Uint16Array([
        0, 1, 2,    0, 2, 3,    // Top
        4, 5, 6,    4, 6, 7,    // Bottom
        8, 9, 10,   8, 10, 11,  // Front
        12, 13, 14, 12, 14, 15, // Back
        16, 17, 18, 16, 18, 19, // Right
        20, 21, 22, 20, 22, 23, // Left
    ]);
    
    // Create position buffer
    const positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);
    
    // Create color buffer
    const colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, colors, gl.STATIC_DRAW);
    
    // Create index buffer
    const indexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indexBuffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, indices, gl.STATIC_DRAW);
    
    return {
        position: positionBuffer,
        color: colorBuffer,
        indices: indexBuffer,
        numIndices: indices.length
    };
}

/**
 * Create buffers for axis lines
 */
function createAxisBuffers() {
    const len = 1.2;
    
    // Axis line vertices: origin to tip for X, Y, Z
    const positions = new Float32Array([
        // X axis (forward) - Red
        0, 0, 0,  len, 0, 0,
        // Y axis (right) - Green  
        0, 0, 0,  0, len, 0,
        // Z axis (up) - Blue
        0, 0, 0,  0, 0, len,
    ]);
    
    const colors = new Float32Array([
        // X - Red
        1.0, 0.42, 0.42,  1.0, 0.42, 0.42,
        // Y - Green
        0.29, 0.87, 0.5,  0.29, 0.87, 0.5,
        // Z - Blue
        0.38, 0.65, 0.98,  0.38, 0.65, 0.98,
    ]);
    
    const positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);
    
    const colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, colors, gl.STATIC_DRAW);
    
    return {
        position: positionBuffer,
        color: colorBuffer,
        numVertices: 6
    };
}

let axisBuffers = null;

// =============================================================================
// 3D Rendering
// =============================================================================

/**
 * Create a look-at view matrix
 */
function mat4LookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ, upX, upY, upZ) {
    // Calculate forward vector (from eye to target)
    let fx = targetX - eyeX;
    let fy = targetY - eyeY;
    let fz = targetZ - eyeZ;
    
    // Normalize forward
    let fLen = Math.sqrt(fx * fx + fy * fy + fz * fz);
    fx /= fLen; fy /= fLen; fz /= fLen;
    
    // Calculate right vector (forward x up)
    let rx = fy * upZ - fz * upY;
    let ry = fz * upX - fx * upZ;
    let rz = fx * upY - fy * upX;
    
    // Normalize right
    let rLen = Math.sqrt(rx * rx + ry * ry + rz * rz);
    rx /= rLen; ry /= rLen; rz /= rLen;
    
    // Calculate true up (right x forward)
    let ux = ry * fz - rz * fy;
    let uy = rz * fx - rx * fz;
    let uz = rx * fy - ry * fx;
    
    // Build rotation matrix (transpose of orientation)
    // Then translate by -eye position
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

/**
 * Draw the 3D scene
 */
function drawScene() {
    if (!gl || !shaderProgram) return;
    
    // Set viewport
    gl.viewport(0, 0, glCanvas.width, glCanvas.height);
    
    // Clear
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    
    // Use shader program
    gl.useProgram(shaderProgram);
    
    // Create projection matrix (perspective)
    const aspect = glCanvas.width / glCanvas.height;
    const projectionMatrix = mat4Perspective(degToRad(45), aspect, 0.1, 100.0);
    
    // Create view matrix using proper look-at
    // Camera orbits around origin at given azimuth and elevation
    const distance = 3.0;
    const azimuthRad = degToRad(cameraAzimuth);
    const elevationRad = degToRad(cameraElevation);
    
    // Camera position in spherical coordinates (Y is forward, Z is up)
    const camX = distance * Math.cos(elevationRad) * Math.sin(azimuthRad);
    const camY = distance * Math.cos(elevationRad) * Math.cos(azimuthRad);
    const camZ = distance * Math.sin(elevationRad);
    
    // Look at origin, Z is up
    const viewMatrix = mat4LookAt(camX, camY, camZ, 0, 0, 0, 0, 0, 1);
    
    // Create model matrix from IMU orientation
    // BNO055 uses: Heading (yaw around Z), Pitch (around X), Roll (around Y)
    let modelMatrix = mat4Identity();
    
    // The cube geometry has bow at -Y, but we want heading 0 to point +Y (North)
    // So we need a 180° base rotation, then apply heading
    // Combined: rotate by (180 - heading) degrees
    // Heading increases CW when viewed from above, RotateZ positive is CCW,
    // so we negate to get CW rotation
    modelMatrix = mat4Multiply(mat4RotateZ(degToRad(180 - state.heading)), modelMatrix);
    // BNO055: Roll is around fore-aft axis (Y), Pitch is around lateral axis (X)
    // Swapped to match BNO055 convention, roll sign inverted
    modelMatrix = mat4Multiply(mat4RotateX(degToRad(state.roll)), modelMatrix);
    modelMatrix = mat4Multiply(mat4RotateY(degToRad(state.pitch)), modelMatrix);
    
    // Set uniforms
    gl.uniformMatrix4fv(shaderProgram.uProjectionMatrix, false, projectionMatrix);
    gl.uniformMatrix4fv(shaderProgram.uViewMatrix, false, viewMatrix);
    gl.uniformMatrix4fv(shaderProgram.uModelMatrix, false, modelMatrix);
    
    // Draw cube
    drawCube(modelMatrix);
    
    // Draw axes
    drawAxes(modelMatrix);
}

/**
 * Draw the cube
 */
function drawCube(modelMatrix) {
    gl.uniformMatrix4fv(shaderProgram.uModelMatrix, false, modelMatrix);
    
    // Bind position buffer
    gl.bindBuffer(gl.ARRAY_BUFFER, cubeBuffers.position);
    gl.enableVertexAttribArray(shaderProgram.aPosition);
    gl.vertexAttribPointer(shaderProgram.aPosition, 3, gl.FLOAT, false, 0, 0);
    
    // Bind color buffer
    gl.bindBuffer(gl.ARRAY_BUFFER, cubeBuffers.color);
    gl.enableVertexAttribArray(shaderProgram.aColor);
    gl.vertexAttribPointer(shaderProgram.aColor, 3, gl.FLOAT, false, 0, 0);
    
    // Bind index buffer and draw
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, cubeBuffers.indices);
    gl.drawElements(gl.TRIANGLES, cubeBuffers.numIndices, gl.UNSIGNED_SHORT, 0);
}

/**
 * Draw axis lines
 */
function drawAxes(modelMatrix) {
    if (!axisBuffers) {
        axisBuffers = createAxisBuffers();
    }
    
    gl.uniformMatrix4fv(shaderProgram.uModelMatrix, false, modelMatrix);
    
    // Bind position buffer
    gl.bindBuffer(gl.ARRAY_BUFFER, axisBuffers.position);
    gl.enableVertexAttribArray(shaderProgram.aPosition);
    gl.vertexAttribPointer(shaderProgram.aPosition, 3, gl.FLOAT, false, 0, 0);
    
    // Bind color buffer
    gl.bindBuffer(gl.ARRAY_BUFFER, axisBuffers.color);
    gl.enableVertexAttribArray(shaderProgram.aColor);
    gl.vertexAttribPointer(shaderProgram.aColor, 3, gl.FLOAT, false, 0, 0);
    
    // Draw lines
    gl.drawArrays(gl.LINES, 0, axisBuffers.numVertices);
}

// =============================================================================
// Compass (2D Canvas)
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
    
    // Clear
    compassCtx.clearRect(0, 0, compassCanvas.width, compassCanvas.height);
    
    // Background circle
    compassCtx.beginPath();
    compassCtx.arc(cx, cy, radius, 0, Math.PI * 2);
    compassCtx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    compassCtx.fill();
    compassCtx.strokeStyle = '#5a7aaa';
    compassCtx.lineWidth = 1;
    compassCtx.stroke();
    
    // Cardinal directions (rotated with view)
    const dirs = [
        { label: 'N', angle: 0 },
        { label: 'E', angle: 90 },
        { label: 'S', angle: 180 },
        { label: 'W', angle: 270 }
    ];
    
    compassCtx.font = '10px sans-serif';
    compassCtx.textAlign = 'center';
    compassCtx.textBaseline = 'middle';
    
    dirs.forEach(d => {
        // Position cardinal labels using polar coordinates
        // In canvas: cos/sin give (right, down), so subtract 90° to put 0° at top
        const a = degToRad(d.angle - cameraAzimuth - 90);
        const x = cx + Math.cos(a) * (radius - 10);
        const y = cy + Math.sin(a) * (radius - 10);
        compassCtx.fillStyle = d.label === 'N' ? '#e94560' : '#888';
        compassCtx.fillText(d.label, x, y);
    });
    
    // Heading indicator (arrow showing where IMU is pointing)
    // Arrow is drawn pointing up, canvas rotate() rotates CW for positive angles
    // Heading 0 (North) should point up (no rotation), Heading 90 (East) should rotate 90° CW
    const headingRad = degToRad(state.heading - cameraAzimuth);
    
    compassCtx.save();
    compassCtx.translate(cx, cy);
    compassCtx.rotate(headingRad);
    
    // Draw arrow
    compassCtx.beginPath();
    compassCtx.moveTo(0, -radius + 18);
    compassCtx.lineTo(-6, 5);
    compassCtx.lineTo(6, 5);
    compassCtx.closePath();
    compassCtx.fillStyle = '#e94560';
    compassCtx.fill();
    
    compassCtx.restore();
}

// =============================================================================
// Animation Loop
// =============================================================================

function animate() {
    drawScene();
    drawCompass();
    animationId = requestAnimationFrame(animate);
}

// =============================================================================
// SSE Client
// =============================================================================

function connectSSE() {
    if (eventSource) {
        eventSource.close();
    }
    
    eventSource = new EventSource('/stream');
    
    eventSource.onopen = () => {
        console.log('SSE connected');
        updateConnectionStatus(true);
    };
    
    eventSource.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            handleIMUData(data);
        } catch (e) {
            console.error('Failed to parse SSE data:', e);
        }
    };
    
    eventSource.onerror = (error) => {
        console.error('SSE error:', error);
        updateConnectionStatus(false);
        
        setTimeout(() => {
            if (eventSource.readyState === EventSource.CLOSED) {
                connectSSE();
            }
        }, 2000);
    };
}

function handleIMUData(data) {
    if (!data.connected) {
        updateConnectionStatus(false, data.error);
        return;
    }
    
    updateConnectionStatus(true);
    
    // Update state
    state.heading = data.heading;
    state.pitch = data.pitch;
    state.roll = data.roll;
    state.yawRate = data.yaw_rate;
    state.pitchRate = data.pitch_rate;
    state.rollRate = data.roll_rate;
    state.accelX = data.accel_x;
    state.accelY = data.accel_y;
    state.accelZ = data.accel_z;
    state.calSys = data.cal_sys;
    state.calGyro = data.cal_gyro;
    state.calAccel = data.cal_accel;
    state.calMag = data.cal_mag;
    state.isCalibrated = data.is_calibrated;
    state.messageCount = data.message_count;
    state.errorCount = data.error_count;
    
    // Calculate data rate
    const now = Date.now();
    state.updateCount++;
    if (now - state.lastUpdate >= 1000) {
        state.dataRate = state.updateCount;
        state.updateCount = 0;
        state.lastUpdate = now;
    }
    
    // Update UI
    updateOrientationDisplay();
    updateCalibrationDisplay();
    updateDataDisplay();
}

// =============================================================================
// UI Updates
// =============================================================================

function updateConnectionStatus(connected, error = null) {
    state.connected = connected;
    const statusEl = document.getElementById('connection-status');
    const textEl = statusEl.querySelector('.text');
    
    if (connected) {
        statusEl.classList.remove('disconnected');
        statusEl.classList.add('connected');
        textEl.textContent = 'Connected';
    } else {
        statusEl.classList.remove('connected');
        statusEl.classList.add('disconnected');
        textEl.textContent = error || 'Disconnected';
    }
}

function updateOrientationDisplay() {
    document.getElementById('heading-value').textContent = state.heading.toFixed(1) + '\u00B0';
    document.getElementById('pitch-value').textContent = state.pitch.toFixed(1) + '\u00B0';
    document.getElementById('roll-value').textContent = state.roll.toFixed(1) + '\u00B0';
}

function updateCalibrationDisplay() {
    updateCalBars('cal-sys', state.calSys);
    updateCalBars('cal-gyro', state.calGyro);
    updateCalBars('cal-accel', state.calAccel);
    updateCalBars('cal-mag', state.calMag);
    
    document.getElementById('cal-sys-value').textContent = state.calSys + '/3';
    document.getElementById('cal-gyro-value').textContent = state.calGyro + '/3';
    document.getElementById('cal-accel-value').textContent = state.calAccel + '/3';
    document.getElementById('cal-mag-value').textContent = state.calMag + '/3';
    
    const hintEl = document.getElementById('calibration-hint');
    if (state.isCalibrated) {
        hintEl.textContent = 'Fully calibrated! You can save the calibration data.';
    } else {
        const hints = [];
        if (state.calGyro < 3) hints.push('Gyro: Keep sensor still');
        if (state.calAccel < 3) hints.push('Accel: Rotate to 6 positions');
        if (state.calMag < 3) hints.push('Mag: Wave in figure-8');
        hintEl.textContent = hints.join(' | ') || 'Calibrating...';
    }
}

function updateCalBars(id, level) {
    const container = document.getElementById(id);
    container.className = 'cal-bars level-' + level;
}

function updateDataDisplay() {
    document.getElementById('yaw-rate-value').textContent = state.yawRate.toFixed(2);
    document.getElementById('pitch-rate-value').textContent = state.pitchRate.toFixed(2);
    document.getElementById('roll-rate-value').textContent = state.rollRate.toFixed(2);
    
    document.getElementById('accel-x-value').textContent = state.accelX.toFixed(3);
    document.getElementById('accel-y-value').textContent = state.accelY.toFixed(3);
    document.getElementById('accel-z-value').textContent = state.accelZ.toFixed(3);
    
    document.getElementById('data-rate-value').textContent = state.dataRate + ' Hz';
    document.getElementById('msg-count-value').textContent = state.messageCount;
    document.getElementById('error-count-value').textContent = state.errorCount;
}

function showStatus(message, type = 'info') {
    const el = document.getElementById('status-message');
    el.textContent = message;
    el.className = type;
    
    setTimeout(() => {
        if (el.textContent === message) {
            el.textContent = '';
            el.className = '';
        }
    }, 5000);
}

// =============================================================================
// API Calls
// =============================================================================

async function saveCalibration() {
    try {
        showStatus('Downloading calibration data...');
        const response = await fetch('/api/calibrate/save', { method: 'GET' });
        
        if (!response.ok) {
            const data = await response.json();
            showStatus('Failed to save: ' + (data.error || 'Unknown error'), 'error');
            return;
        }
        
        // Get the JSON data and trigger download
        const blob = await response.blob();
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'bno055_calibration.json';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        window.URL.revokeObjectURL(url);
        
        showStatus('Calibration file downloaded', 'success');
    } catch (e) {
        showStatus('Error saving calibration: ' + e.message, 'error');
    }
}

function loadCalibration() {
    // Create a hidden file input and trigger it
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json,application/json';
    
    input.onchange = async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        
        try {
            showStatus('Reading calibration file...');
            const text = await file.text();
            const calData = JSON.parse(text);
            
            // Validate basic structure
            if (!calData.calibration_bytes) {
                showStatus('Invalid calibration file format', 'error');
                return;
            }
            
            showStatus('Applying calibration to sensor...');
            const response = await fetch('/api/calibrate/load', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(calData)
            });
            
            const data = await response.json();
            
            if (data.success) {
                const savedAt = calData.saved_at ? 
                    ` (saved: ${new Date(calData.saved_at).toLocaleString()})` : '';
                showStatus('Calibration applied successfully' + savedAt, 'success');
            } else {
                showStatus('Failed to apply: ' + data.error, 'error');
            }
        } catch (e) {
            if (e instanceof SyntaxError) {
                showStatus('Invalid JSON file', 'error');
            } else {
                showStatus('Error loading calibration: ' + e.message, 'error');
            }
        }
    };
    
    input.click();
}

async function applyConfig() {
    const bus = parseInt(document.getElementById('i2c-bus').value);
    const address = document.getElementById('i2c-address').value;
    
    try {
        showStatus('Applying configuration...');
        const response = await fetch('/api/config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ i2c_bus: bus, i2c_address: address })
        });
        const data = await response.json();
        
        if (data.success) {
            showStatus(data.message, 'success');
        } else {
            showStatus('Failed: ' + data.error, 'error');
        }
    } catch (e) {
        showStatus('Error: ' + e.message, 'error');
    }
}

async function restartIMU() {
    try {
        showStatus('Restarting IMU...');
        const response = await fetch('/api/restart', { method: 'POST' });
        const data = await response.json();
        
        if (data.success) {
            showStatus('IMU restarted', 'success');
        } else {
            showStatus('Failed: ' + data.error, 'error');
        }
    } catch (e) {
        showStatus('Error: ' + e.message, 'error');
    }
}

async function loadConfig() {
    try {
        const response = await fetch('/api/config');
        const data = await response.json();
        
        document.getElementById('i2c-bus').value = data.i2c_bus;
        
        const addressSelect = document.getElementById('i2c-address');
        const address = data.i2c_address;
        for (let option of addressSelect.options) {
            if (option.value === address) {
                option.selected = true;
                break;
            }
        }
    } catch (e) {
        console.error('Failed to load config:', e);
    }
}

function azimuthToCardinal(deg) {
    const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'N'];
    const index = Math.round(deg / 45) % 8;
    return directions[index];
}

// =============================================================================
// Initialization
// =============================================================================

function init() {
    // Initialize WebGL
    if (!initWebGL()) {
        console.error('Failed to initialize WebGL');
        return;
    }
    
    // Initialize compass
    initCompass();
    
    // Start animation loop
    state.lastUpdate = Date.now();
    animate();
    
    // Connect to SSE stream
    connectSSE();
    
    // Bind button events
    document.getElementById('btn-save-cal').addEventListener('click', saveCalibration);
    document.getElementById('btn-load-cal').addEventListener('click', loadCalibration);
    document.getElementById('btn-apply-config').addEventListener('click', applyConfig);
    document.getElementById('btn-restart').addEventListener('click', restartIMU);
    
    // View rotation slider
    const azimuthSlider = document.getElementById('view-azimuth');
    const azimuthValue = document.getElementById('view-azimuth-value');
    
    azimuthSlider.addEventListener('input', (e) => {
        cameraAzimuth = parseInt(e.target.value);
        azimuthValue.textContent = azimuthToCardinal(cameraAzimuth);
    });
    
    // Initialize azimuth display
    azimuthValue.textContent = azimuthToCardinal(cameraAzimuth);
    azimuthSlider.value = cameraAzimuth;
    
    // Load current config
    loadConfig();
}

document.addEventListener('DOMContentLoaded', init);
