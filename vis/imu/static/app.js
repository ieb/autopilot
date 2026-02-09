/**
 * BNO055 IMU Testing Tool
 * 
 * Pure JavaScript application for real-time IMU visualization.
 * Uses 2D canvas with manual 3D projection for orientation display.
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
let canvas = null;
let ctx = null;
let animationId = null;

// =============================================================================
// 3D Rendering
// =============================================================================

/**
 * Convert degrees to radians
 */
function degToRad(deg) {
    return deg * Math.PI / 180;
}

/**
 * Multiply two 3x3 matrices
 */
function matMul(a, b) {
    return [
        [a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0],
         a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1],
         a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2]],
        [a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0],
         a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1],
         a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2]],
        [a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0],
         a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1],
         a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2]]
    ];
}

/**
 * Apply matrix to vector
 */
function matVec(m, v) {
    return [
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]
    ];
}

/**
 * Create rotation matrix for Euler angles (ZYX order: heading, pitch, roll)
 * Following nautical convention:
 * - Heading (yaw): rotation around Z axis (positive = clockwise from above)
 * - Pitch: rotation around Y axis  
 * - Roll: rotation around X axis
 */
function createRotationMatrix(heading, pitch, roll) {
    const h = degToRad(heading);  // Heading: 0=N, 90=E, 180=S, 270=W
    const p = degToRad(-pitch);   // Negate pitch for correct visual
    const r = degToRad(-roll);    // Negate roll for correct visual
    
    // Rotation around Z (heading/yaw)
    const rz = [
        [Math.cos(h), -Math.sin(h), 0],
        [Math.sin(h),  Math.cos(h), 0],
        [0,            0,           1]
    ];
    
    // Rotation around Y (pitch)
    const ry = [
        [ Math.cos(p), 0, Math.sin(p)],
        [ 0,           1, 0          ],
        [-Math.sin(p), 0, Math.cos(p)]
    ];
    
    // Rotation around X (roll)
    const rx = [
        [1, 0,            0           ],
        [0, Math.cos(r), -Math.sin(r)],
        [0, Math.sin(r),  Math.cos(r)]
    ];
    
    // Combined: R = Rz * Ry * Rx
    return matMul(matMul(rz, ry), rx);
}

/**
 * Create camera view matrix for fixed viewing angle
 * Camera positioned North-West looking at the board from above
 * 
 * World coordinates (BNO055): X=forward, Y=right, Z=up
 * Screen coordinates: X=right, Y=up
 * 
 * Camera is positioned at NW (azimuth -45° from North) and 30° above horizon
 */
function createCameraMatrix() {
    // Azimuth: -45 degrees (NW position, looking toward SE)
    const azimuth = degToRad(-45);
    
    // Elevation: 30 degrees above horizon (looking slightly down)
    const elevation = degToRad(30);
    
    // Step 1: Rotate around Z axis for azimuth (horizontal rotation)
    const cosA = Math.cos(azimuth);
    const sinA = Math.sin(azimuth);
    const rz = [
        [cosA, -sinA, 0],
        [sinA,  cosA, 0],
        [0,     0,    1]
    ];
    
    // Step 2: Rotate around the NEW X axis for elevation (tilt view down)
    const cosE = Math.cos(elevation);
    const sinE = Math.sin(elevation);
    const rx = [
        [1, 0,     0    ],
        [0, cosE, -sinE],
        [0, sinE,  cosE]
    ];
    
    // Step 3: Transform from world coords (Z-up) to screen coords (Y-up)
    // This swaps Y and Z so that world-Z appears as screen-Y (vertical)
    const coordTransform = [
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0]
    ];
    
    // Combined: world rotation, then elevation, then coord transform
    return matMul(coordTransform, matMul(rx, rz));
}

// Pre-compute camera matrix (static view angle)
let cameraMatrix = null;

/**
 * Project 3D point to 2D screen coordinates
 */
function project(point, rotMatrix, scale, offsetX, offsetY) {
    // Apply rotation
    const rotated = matVec(rotMatrix, point);
    
    // Simple perspective projection
    // Camera is positioned further back for a better view
    const fov = 300;
    const cameraDistance = 200;  // Distance from origin to camera
    const z = rotated[2] + cameraDistance;
    const factor = fov / z;
    
    return {
        x: offsetX + rotated[0] * factor * scale,
        y: offsetY - rotated[1] * factor * scale,  // Flip Y for screen coords
        z: rotated[2]
    };
}

/**
 * Draw the 3D sensor visualization
 */
function draw3D() {
    if (!ctx || !canvas) return;
    
    const width = canvas.width;
    const height = canvas.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const scale = 2.5;  // Scale up to compensate for zoomed out camera
    
    // Clear canvas
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, width, height);
    
    // Initialize camera matrix if needed
    if (!cameraMatrix) {
        cameraMatrix = createCameraMatrix();
    }
    
    // Create rotation matrix from current orientation
    const imuRotation = createRotationMatrix(state.heading, state.pitch, state.roll);
    
    // Apply camera view transformation after IMU rotation
    const rotMatrix = matMul(cameraMatrix, imuRotation);
    
    // Define box vertices (representing the sensor)
    // Dimensions: 80x50x20 (length x width x height)
    const hw = 40, hd = 25, hh = 10;  // half-widths
    const vertices = [
        [-hw, -hd, -hh], [hw, -hd, -hh], [hw, hd, -hh], [-hw, hd, -hh],  // bottom
        [-hw, -hd,  hh], [hw, -hd,  hh], [hw, hd,  hh], [-hw, hd,  hh]   // top
    ];
    
    // Define edges (pairs of vertex indices)
    const edges = [
        [0,1], [1,2], [2,3], [3,0],  // bottom
        [4,5], [5,6], [6,7], [7,4],  // top
        [0,4], [1,5], [2,6], [3,7]   // sides
    ];
    
    // Define faces for depth sorting (vertex indices, color)
    const faces = [
        { verts: [0,1,2,3], color: '#2a4a7a', label: 'Bottom' },  // bottom
        { verts: [4,5,6,7], color: '#3a6a9a', label: 'Top' },     // top
        { verts: [0,1,5,4], color: '#4a5a8a', label: 'Front' },   // front (bow)
        { verts: [2,3,7,6], color: '#3a4a7a', label: 'Back' },    // back (stern)
        { verts: [1,2,6,5], color: '#4a6a8a', label: 'Right' },   // right (starboard)
        { verts: [0,3,7,4], color: '#3a5a7a', label: 'Left' }     // left (port)
    ];
    
    // Project all vertices
    const projected = vertices.map(v => project(v, rotMatrix, scale, centerX, centerY));
    
    // Calculate face depths and sort back-to-front
    const facesWithDepth = faces.map(face => {
        const avgZ = face.verts.reduce((sum, vi) => sum + projected[vi].z, 0) / face.verts.length;
        return { ...face, avgZ };
    });
    facesWithDepth.sort((a, b) => a.avgZ - b.avgZ);
    
    // Draw faces
    facesWithDepth.forEach(face => {
        ctx.beginPath();
        const first = projected[face.verts[0]];
        ctx.moveTo(first.x, first.y);
        for (let i = 1; i < face.verts.length; i++) {
            const p = projected[face.verts[i]];
            ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.fillStyle = face.color;
        ctx.fill();
        ctx.strokeStyle = '#5a7aaa';
        ctx.lineWidth = 1;
        ctx.stroke();
    });
    
    // Draw front indicator (arrow on bow)
    const bowCenter = [0, -hd - 15, 0];
    const bowLeft = [-8, -hd - 5, 0];
    const bowRight = [8, -hd - 5, 0];
    const bowP = project(bowCenter, rotMatrix, scale, centerX, centerY);
    const bowL = project(bowLeft, rotMatrix, scale, centerX, centerY);
    const bowR = project(bowRight, rotMatrix, scale, centerX, centerY);
    
    ctx.beginPath();
    ctx.moveTo(bowP.x, bowP.y);
    ctx.lineTo(bowL.x, bowL.y);
    ctx.lineTo(bowR.x, bowR.y);
    ctx.closePath();
    ctx.fillStyle = '#e94560';
    ctx.fill();
    
    // Draw axis indicators
    const axisLength = 60;
    const axisOrigin = [0, 0, 0];
    const axes = [
        { dir: [axisLength, 0, 0], color: '#ff6b6b', label: 'X' },  // X - Red (forward)
        { dir: [0, axisLength, 0], color: '#4ade80', label: 'Y' },  // Y - Green (right)
        { dir: [0, 0, axisLength], color: '#60a5fa', label: 'Z' }   // Z - Blue (up)
    ];
    
    const originP = project(axisOrigin, rotMatrix, scale, centerX, centerY);
    
    axes.forEach(axis => {
        const endPoint = [axis.dir[0], axis.dir[1], axis.dir[2]];
        const endP = project(endPoint, rotMatrix, scale, centerX, centerY);
        
        ctx.beginPath();
        ctx.moveTo(originP.x, originP.y);
        ctx.lineTo(endP.x, endP.y);
        ctx.strokeStyle = axis.color;
        ctx.lineWidth = 2;
        ctx.stroke();
        
        // Axis label
        ctx.fillStyle = axis.color;
        ctx.font = 'bold 12px sans-serif';
        ctx.fillText(axis.label, endP.x + 5, endP.y - 5);
    });
    
    // Draw compass rose in corner
    drawCompassRose(50, height - 50, 35);
}

/**
 * Draw a compass rose showing current heading
 */
function drawCompassRose(cx, cy, radius) {
    const headingRad = degToRad(state.heading);
    
    // Background circle
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
    ctx.fill();
    ctx.strokeStyle = '#5a7aaa';
    ctx.lineWidth = 1;
    ctx.stroke();
    
    // Cardinal directions (fixed)
    const dirs = [
        { label: 'N', angle: 0 },
        { label: 'E', angle: 90 },
        { label: 'S', angle: 180 },
        { label: 'W', angle: 270 }
    ];
    
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    
    dirs.forEach(d => {
        const a = degToRad(d.angle - 90);
        const x = cx + Math.cos(a) * (radius - 8);
        const y = cy + Math.sin(a) * (radius - 8);
        ctx.fillStyle = d.label === 'N' ? '#e94560' : '#888';
        ctx.fillText(d.label, x, y);
    });
    
    // Heading indicator (rotating arrow)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(headingRad);
    
    ctx.beginPath();
    ctx.moveTo(0, -radius + 15);
    ctx.lineTo(-6, 5);
    ctx.lineTo(6, 5);
    ctx.closePath();
    ctx.fillStyle = '#e94560';
    ctx.fill();
    
    ctx.restore();
}

/**
 * Animation loop
 */
function animate() {
    draw3D();
    animationId = requestAnimationFrame(animate);
}

// =============================================================================
// SSE Client
// =============================================================================

/**
 * Connect to SSE stream
 */
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
        
        // Attempt reconnect after delay
        setTimeout(() => {
            if (eventSource.readyState === EventSource.CLOSED) {
                connectSSE();
            }
        }, 2000);
    };
}

/**
 * Handle incoming IMU data
 */
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

/**
 * Update connection status indicator
 */
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

/**
 * Update orientation value displays
 */
function updateOrientationDisplay() {
    document.getElementById('heading-value').textContent = state.heading.toFixed(1) + '\u00B0';
    document.getElementById('pitch-value').textContent = state.pitch.toFixed(1) + '\u00B0';
    document.getElementById('roll-value').textContent = state.roll.toFixed(1) + '\u00B0';
}

/**
 * Update calibration status display
 */
function updateCalibrationDisplay() {
    updateCalBars('cal-sys', state.calSys);
    updateCalBars('cal-gyro', state.calGyro);
    updateCalBars('cal-accel', state.calAccel);
    updateCalBars('cal-mag', state.calMag);
    
    document.getElementById('cal-sys-value').textContent = state.calSys + '/3';
    document.getElementById('cal-gyro-value').textContent = state.calGyro + '/3';
    document.getElementById('cal-accel-value').textContent = state.calAccel + '/3';
    document.getElementById('cal-mag-value').textContent = state.calMag + '/3';
    
    // Update hint
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

/**
 * Update calibration bar display
 */
function updateCalBars(id, level) {
    const container = document.getElementById(id);
    container.className = 'cal-bars level-' + level;
}

/**
 * Update sensor data display
 */
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

/**
 * Show status message in footer
 */
function showStatus(message, type = 'info') {
    const el = document.getElementById('status-message');
    el.textContent = message;
    el.className = type;
    
    // Clear after delay
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

/**
 * Save calibration to file
 */
async function saveCalibration() {
    try {
        const response = await fetch('/api/calibrate/save', { method: 'POST' });
        const data = await response.json();
        
        if (data.success) {
            showStatus('Calibration saved successfully', 'success');
        } else {
            showStatus('Failed to save: ' + data.error, 'error');
        }
    } catch (e) {
        showStatus('Error saving calibration: ' + e.message, 'error');
    }
}

/**
 * Load calibration from file
 */
async function loadCalibration() {
    try {
        showStatus('Loading calibration (restarting IMU)...');
        const response = await fetch('/api/calibrate/load', { method: 'POST' });
        const data = await response.json();
        
        if (data.success) {
            showStatus('Calibration loaded successfully', 'success');
        } else {
            showStatus('Failed to load: ' + data.error, 'error');
        }
    } catch (e) {
        showStatus('Error loading calibration: ' + e.message, 'error');
    }
}

/**
 * Apply new configuration
 */
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

/**
 * Restart IMU
 */
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

// =============================================================================
// Initialization
// =============================================================================

function init() {
    // Get canvas
    canvas = document.getElementById('imu-canvas');
    ctx = canvas.getContext('2d');
    
    // Handle high DPI displays
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    canvas.style.width = rect.width + 'px';
    canvas.style.height = rect.height + 'px';
    
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
    
    // Load current config
    loadConfig();
}

/**
 * Load current configuration
 */
async function loadConfig() {
    try {
        const response = await fetch('/api/config');
        const data = await response.json();
        
        document.getElementById('i2c-bus').value = data.i2c_bus;
        
        // Set address dropdown
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

// Start when DOM is ready
document.addEventListener('DOMContentLoaded', init);
