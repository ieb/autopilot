/**
 * GRIB Weather Visualization
 * 
 * Displays animated wind streamlines and wave height gradient overlay on OpenSeaMap.
 * Wind visualization inspired by windy.com particle animation.
 */

// Global state
let map = null;
let windCanvas = null;
let waveCanvas = null;
let windData = null;
let waveData = null;
let metadata = null;
let currentTimeIndex = 0;

// Particle animation state
let particles = [];
let animationId = null;
let lastFrameTime = 0;
const PARTICLE_MAX_AGE = 100;  // frames
const PARTICLE_FADE_START = 0.7;  // start fading at 70% of max age
const SPEED_SCALE = 0.00015;  // scale wind speed to pixel movement

// Particle count based on density setting
function getParticleCount() {
    const density = parseInt(document.getElementById('particle-density')?.value || 3);
    return [500, 1500, 3000, 5000, 8000][density - 1];
}

// Get particle line width from slider
function getParticleSize() {
    return parseFloat(document.getElementById('particle-size')?.value || 1.5);
}

// Get tail fade factor from slider (higher = longer tail)
function getTailFade() {
    return parseFloat(document.getElementById('tail-length')?.value || 0.92);
}

// Wave color scale - extended to 9m with gradient interpolation
const WAVE_COLOR_STOPS = [
    { height: 0, r: 52, g: 152, b: 219 },    // Blue (calm)
    { height: 1, r: 46, g: 204, b: 170 },    // Teal
    { height: 2, r: 46, g: 204, b: 113 },    // Green
    { height: 3, r: 162, g: 212, b: 53 },    // Yellow-green
    { height: 4, r: 241, g: 196, b: 15 },    // Yellow
    { height: 5, r: 230, g: 126, b: 34 },    // Orange
    { height: 6, r: 231, g: 76, b: 60 },     // Red
    { height: 7, r: 192, g: 57, b: 112 },    // Magenta/pink
    { height: 8, r: 142, g: 68, b: 173 },    // Purple
    { height: 9, r: 255, g: 255, b: 255 },   // White (extreme)
];

/**
 * Initialize the map and layers
 */
function initMap() {
    // Create map centered on English Channel (default)
    map = L.map('map').setView([50.0, -1.5], 7);
    
    // OpenStreetMap base layer
    const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    });
    
    // OpenSeaMap overlay
    const seamapLayer = L.tileLayer('https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="http://www.openseamap.org">OpenSeaMap</a> contributors'
    });
    
    // Add layers
    osmLayer.addTo(map);
    seamapLayer.addTo(map);
    
    // Create canvas layers for wind and waves
    createCanvasLayers();
    
    // Redraw on map events
    map.on('moveend', redraw);
    map.on('zoomend', redraw);
}

/**
 * Create canvas overlay layers for wind and waves
 */
function createCanvasLayers() {
    // Wave canvas (below wind arrows)
    const WaveCanvasLayer = L.Layer.extend({
        onAdd: function(map) {
            this._map = map;
            const pane = map.getPane('overlayPane');
            this._canvas = L.DomUtil.create('canvas', 'wave-canvas');
            pane.appendChild(this._canvas);
            waveCanvas = this._canvas;
            this._updatePosition();
        },
        onRemove: function(map) {
            L.DomUtil.remove(this._canvas);
        },
        _updatePosition: function() {
            const size = this._map.getSize();
            this._canvas.width = size.x;
            this._canvas.height = size.y;
            const topLeft = this._map.containerPointToLayerPoint([0, 0]);
            L.DomUtil.setPosition(this._canvas, topLeft);
        }
    });
    
    // Wind canvas (above waves)
    const WindCanvasLayer = L.Layer.extend({
        onAdd: function(map) {
            this._map = map;
            const pane = map.getPane('overlayPane');
            this._canvas = L.DomUtil.create('canvas', 'wind-canvas');
            pane.appendChild(this._canvas);
            windCanvas = this._canvas;
            this._updatePosition();
        },
        onRemove: function(map) {
            L.DomUtil.remove(this._canvas);
        },
        _updatePosition: function() {
            const size = this._map.getSize();
            this._canvas.width = size.x;
            this._canvas.height = size.y;
            const topLeft = this._map.containerPointToLayerPoint([0, 0]);
            L.DomUtil.setPosition(this._canvas, topLeft);
        }
    });
    
    new WaveCanvasLayer().addTo(map);
    new WindCanvasLayer().addTo(map);
}

/**
 * Fetch metadata about available GRIB data
 */
async function fetchMetadata() {
    showLoading(true);
    try {
        const response = await fetch('/api/metadata');
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        metadata = await response.json();
        
        if (metadata.error) {
            throw new Error(metadata.error);
        }
        
        // Set up time slider
        const times = metadata.wind_times.length > 0 ? metadata.wind_times : metadata.wave_times;
        if (times.length > 0) {
            const slider = document.getElementById('time-slider');
            slider.min = 0;
            slider.max = times.length - 1;
            slider.value = 0;
            updateTimeDisplay(times[0]);
        }
        
        // Fit map to data bounds if available
        if (metadata.bounds) {
            const bounds = L.latLngBounds(
                [metadata.bounds.lat_min, metadata.bounds.lon_min],
                [metadata.bounds.lat_max, metadata.bounds.lon_max]
            );
            map.fitBounds(bounds.pad(0.1));
        }
        
        // Load initial data
        await loadData();
        
    } catch (error) {
        showError(`Failed to load metadata: ${error.message}`);
    } finally {
        showLoading(false);
    }
}

/**
 * Load wind and wave data for current time
 */
async function loadData() {
    if (!metadata) return;
    
    const times = metadata.wind_times.length > 0 ? metadata.wind_times : metadata.wave_times;
    if (times.length === 0) return;
    
    const time = times[currentTimeIndex];
    showLoading(true);
    
    try {
        // Fetch wind and wave data in parallel
        const [windResponse, waveResponse] = await Promise.all([
            metadata.wind_times.length > 0 ? fetch(`/api/wind?time=${time}`) : null,
            metadata.wave_times.length > 0 ? fetch(`/api/waves?time=${time}`) : null
        ].filter(Boolean));
        
        if (windResponse) {
            const data = await windResponse.json();
            if (!data.error) {
                windData = data;
            }
        }
        
        if (waveResponse) {
            const data = await waveResponse.json();
            if (!data.error) {
                waveData = data;
            }
        }
        
        redraw();
        
        // Start or restart wind animation
        if (windData) {
            startWindAnimation();
        }
        
    } catch (error) {
        showError(`Failed to load data: ${error.message}`);
    } finally {
        showLoading(false);
    }
}

/**
 * Redraw all canvas layers
 */
function redraw() {
    if (!map) return;
    
    // Update canvas positions and sizes
    const size = map.getSize();
    
    if (waveCanvas) {
        waveCanvas.width = size.x;
        waveCanvas.height = size.y;
        const topLeft = map.containerPointToLayerPoint([0, 0]);
        L.DomUtil.setPosition(waveCanvas, topLeft);
        
        if (document.getElementById('show-waves').checked) {
            drawWaves();
        } else {
            const ctx = waveCanvas.getContext('2d');
            ctx.clearRect(0, 0, waveCanvas.width, waveCanvas.height);
        }
    }
    
    if (windCanvas) {
        windCanvas.width = size.x;
        windCanvas.height = size.y;
        const topLeft = map.containerPointToLayerPoint([0, 0]);
        L.DomUtil.setPosition(windCanvas, topLeft);
        
        // Reset particle animation on map move/zoom
        resetWindAnimation();
        
        if (!document.getElementById('show-wind').checked) {
            const ctx = windCanvas.getContext('2d');
            ctx.clearRect(0, 0, windCanvas.width, windCanvas.height);
        }
    }
}

/**
 * Draw wave height gradient overlay
 */
function drawWaves() {
    if (!waveCanvas || !waveData || !waveData.points) return;
    
    const ctx = waveCanvas.getContext('2d');
    ctx.clearRect(0, 0, waveCanvas.width, waveCanvas.height);
    
    const bounds = map.getBounds();
    const zoom = map.getZoom();
    
    // Calculate cell size in pixels based on grid resolution
    if (!waveData.bounds || !waveData.grid_size) return;
    
    const latStep = (waveData.bounds.lat_max - waveData.bounds.lat_min) / (waveData.grid_size.rows - 1);
    const lonStep = (waveData.bounds.lon_max - waveData.bounds.lon_min) / (waveData.grid_size.cols - 1);
    
    for (const point of waveData.points) {
        // Check if point is in view
        if (point.lat < bounds.getSouth() - latStep || 
            point.lat > bounds.getNorth() + latStep ||
            point.lon < bounds.getWest() - lonStep || 
            point.lon > bounds.getEast() + lonStep) {
            continue;
        }
        
        // Get pixel position
        const topLeft = map.latLngToContainerPoint([point.lat + latStep/2, point.lon - lonStep/2]);
        const bottomRight = map.latLngToContainerPoint([point.lat - latStep/2, point.lon + lonStep/2]);
        
        // Get color for wave height
        const color = getWaveColor(point.height);
        
        ctx.fillStyle = `rgba(${color.r}, ${color.g}, ${color.b}, 0.4)`;
        ctx.fillRect(
            topLeft.x, 
            topLeft.y, 
            bottomRight.x - topLeft.x, 
            bottomRight.y - topLeft.y
        );
    }
}

/**
 * Get color for wave height with gradient interpolation
 */
function getWaveColor(height) {
    // Clamp height to scale range
    const clampedHeight = Math.max(0, Math.min(height, 9));
    
    // Find the two color stops to interpolate between
    let lower = WAVE_COLOR_STOPS[0];
    let upper = WAVE_COLOR_STOPS[WAVE_COLOR_STOPS.length - 1];
    
    for (let i = 0; i < WAVE_COLOR_STOPS.length - 1; i++) {
        if (clampedHeight >= WAVE_COLOR_STOPS[i].height && 
            clampedHeight <= WAVE_COLOR_STOPS[i + 1].height) {
            lower = WAVE_COLOR_STOPS[i];
            upper = WAVE_COLOR_STOPS[i + 1];
            break;
        }
    }
    
    // Calculate interpolation factor
    const range = upper.height - lower.height;
    const t = range > 0 ? (clampedHeight - lower.height) / range : 0;
    
    // Interpolate RGB values
    return {
        r: Math.round(lower.r + (upper.r - lower.r) * t),
        g: Math.round(lower.g + (upper.g - lower.g) * t),
        b: Math.round(lower.b + (upper.b - lower.b) * t)
    };
}

/**
 * Particle class for wind animation
 */
class WindParticle {
    constructor(canvas) {
        this.reset(canvas, true);
    }
    
    reset(canvas, randomAge = false) {
        // Random position on canvas
        this.x = Math.random() * canvas.width;
        this.y = Math.random() * canvas.height;
        this.prevX = this.x;
        this.prevY = this.y;
        // Random age for initial distribution
        this.age = randomAge ? Math.floor(Math.random() * PARTICLE_MAX_AGE) : 0;
        this.maxAge = PARTICLE_MAX_AGE + Math.floor(Math.random() * 20 - 10);
    }
}

/**
 * Initialize particles for wind animation
 */
function initParticles() {
    particles = [];
    if (!windCanvas) return;
    
    const count = getParticleCount();
    for (let i = 0; i < count; i++) {
        particles.push(new WindParticle(windCanvas));
    }
}

/**
 * Get wind velocity at a pixel position
 */
function getWindAtPixel(x, y) {
    if (!windData || !windData.points || !windData.bounds || !map) return null;
    
    // Convert pixel to lat/lng
    const latlng = map.containerPointToLatLng([x, y]);
    const lat = latlng.lat;
    const lon = latlng.lng;
    
    // Check if point is within data bounds (with small margin)
    const bounds = windData.bounds;
    const margin = (bounds.lat_max - bounds.lat_min) * 0.1;  // 10% margin
    if (lat < bounds.lat_min - margin || lat > bounds.lat_max + margin ||
        lon < bounds.lon_min - margin || lon > bounds.lon_max + margin) {
        return null;
    }
    
    // Find closest wind point
    let closest = null;
    let minDist = Infinity;
    
    for (const point of windData.points) {
        const dist = Math.pow(point.lat - lat, 2) + Math.pow(point.lon - lon, 2);
        if (dist < minDist) {
            minDist = dist;
            closest = point;
        }
    }
    
    if (!closest) return null;
    
    // Also check that closest point is reasonably close
    // (handles sparse data or gaps in coverage)
    const gridStep = windData.grid_size ? 
        Math.max(
            (bounds.lat_max - bounds.lat_min) / windData.grid_size.rows,
            (bounds.lon_max - bounds.lon_min) / windData.grid_size.cols
        ) : 1;
    const maxDist = gridStep * 2;  // Allow up to 2 grid cells distance
    if (Math.sqrt(minDist) > maxDist) {
        return null;
    }
    
    // Convert wind speed and direction to velocity components
    // direction is meteorological (FROM), we want velocity TO
    const toDir = (closest.direction + 180) % 360;
    const rad = toDir * Math.PI / 180;
    const speed = closest.speed;
    
    // Scale speed for pixel movement
    const zoom = map.getZoom();
    const scale = SPEED_SCALE * Math.pow(2, zoom);
    
    // vx = east component, vy = south component (canvas Y is down)
    const vx = Math.sin(rad) * speed * scale;
    const vy = Math.cos(rad) * speed * scale;
    
    return { vx, vy, speed };
}

/**
 * Animate wind particles
 */
function animateWind(timestamp) {
    if (!windCanvas || !document.getElementById('show-wind').checked) {
        animationId = requestAnimationFrame(animateWind);
        return;
    }
    
    const ctx = windCanvas.getContext('2d');
    
    // Fade previous frame (creates trail effect)
    // Higher fade value = longer tail
    const tailFade = getTailFade();
    ctx.fillStyle = `rgba(255, 255, 255, ${tailFade})`;
    ctx.globalCompositeOperation = 'destination-in';
    ctx.fillRect(0, 0, windCanvas.width, windCanvas.height);
    ctx.globalCompositeOperation = 'source-over';
    
    // Update and draw particles
    for (const particle of particles) {
        // Get wind at current position
        const wind = getWindAtPixel(particle.x, particle.y);
        
        if (wind && wind.speed > 0.5) {
            // Store previous position for line drawing
            particle.prevX = particle.x;
            particle.prevY = particle.y;
            
            // Move particle
            particle.x += wind.vx;
            particle.y += wind.vy;
            particle.age++;
            
            // Calculate opacity based on age
            let opacity = 1;
            const fadeStart = particle.maxAge * PARTICLE_FADE_START;
            if (particle.age > fadeStart) {
                opacity = 1 - (particle.age - fadeStart) / (particle.maxAge - fadeStart);
            }
            // Also fade in at start
            if (particle.age < 10) {
                opacity = Math.min(opacity, particle.age / 10);
            }
            
            // Get color based on wind speed
            const color = getWindColorRGB(wind.speed);
            
            // Draw particle trail
            ctx.strokeStyle = `rgba(${color.r}, ${color.g}, ${color.b}, ${opacity * 0.8})`;
            ctx.lineWidth = getParticleSize();
            ctx.lineCap = 'round';
            ctx.beginPath();
            ctx.moveTo(particle.prevX, particle.prevY);
            ctx.lineTo(particle.x, particle.y);
            ctx.stroke();
            
            // Reset if out of bounds or too old
            if (particle.age > particle.maxAge ||
                particle.x < 0 || particle.x > windCanvas.width ||
                particle.y < 0 || particle.y > windCanvas.height) {
                particle.reset(windCanvas, false);
            }
        } else {
            // No wind data at this position, reset particle
            particle.reset(windCanvas, false);
        }
    }
    
    animationId = requestAnimationFrame(animateWind);
}

/**
 * Start wind animation
 */
function startWindAnimation() {
    if (animationId) {
        cancelAnimationFrame(animationId);
    }
    initParticles();
    animationId = requestAnimationFrame(animateWind);
}

/**
 * Stop wind animation
 */
function stopWindAnimation() {
    if (animationId) {
        cancelAnimationFrame(animationId);
        animationId = null;
    }
}

/**
 * Clear and restart wind animation (e.g., after map move)
 */
function resetWindAnimation() {
    if (!windCanvas) return;
    const ctx = windCanvas.getContext('2d');
    ctx.clearRect(0, 0, windCanvas.width, windCanvas.height);
    initParticles();
}

/**
 * Get wind color as RGB object
 */
function getWindColorRGB(speed) {
    // Windy.com style color scale
    if (speed < 5) {
        return { r: 98, g: 182, b: 183 };      // Light teal
    } else if (speed < 10) {
        return { r: 76, g: 165, b: 130 };      // Teal-green
    } else if (speed < 15) {
        return { r: 102, g: 194, b: 64 };      // Green
    } else if (speed < 20) {
        return { r: 162, g: 212, b: 53 };      // Yellow-green
    } else if (speed < 25) {
        return { r: 243, g: 225, b: 55 };      // Yellow
    } else if (speed < 30) {
        return { r: 248, g: 180, b: 52 };      // Orange
    } else if (speed < 40) {
        return { r: 240, g: 96, b: 52 };       // Red-orange
    } else {
        return { r: 206, g: 48, b: 103 };      // Magenta/red
    }
}

/**
 * Get color for wind speed (string version)
 */
function getWindColor(speed) {
    const rgb = getWindColorRGB(speed);
    return `rgb(${rgb.r}, ${rgb.g}, ${rgb.b})`;
}

/**
 * Update the time display
 */
function updateTimeDisplay(isoTime) {
    const display = document.getElementById('time-display');
    try {
        const date = new Date(isoTime);
        display.textContent = date.toLocaleString();
    } catch {
        display.textContent = isoTime;
    }
}

/**
 * Show/hide loading indicator
 */
function showLoading(show) {
    const loading = document.getElementById('loading');
    if (show) {
        loading.classList.remove('hidden');
    } else {
        loading.classList.add('hidden');
    }
}

/**
 * Show error message
 */
function showError(message) {
    const error = document.getElementById('error');
    error.textContent = message;
    error.classList.remove('hidden');
    setTimeout(() => {
        error.classList.add('hidden');
    }, 5000);
}

/**
 * Find the closest data point to a lat/lon position
 */
function findClosestWindPoint(lat, lon) {
    if (!windData || !windData.points || windData.points.length === 0) {
        return null;
    }
    
    let closest = null;
    let minDist = Infinity;
    
    for (const point of windData.points) {
        const dist = Math.pow(point.lat - lat, 2) + Math.pow(point.lon - lon, 2);
        if (dist < minDist) {
            minDist = dist;
            closest = point;
        }
    }
    
    return closest;
}

/**
 * Find the closest wave data point to a lat/lon position
 */
function findClosestWavePoint(lat, lon) {
    if (!waveData || !waveData.points || waveData.points.length === 0) {
        return null;
    }
    
    let closest = null;
    let minDist = Infinity;
    
    for (const point of waveData.points) {
        const dist = Math.pow(point.lat - lat, 2) + Math.pow(point.lon - lon, 2);
        if (dist < minDist) {
            minDist = dist;
            closest = point;
        }
    }
    
    return closest;
}

/**
 * Helper to set tooltip field visibility and value
 */
function setTooltipField(elementId, value, wrapId = null) {
    const el = document.getElementById(elementId);
    if (!el) return;
    
    if (value !== undefined && value !== null) {
        el.textContent = typeof value === 'number' ? value.toFixed(value < 10 ? 2 : 1) : value;
        if (wrapId) {
            const wrap = document.getElementById(wrapId);
            if (wrap) wrap.style.display = 'inline';
        }
    } else {
        el.textContent = '--';
        if (wrapId) {
            const wrap = document.getElementById(wrapId);
            if (wrap) wrap.style.display = 'none';
        }
    }
}

/**
 * Update tooltip with data at mouse position
 */
function updateTooltip(e) {
    const tooltip = document.getElementById('tooltip');
    const latlng = e.latlng;
    
    if (!latlng) {
        tooltip.classList.add('hidden');
        return;
    }
    
    const lat = latlng.lat;
    const lon = latlng.lng;
    
    // Update coordinates
    document.getElementById('tooltip-lat').textContent = lat.toFixed(3) + '°';
    document.getElementById('tooltip-lon').textContent = lon.toFixed(3) + '°';
    
    // Find closest wind point
    const windPoint = findClosestWindPoint(lat, lon);
    const windSection = document.getElementById('tooltip-wind');
    
    if (windPoint) {
        document.getElementById('tooltip-wind-speed').textContent = windPoint.speed.toFixed(1);
        document.getElementById('tooltip-wind-dir').textContent = windPoint.direction.toFixed(0);
        windSection.style.display = 'block';
    } else {
        windSection.style.display = 'none';
    }
    
    // Find closest wave point
    const wavePoint = findClosestWavePoint(lat, lon);
    const waveSection = document.getElementById('tooltip-waves');
    
    if (wavePoint) {
        waveSection.style.display = 'block';
        
        // Total/combined waves
        const totalRow = document.getElementById('tooltip-wave-total');
        if (wavePoint.height !== undefined) {
            setTooltipField('tooltip-wave-height', wavePoint.height);
            setTooltipField('tooltip-wave-period', wavePoint.period, 'tooltip-wave-period-wrap');
            setTooltipField('tooltip-wave-dir', wavePoint.direction !== undefined ? Math.round(wavePoint.direction) : null, 'tooltip-wave-dir-wrap');
            totalRow.style.display = 'block';
        } else {
            totalRow.style.display = 'none';
        }
        
        // Wind waves
        const windWaveRow = document.getElementById('tooltip-wave-wind');
        if (wavePoint.wind_wave_height !== undefined) {
            setTooltipField('tooltip-wind-wave-height', wavePoint.wind_wave_height);
            setTooltipField('tooltip-wind-wave-period', wavePoint.wind_wave_period, 'tooltip-wind-wave-period-wrap');
            setTooltipField('tooltip-wind-wave-dir', wavePoint.wind_wave_dir !== undefined ? Math.round(wavePoint.wind_wave_dir) : null, 'tooltip-wind-wave-dir-wrap');
            windWaveRow.style.display = 'block';
        } else {
            windWaveRow.style.display = 'none';
        }
        
        // Swell
        const swellRow = document.getElementById('tooltip-wave-swell');
        if (wavePoint.swell_height !== undefined) {
            setTooltipField('tooltip-swell-height', wavePoint.swell_height);
            setTooltipField('tooltip-swell-period', wavePoint.swell_period, 'tooltip-swell-period-wrap');
            setTooltipField('tooltip-swell-dir', wavePoint.swell_dir !== undefined ? Math.round(wavePoint.swell_dir) : null, 'tooltip-swell-dir-wrap');
            swellRow.style.display = 'block';
        } else {
            swellRow.style.display = 'none';
        }
        
        // Peak period
        const peakRow = document.getElementById('tooltip-wave-peak');
        if (wavePoint.peak_period !== undefined) {
            setTooltipField('tooltip-peak-period', wavePoint.peak_period);
            peakRow.style.display = 'block';
        } else {
            peakRow.style.display = 'none';
        }
        
    } else {
        waveSection.style.display = 'none';
    }
    
    // Position tooltip near mouse but keep on screen
    const mapContainer = document.getElementById('map');
    const rect = mapContainer.getBoundingClientRect();
    let x = e.containerPoint.x + 15;
    let y = e.containerPoint.y + 15;
    
    // Keep tooltip on screen
    const tooltipRect = tooltip.getBoundingClientRect();
    if (x + tooltipRect.width > rect.width) {
        x = e.containerPoint.x - tooltipRect.width - 15;
    }
    if (y + tooltipRect.height > rect.height) {
        y = e.containerPoint.y - tooltipRect.height - 15;
    }
    
    tooltip.style.left = x + 'px';
    tooltip.style.top = y + 'px';
    tooltip.classList.remove('hidden');
}

/**
 * Hide tooltip when mouse leaves map
 */
function hideTooltip() {
    document.getElementById('tooltip').classList.add('hidden');
}

/**
 * Set up event listeners
 */
function setupEventListeners() {
    // Time slider
    document.getElementById('time-slider').addEventListener('input', async (e) => {
        currentTimeIndex = parseInt(e.target.value);
        const times = metadata.wind_times.length > 0 ? metadata.wind_times : metadata.wave_times;
        if (times[currentTimeIndex]) {
            updateTimeDisplay(times[currentTimeIndex]);
            await loadData();
        }
    });
    
    // Show/hide checkboxes
    document.getElementById('show-wind').addEventListener('change', () => {
        redraw();
        if (document.getElementById('show-wind').checked && windData) {
            startWindAnimation();
        }
    });
    document.getElementById('show-waves').addEventListener('change', redraw);
    
    // Particle density - reinitialize particles when changed
    document.getElementById('particle-density').addEventListener('input', () => {
        initParticles();
    });
    
    // Particle size slider - update displayed value
    document.getElementById('particle-size').addEventListener('input', (e) => {
        document.getElementById('particle-size-value').textContent = e.target.value;
    });
    
    // Tail length slider - update displayed value
    document.getElementById('tail-length').addEventListener('input', (e) => {
        document.getElementById('tail-length-value').textContent = e.target.value;
    });
    
    // Map hover for tooltip
    map.on('mousemove', updateTooltip);
    map.on('mouseout', hideTooltip);
}

/**
 * Initialize application
 */
async function init() {
    initMap();
    setupEventListeners();
    await fetchMetadata();
}

// Start when DOM is ready
document.addEventListener('DOMContentLoaded', init);
