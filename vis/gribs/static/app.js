/**
 * GRIB Weather Visualization
 * 
 * Displays wind arrows and wave height gradient overlay on OpenSeaMap.
 */

// Global state
let map = null;
let windCanvas = null;
let waveCanvas = null;
let windData = null;
let waveData = null;
let metadata = null;
let currentTimeIndex = 0;

// Color scales
const WIND_COLORS = [
    { max: 10, color: '#3498db' },  // Blue - light
    { max: 20, color: '#2ecc71' },  // Green - moderate
    { max: 30, color: '#f1c40f' },  // Yellow - fresh
    { max: 40, color: '#e67e22' },  // Orange - strong
    { max: Infinity, color: '#e74c3c' }  // Red - gale
];

const WAVE_COLORS = [
    { max: 1, r: 52, g: 152, b: 219 },   // Blue
    { max: 2, r: 46, g: 204, b: 113 },   // Green
    { max: 3, r: 241, g: 196, b: 15 },   // Yellow
    { max: 4, r: 230, g: 126, b: 34 },   // Orange
    { max: Infinity, r: 231, g: 76, b: 60 }  // Red
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
        
        if (document.getElementById('show-wind').checked) {
            drawWind();
        } else {
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
 * Get color for wave height
 */
function getWaveColor(height) {
    for (const scale of WAVE_COLORS) {
        if (height < scale.max) {
            return { r: scale.r, g: scale.g, b: scale.b };
        }
    }
    return WAVE_COLORS[WAVE_COLORS.length - 1];
}

/**
 * Draw wind arrows
 */
function drawWind() {
    if (!windCanvas || !windData || !windData.points) return;
    
    const ctx = windCanvas.getContext('2d');
    ctx.clearRect(0, 0, windCanvas.width, windCanvas.height);
    
    const bounds = map.getBounds();
    const zoom = map.getZoom();
    
    // Get density setting (1-5, lower = more arrows)
    const densitySetting = parseInt(document.getElementById('arrow-density').value);
    const skip = densitySetting;
    
    // Calculate step based on grid
    if (!windData.bounds || !windData.grid_size) return;
    
    const latStep = (windData.bounds.lat_max - windData.bounds.lat_min) / (windData.grid_size.rows - 1);
    const lonStep = (windData.bounds.lon_max - windData.bounds.lon_min) / (windData.grid_size.cols - 1);
    
    // Filter points by skip pattern
    let rowCount = 0;
    let lastLat = null;
    let colCount = 0;
    
    // Sort points for consistent skip pattern
    const sortedPoints = [...windData.points].sort((a, b) => {
        if (a.lat !== b.lat) return b.lat - a.lat;  // Descending lat
        return a.lon - b.lon;  // Ascending lon
    });
    
    for (const point of sortedPoints) {
        // Track row changes
        if (lastLat === null || Math.abs(point.lat - lastLat) > latStep * 0.5) {
            rowCount++;
            colCount = 0;
            lastLat = point.lat;
        }
        colCount++;
        
        // Skip based on density
        if (rowCount % skip !== 0 || colCount % skip !== 0) {
            continue;
        }
        
        // Check if point is in view
        if (point.lat < bounds.getSouth() || point.lat > bounds.getNorth() ||
            point.lon < bounds.getWest() || point.lon > bounds.getEast()) {
            continue;
        }
        
        // Get pixel position
        const pos = map.latLngToContainerPoint([point.lat, point.lon]);
        
        // Draw arrow
        drawWindArrow(ctx, pos.x, pos.y, point.speed, point.direction, zoom);
    }
}

/**
 * Draw a single wind arrow
 */
function drawWindArrow(ctx, x, y, speed, direction, zoom) {
    // Arrow size scales with zoom and speed
    const baseSize = Math.min(15 + zoom * 2, 40);
    const length = baseSize * Math.min(speed / 20, 2);
    const headSize = length * 0.3;
    
    // Get color based on speed
    const color = getWindColor(speed);
    
    // Convert meteorological direction (FROM) to direction wind is blowing TO
    // Canvas rotation: 0° = down (south), 90° = left (west), 180° = up (north)
    // To point arrow in compass direction D, canvas needs rotation (D + 180)
    // Wind goes TO = (direction + 180), so canvas rotation = (direction + 180 + 180) = direction
    const angle = direction * Math.PI / 180;
    
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);
    
    // Draw arrow shaft
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, -length / 2);
    ctx.lineTo(0, length / 2);
    ctx.stroke();
    
    // Draw arrow head
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(0, length / 2);
    ctx.lineTo(-headSize / 2, length / 2 - headSize);
    ctx.lineTo(headSize / 2, length / 2 - headSize);
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
}

/**
 * Get color for wind speed
 */
function getWindColor(speed) {
    for (const scale of WIND_COLORS) {
        if (speed < scale.max) {
            return scale.color;
        }
    }
    return WIND_COLORS[WIND_COLORS.length - 1].color;
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
    document.getElementById('show-wind').addEventListener('change', redraw);
    document.getElementById('show-waves').addEventListener('change', redraw);
    
    // Arrow density
    document.getElementById('arrow-density').addEventListener('input', redraw);
    
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
