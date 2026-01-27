# GRIB Weather Visualization

A web-based tool for visualizing GRIB weather data with animated wind streamlines, wave overlays, and passage/track display.

## Features

### Weather Visualization
- **Animated wind streamlines**: Particle-based flow visualization showing wind speed and direction
- **Wave height overlay**: Color-coded transparent gradient (0-9m scale)
- **Time navigation**: Slider to scrub through GRIB forecast times
- **Hover tooltips**: Detailed wind and wave data at cursor position

### Passage Display
- **Planned routes**: Load and display route CSV files with waypoint markers
- **Experiment results**: Overlay multiple result tracks for comparison
- **Animated boats**: Triangle icons showing current position and heading
- **Time synchronization**: Passage slider syncs with weather data

### Interactive Controls
- **Layer toggles**: Show/hide wind particles and wave overlay
- **Particle customization**: Adjust density, size, tail length, and speed
- **Multiple result tracks**: Select and compare different experiment runs

## Running the Server

### Prerequisites

```bash
# Install visualization dependencies
uv sync --extra viz
```

### Basic Usage

```bash
python vis/gribs/server.py --grib-dir <path-to-grib-files>
```

### Full Options

```bash
python vis/gribs/server.py \
    --grib-dir data/experiment1/grib \
    --routes-dir data/experiment1/route \
    --results-dir results \
    --port 5000 \
    --debug
```

| Option | Description | Default |
|--------|-------------|---------|
| `--grib-dir` | Directory containing GRIB files (.grb, .grb2, .grb.bz2) | Required |
| `--routes-dir` | Directory containing route CSV files | None |
| `--results-dir` | Directory containing experiment result folders | None |
| `--port` | Server port | 5000 |
| `--debug` | Enable Flask debug mode | False |

### Access

Open http://localhost:5000 in a browser (Chrome recommended).

## GRIB File Support

### Supported Parameters

**Wind:**
- `u10` / `10u` - 10m U-component (eastward)
- `v10` / `10v` - 10m V-component (northward)

**Waves (total):**
- `swh` - Significant wave height
- `mwp` - Mean wave period
- `mwd` - Mean wave direction

**Wind waves:**
- `shww` - Significant height of wind waves
- `mpww` - Mean period of wind waves
- `mdww` - Mean direction of wind waves

**Swell:**
- `shts` - Significant height of total swell
- `mpts` - Mean period of total swell
- `mdts` - Mean direction of total swell

**Peak period:**
- `pp1d` - Peak wave period
- `mp2` - Mean zero-crossing wave period

### File Formats

- `.grb` / `.grb2` - Standard GRIB files
- `.grb.bz2` - BZ2 compressed GRIB files (auto-decompressed)

## API Endpoints

### Metadata

```
GET /api/metadata
```

Returns GRIB file information including available times and data bounds.

### Wind Data

```
GET /api/wind?time=2024-01-26T12:00:00
```

Returns wind grid with u/v components, speed (knots), and direction (degrees FROM).

### Wave Data

```
GET /api/waves?time=2024-01-26T12:00:00
```

Returns wave data including height, period, and direction for total sea, wind waves, and swell.

### Routes

```
GET /api/routes          # List available routes
GET /api/route/<name>    # Get route waypoints
```

### Results

```
GET /api/results         # List available results
GET /api/result/<name>   # Get track time series
```

### Debug

```
GET /api/debug/grib
```

Lists all parameters found in the loaded GRIB files (useful for debugging).

## User Interface

### Map Controls

- **Pan**: Click and drag
- **Zoom**: Scroll wheel or +/- buttons
- **Hover**: Shows wind/wave data tooltip

### Weather Controls (left panel)

- **Show Wind**: Toggle animated wind particles
- **Show Waves**: Toggle wave height overlay
- **Time Slider**: Navigate through forecast times
- **Particle Density**: Number of wind particles
- **Particle Size**: Line width of particles
- **Tail Length**: Trail fade duration
- **Particle Speed**: Animation speed multiplier

### Passage Controls (second panel)

- **Route**: Select a planned route to display
- **Results**: Check result tracks to overlay
- **Show Route**: Toggle route polyline visibility
- **Passage Time**: Scrub through the passage timeline
- **Play/Pause**: Animate boat positions
- **Reset**: Return to start of passage

### Boat Tooltips

Hover over boat icons to see:
- **HDG**: Current heading → Target heading
- **SOG/STW**: Speed over ground / through water
- **AWA/AWS**: Apparent wind angle and speed

## Technical Details

### Wind Particle System

The wind visualization uses a particle system inspired by windy.com:

1. Particles spawn at random positions within data bounds
2. Each frame, particle velocity is interpolated from nearest GRIB grid point
3. Particle speed is proportional to wind speed
4. Trails are created using canvas composite operations
5. Particles respawn when leaving bounds or reaching max age

### Coordinate Systems

- **GRIB data**: Meteorological convention (direction wind is FROM, 0°=N, 90°=E)
- **Display**: Particles flow in the direction wind is blowing (TO)
- **Canvas**: Y-axis inverted (positive = down), handled in velocity calculation

### Time Synchronization

The passage slider and GRIB slider are linked:
- Moving passage slider → GRIB time updates to closest available forecast
- Moving GRIB slider → Independent (allows viewing different forecast times)

## Troubleshooting

### "Ran out of input" Error

Corrupted cfgrib index file. Delete the cache:

```bash
rm -f /var/folders/*/T/*.grb.*.idx
```

### No Wind Particles Visible

1. Check "Show Wind" is enabled
2. Verify GRIB file contains u10/v10 data
3. Zoom/pan to the data coverage area
4. Increase particle density

### Missing Wave Parameters

Not all GRIB files contain all wave parameters. Use the debug endpoint to see what's available:

```
GET /api/debug/grib
```

### Boat Tooltip Missing AWA/AWS

If the result data doesn't include AWA/AWS columns, they're calculated from:
- TWS (true wind speed)
- TWA (true wind angle)
- STW (speed through water)
