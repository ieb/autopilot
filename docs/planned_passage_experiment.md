# Planned Passage Experiment

This document describes how to run the planned passage following experiment, which tests the autopilot's ability to follow a pre-planned sailing route.

## Overview

The experiment simulates a yacht following a weather-routed passage plan. It uses:

- **Route data**: CSV file with waypoints, timing, and predicted conditions
- **Weather data**: GRIB files with wind and wave forecasts (optional)
- **Autopilot**: Either the trained ML model or a baseline PD controller

The simulation tracks performance metrics including arrival time accuracy, cross-track error, polar performance, and course accuracy.

## Prerequisites

### Install Dependencies

The experiment requires additional dependencies for GRIB file support:

```bash
# Install experiment dependencies
uv pip install cfgrib xarray eccodes

# Or install the experiment optional group
uv pip install -e ".[experiment]"
```

**Note**: GRIB support is optional. Without it, the experiment falls back to using wind/wave predictions from the route file.

### Data Files

The experiment requires:

1. **Route CSV file** - Exported from weather routing software (e.g., Expedition, qtVlm)
2. **GRIB files** (optional) - Wind and wave forecasts covering the route area and time

Example data is provided in:
- `data/experiment1/route/wr_route_1_20260125_005338.csv`
- `data/experiment1/gribs/`

## Running the Experiment

### Basic Usage (Baseline Controller)

Run with the baseline PD controller to establish a reference:

```bash
uv run python -m experiments.experiment1.run_experiment \
  --route data/experiment1/route/wr_route_1_20260125_005338.csv \
  --baseline \
  --output results/experiment1/
```

### With GRIB Weather Data

Include GRIB data for more realistic wind/wave conditions:

```bash
uv run python -m experiments.experiment1.run_experiment \
  --route data/experiment1/route/wr_route_1_20260125_005338.csv \
  --gribs data/experiment1/gribs/ \
  --baseline \
  --output results/experiment1/
```

### With Trained ML Model

Test the trained autopilot model:

```bash
uv run python -m experiments.experiment1.run_experiment \
  --route data/experiment1/route/wr_route_1_20260125_005338.csv \
  --gribs data/experiment1/gribs/ \
  --model models/autopilot_best.onnx \
  --output results/experiment/
```

### Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `--route`, `-r` | Path to route CSV file | Required |
| `--gribs`, `-g` | Path to GRIB directory | None |
| `--model`, `-m` | Path to trained model | None |
| `--output`, `-o` | Output directory for results | `results/experiment1` |
| `--baseline`, `-b` | Use baseline controller | False |
| `--max-hours` | Maximum simulation duration | 24.0 |
| `--dt` | Simulation time step (seconds) | 0.1 |
| `--verbose`, `-v` | Enable verbose logging | False |

## Output Files

The experiment generates three output files:

### 1. `summary.json`

JSON file with aggregate metrics:

```json
{
  "planned_duration_sec": 49358.0,
  "actual_duration_sec": 48793.0,
  "eta_error_sec": -565.0,
  "eta_error_percent": -1.1,
  "mean_xte_nm": 0.1167,
  "max_xte_nm": 1.6436,
  "mean_polar_performance": 1.001,
  "rms_heading_error_deg": 3.0
}
```

### 2. `time_series.csv`

Detailed time-series data for analysis and plotting:

| Column | Description |
|--------|-------------|
| timestamp | Elapsed time (seconds) |
| latitude, longitude | Position |
| heading | Current heading (degrees) |
| target_heading | Target heading (degrees) |
| heading_error | Heading deviation (degrees) |
| stw, sog | Speed through water / over ground (knots) |
| tws, twd, twa | True wind speed, direction, angle |
| cross_track_error | Distance from planned track (nm) |
| polar_performance | Ratio to polar target speed |
| rudder_angle | Current rudder position (degrees) |
| steering_mode | Current mode (wind_awa, wind_twa, compass, motoring) |
| leg_index | Current route leg |

### 3. `report.md`

Human-readable performance report with:
- Timing summary
- Cross-track error statistics
- Polar performance analysis
- Course accuracy metrics
- Steering mode distribution

## Performance Metrics

### ETA Error

Difference between actual arrival time and planned arrival time:
- Negative = arrived early
- Positive = arrived late

### Cross-Track Error (XTE)

Perpendicular distance from the planned track line:
- Measured in nautical miles
- Mean, max, and RMS values reported

### Polar Performance

Ratio of actual boat speed to polar target speed:
- 100% = sailing at polar speed
- <100% = below polar
- >100% = exceeding polar (favorable conditions)

### Course Accuracy

RMS heading error relative to target:
- Lower is better
- High values may indicate control instability or challenging conditions

## Route File Format

The experiment expects a CSV file with the following columns:

```
Isochrone start time,From Latitude,From Longitude,To Latitude,To Longitude,
Course Over Ground,Speed Over Ground,Distance Over Ground,True Wind Speed,
True Wind Direction,Surface Wind Angle,Distance to Finish,Is motoring,...
```

Coordinates can be in decimal degrees or DMS format (e.g., `51°56.578'N`).

## Steering Mode Logic

The navigator selects steering mode based on true wind angle:

| TWA Range | Steering Mode | Description |
|-----------|---------------|-------------|
| 0-30° | AWA | Close-hauled, steering by apparent wind |
| 30-120° | AWA | Reaching, steering by apparent wind |
| 120-180° | TWA | Running, steering by true wind |
| Motoring | COG | Hold course over ground at 6 knots |

## Example Results

Baseline controller results for the test route:

```
PASSAGE SIMULATION RESULTS
============================================================

Timing:
  Planned:  13:42:38
  Actual:   13:33:13
  Error:    -565s (-1.1%)

Cross-Track Error:
  Mean: 0.1167 nm (216 m)
  Max:  1.6436 nm (3044 m)

Polar Performance:
  Mean: 100.1% (± 40.9%)

Course Accuracy:
  RMS Error: 3.0°
  Max Error: 107.5°

============================================================
```

## Exporting Track to GPX

After running the experiment, you can export the simulated track to GPX format for viewing on OpenSeaMap or other mapping applications.

### Basic Export

```bash
uv run python -m experiments.experiment1.export_track results/experiment1/time_series.csv
```

This creates `time_series.gpx` in the same directory.

### Export with Route Waypoints

Include the planned waypoints in the GPX file:

```bash
uv run python -m experiments.experiment1.export_track \
  results/experiment1/time_series.csv \
  --route data/experiment1/route/wr_route_1_20260125_005338.csv \
  -o passage_track.gpx
```

### Export Options

| Option | Description | Default |
|--------|-------------|---------|
| `-o`, `--output` | Output GPX file path | Same as input with .gpx |
| `--route` | Route CSV to include waypoints | None |
| `--name` | Track name in GPX | "Simulated Passage" |
| `--start-time` | Start time (ISO format) | Current time |
| `--sample-interval` | Sample interval in seconds | 60 |

### Viewing on OpenSeaMap

1. Go to [GPS Visualizer](https://www.gpsvisualizer.com/)
2. Upload the GPX file
3. Select "OpenSeaMap" as the base map
4. Click "Draw the map"

Alternatively, use any GPX-compatible application like:
- OpenCPN (free nautical chart plotter)
- Navionics (mobile app)
- QGIS with OpenSeaMap tiles

## Troubleshooting

### "cfgrib/xarray not available"

Install the experiment dependencies:
```bash
uv pip install cfgrib xarray eccodes
```

### GRIB files not loading

- Ensure files are in `.grb` or `.grb.bz2` format
- Check that eccodes is properly installed
- Try running with `--verbose` to see detailed error messages

### Simulation runs too slow

- Increase `--dt` to 0.2 or 0.5 for faster (less accurate) simulation
- Reduce `--max-hours` to limit simulation duration

### Model not loading

- Verify the model file exists and is a valid `.keras` or `.tflite` file
- The experiment will fall back to baseline controller if model fails to load
