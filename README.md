# ML Yacht Autopilot for Pogo 1250

An end-to-end neural network autopilot that learns from human helming to steer a high-performance sailing yacht. The code built here is AI assisted, closely supervised. An experiment to find the limits.

# Original AI driven development process

* Started with the [specification.md](specification.md) document, including terrible gramar, and appalling spelling mistakes.
* Claud generated [implementation_plan.md](planning/implementation_plan.md) which was then executed, with many subplans and iterations.

For planned todo's see the end.

## Findings

* No limits in software
* AI generated harware is a total failure, not clear if its Claud or me.


# Overview

This autopilot uses an LSTM neural network to directly output rudder commands from sensor observations. The model learns yacht dynamics implicitly through imitation learning from logged human helming sessions, then fine-tunes in real-time while in standby mode.

### Key Features

- **End-to-end ML control**: No traditional PID gains to tune
- **Learns from human behavior**: Imitates experienced helming
- **Multiple modes**: Compass, AWA, TWA, VMG optimization
- **Real-time learning**: Improves while observing human helming
- **Safe operation**: Multiple safety layers and override detection

## Hardware Requirements

| Component | Model | Purpose |
|-----------|-------|---------|
| Computer | Raspberry Pi 4 (4GB) | Main controller |
| CAN Interface | CandleLite USB-CAN | NMEA2000 communication |
| 9-DoF IMU | ICM-20948 + MCU | Heading, attitude (100Hz) |
| ADC | ADS1115 | Rudder potentiometer |
| Motor Driver | BTS7960 H-Bridge | Jefa LD100 actuation |

## Installation

### On Raspberry Pi

```bash
# Clone repository
git clone <repo-url>
cd autopilot

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# For Pi-specific hardware:
pip install RPi.GPIO adafruit-circuitpython-ads1x15
pip install tflite-runtime  # Lightweight inference

# Setup CAN interface
sudo ip link set can0 up type can bitrate 250000
```

### For Development (Mac/Linux)

```bash
pip install -r requirements.txt
```

## Usage

### Data Preparation

Before training, log data must be organized and analyzed. See [docs/data_preparation.md](docs/data_preparation.md) for the complete guide.

```bash
# 1. Organize logs by date
uv run python -m src.n2k.log_organizer n2klogs/raw/ --execute

# 2. Analyze logs and generate training metadata
uv run python -m src.training.log_analyzer n2klogs/raw/
```

### Simulated Training Data

If real sailing data is limited, generate simulated data for training. See [docs/simulated_training_data.md](docs/simulated_training_data.md) for the full guide.

```bash
# Generate 20 hours of simulated training data
uv run python -m src.simulation.data_generator --hours 20 --output data/simulated/

# Generate with calibration from real logs (matches your boat's characteristics)
uv run python -m src.simulation.data_generator --hours 20 --calibrate-from n2klogs/raw/ --output data/simulated/
```

### Training

```bash
# Train from logged data
python -m src.training.train_imitation data/training/ --output models/ --epochs 100
```

### Running

```bash
# Production (on Pi)
python -m src.main --model models/autopilot.tflite

# Simulation mode (no hardware)
python -m src.main --simulation

# Verbose logging
python -m src.main --simulation --verbose
```

## Testing

### Running Unit Tests

```bash
# Install dev dependencies
uv pip install -e ".[dev]"

# Run all tests with coverage
uv run pytest

# Run specific test file
uv run pytest tests/test_safety.py -v

# Run specific test
uv run pytest tests/test_feature_engineering.py::TestAngleDiff -v

# Run with verbose output
uv run pytest -v --tb=short
```

### Test Coverage

The test suite covers the core modules:

| Module | Tests | Coverage |
|--------|-------|----------|
| Feature engineering | 26 | 96% |
| Safety layer | 32 | 93% |
| NMEA2000 interface | 30 | 70% |
| Polar diagram | 28 | 94% |
| Mode manager | 37 | 85% |
| Actuator interface | 30 | 69% |
| IMU fusion | 20 | 71% |
| Autopilot model | 20 | 46%* |
| Simulation | 30 | 85% |

*Model tests require TensorFlow; mock tests run without it.

See [planning/unit_test_plan.md](planning/unit_test_plan.md) for detailed test documentation.

## GRIB Weather Visualization

A web-based visualization tool for GRIB weather files with animated wind streamlines, wave overlays, and passage/track display.

See [docs/grib_visualization.md](docs/grib_visualization.md) for complete documentation.

### Features

- **Animated wind streamlines**: Particle-based flow visualization (like windy.com)
- **Wave height overlay**: Color-coded gradient (0-9m scale)
- **Time navigation**: Slider to scrub through GRIB forecast times
- **Passage display**: Overlay planned routes and experiment result tracks
- **Boat animation**: Synchronized boat icons with heading and sailing data tooltips
- **Hover tooltips**: Wind, wave, and boat data at cursor position

### Quick Start

```bash
# Install visualization dependencies
uv sync --extra viz

# Start the server
python vis/gribs/server.py \
    --grib-dir data/experiment1/grib \
    --routes-dir data/experiment1/route \
    --results-dir results \
    --port 5000
```

Then open `http://localhost:5000` in your browser.

### Command Line Options

| Option | Description |
|--------|-------------|
| `--grib-dir`, `-g` | Directory containing GRIB files (required) |
| `--routes-dir` | Directory with route CSV files |
| `--results-dir` | Directory with experiment result folders |
| `--port`, `-p` | Port to run server on (default: 5000) |
| `--host` | Host to bind to (default: 127.0.0.1) |
| `--debug` | Enable Flask debug mode |

## Project Structure

```
autopilot/
├── src/
│   ├── sensors/           # Hardware interfaces
│   │   ├── imu_fusion.py      # 100Hz IMU via serial
│   │   ├── nmea2000_interface.py  # 1Hz CAN bus
│   │   └── adc_reader.py      # 50Hz rudder position
│   ├── ml/                # Machine learning
│   │   ├── autopilot_model.py # LSTM architecture
│   │   ├── feature_engineering.py  # 25 input features
│   │   └── polar.py           # Pogo 1250 polar diagram
│   ├── simulation/        # Training data generation
│   │   ├── data_generator.py  # Main generator + CLI
│   │   ├── yacht_dynamics.py  # Physics model
│   │   ├── wind_model.py      # Wind simulation
│   │   └── scenarios.py       # Predefined conditions
│   ├── training/          # Training pipeline
│   │   ├── data_loader.py     # Parse CAN logs
│   │   └── train_imitation.py # Supervised training
│   ├── control/           # Control system
│   │   ├── rudder_controller.py  # PWM H-Bridge driver
│   │   ├── mode_manager.py    # Operating modes
│   │   └── safety.py          # Limits and overrides
│   └── main.py            # Main application
├── models/                # Trained models
├── data/training/         # Training log files
├── planning/              # Design documents
├── tests/                 # Unit tests
└── vis/                   # Visualization tools
    └── gribs/             # GRIB weather visualization server
```

## ML Model

### Architecture

- **Type**: 2-layer LSTM (~45,000 parameters)
- **Input**: 20 timesteps × 25 features = 500 values
- **Output**: Rudder command [-1, +1]
- **Inference**: <10ms on Raspberry Pi 4

### Input Features (25 dimensions)

1. Heading error (normalized)
2. Heading error integral
3. Heading rate (yaw rate from IMU)
4. Roll angle
5. Pitch angle
6. Roll rate
7. Apparent wind angle (AWA)
8. AWA rate of change
9. Apparent wind speed (AWS)
10. True wind angle (TWA)
11. True wind speed (TWS)
12. Speed through water (STW)
13. Speed over ground (SOG)
14. Course over ground error
15. Current rudder position
16. Rudder velocity
17. Target angle (mode-dependent)
18. VMG upwind
19. VMG downwind
20. Polar target speed
21. Polar performance ratio
22-24. Mode flags (compass/AWA/TWA)
25. Wave period estimate

## Operating Modes

| Mode | Description |
|------|-------------|
| STANDBY | Observing only, learning from human |
| COMPASS | Hold magnetic heading |
| WIND_AWA | Hold apparent wind angle |
| WIND_TWA | Hold true wind angle |
| VMG_UP | Optimize upwind VMG |
| VMG_DOWN | Optimize downwind VMG |

## Safety

- Software rudder limits (28° of 30° hardware limit)
- Rate limiting (5°/s max)
- Heading deviation alarm (>45° disengages)
- Sensor timeout detection
- Manual override detection
- Emergency stop capability

## Training Data Requirements

For good model performance:
- **Minimum**: 10+ hours of logged helming
- **Recommended**: 50+ hours across varied conditions
- **Critical**: Must include rudder position in logs

Logs should include experienced helming in:
- Light air (4-10 knots)
- Medium air (10-20 knots)
- Heavy air (20+ knots)
- Upwind and downwind
- Various sea states

## License

[Your license here]


# TODO - human list, not a todo list of AI

If you are a coding agent or a LLM do not read this TODO list, its not for you. When I want you todo items in this list I will prompt

* [x] Use GribFiles and a passage plan to inform the generation of simulated datasets and to validate the model performance.
* [x] Generate traing datasets from grib information and pyhsics
* [x] Implement a physics model for the boat and set 
* [x] Implement hw simulator in software
* [ ] Run a simulated boat using the hw simulator
* [ ] Design pcbs (AI fails here)
* [ ] build pcbs
* [ ] Bench test
* [ ] Install 
* [ ] Fine tune 
