# ML Yacht Autopilot for Pogo 1250

An end-to-end neural network autopilot that learns from human helming to steer a high-performance sailing yacht.

## Overview

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
└── tests/                 # Unit tests
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
