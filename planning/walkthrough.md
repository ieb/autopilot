# ML Yacht Autopilot - Implementation Walkthrough

## Summary

I've implemented the core software architecture for an ML-based yacht autopilot for the Pogo 1250. The system uses an LSTM neural network to directly output rudder commands from sensor observations, learning from human helming behavior.

## Project Structure Created

```
autopilot/
├── README.md                    # Project documentation
├── requirements.txt             # Python dependencies
├── specification.md             # Original spec
├── polar.h                      # Polar data (JavaScript format)
├── planning/
│   └── implementation_plan.md   # Detailed design document
├── src/
│   ├── __init__.py
│   ├── main.py                  # Main application entry point
│   ├── sensors/
│   │   ├── imu_fusion.py        # 100Hz IMU via serial
│   │   ├── nmea2000_interface.py # 1Hz CAN bus
│   │   └── adc_reader.py        # 50Hz rudder ADC
│   ├── ml/
│   │   ├── polar.py             # Pogo 1250 polar diagram
│   │   ├── feature_engineering.py # 25 input features
│   │   └── autopilot_model.py   # LSTM model + inference
│   ├── training/
│   │   ├── data_loader.py       # CAN log parsing
│   │   └── train_imitation.py   # Supervised training
│   ├── control/
│   │   ├── rudder_controller.py # PWM H-Bridge driver
│   │   ├── mode_manager.py      # Operating modes
│   │   └── safety.py            # Limits and overrides
│   └── n2k/
│       └── __init__.py          # (N2K broadcast - future)
├── models/                      # For trained models
├── data/training/               # For training logs
└── tests/                       # Unit tests
```

---

## Key Modules Implemented

### Sensor Interfaces

| Module | Rate | Description |
|--------|------|-------------|
| [imu_fusion.py](file:///Users/ieb/timefields/antigravity/autopilot/src/sensors/imu_fusion.py) | 100Hz | Serial interface to ICM-20948 on dedicated MCU |
| [nmea2000_interface.py](file:///Users/ieb/timefields/antigravity/autopilot/src/sensors/nmea2000_interface.py) | 1Hz | CAN bus via CandleLite + SocketCAN |
| [adc_reader.py](file:///Users/ieb/timefields/antigravity/autopilot/src/sensors/adc_reader.py) | 50Hz | Rudder potentiometer via ADS1115 |

### ML Components

| Module | Description |
|--------|-------------|
| [polar.py](file:///Users/ieb/timefields/antigravity/autopilot/src/ml/polar.py) | Pogo 1250 polar with VMG optimization |
| [feature_engineering.py](file:///Users/ieb/timefields/antigravity/autopilot/src/ml/feature_engineering.py) | 25 normalized features with 2-second sliding window |
| [autopilot_model.py](file:///Users/ieb/timefields/antigravity/autopilot/src/ml/autopilot_model.py) | 2-layer LSTM (~45k params), TFLite inference |

### Control System

| Module | Description |
|--------|-------------|
| [rudder_controller.py](file:///Users/ieb/timefields/antigravity/autopilot/src/control/rudder_controller.py) | PWM H-Bridge driver with rate limiting |
| [mode_manager.py](file:///Users/ieb/timefields/antigravity/autopilot/src/control/mode_manager.py) | STANDBY, COMPASS, WIND_AWA, WIND_TWA, VMG modes |
| [safety.py](file:///Users/ieb/timefields/antigravity/autopilot/src/control/safety.py) | Limits, override detection, emergency stop |

### Training Pipeline

| Module | Description |
|--------|-------------|
| [data_loader.py](file:///Users/ieb/timefields/antigravity/autopilot/src/training/data_loader.py) | Parses candump/JSON/CSV logs, extracts PGNs |
| [train_imitation.py](file:///Users/ieb/timefields/antigravity/autopilot/src/training/train_imitation.py) | Supervised training with early stopping |

---

## ML Model Architecture

```
Input: [batch, 20, 25] → 20 timesteps × 25 features

┌─────────────────────────────────────────────┐
│  TimeDistributed(Dense(64, relu))           │  Feature mixing
├─────────────────────────────────────────────┤
│  LSTM(64, return_sequences=True)            │  Temporal processing
│  Dropout(0.2)                               │
├─────────────────────────────────────────────┤
│  LSTM(32, return_sequences=False)           │  Temporal processing
│  Dropout(0.2)                               │
├─────────────────────────────────────────────┤
│  Dense(16, relu)                            │  Output mapping
│  Dense(1, tanh)                             │  Bounded [-1, 1]
└─────────────────────────────────────────────┘

Output: [batch, 1] → rudder command normalized to [-1, +1]
```

**Parameters**: ~45,000  
**Inference target**: <10ms on Raspberry Pi 4

---

## Data Flow

```
┌─────────────┐   100Hz   ┌──────────────────┐
│ ICM-20948   │──────────▶│                  │
│ (via MCU)   │           │                  │
└─────────────┘           │                  │
                          │  Feature         │   10Hz   ┌──────────┐
┌─────────────┐    1Hz    │  Engineering     │─────────▶│          │
│ NMEA2000    │──────────▶│  (25 features    │          │  LSTM    │
│ (CAN bus)   │           │   × 20 steps)    │          │  Model   │
└─────────────┘           │                  │          │          │
                          │                  │          └────┬─────┘
┌─────────────┐   50Hz    │                  │               │
│ Rudder ADC  │──────────▶│                  │               │ 10Hz
└─────────────┘           └──────────────────┘               ▼
                                                      ┌──────────┐
                                                      │ Safety   │
                                                      │ Layer    │
                                                      └────┬─────┘
                                                           │
                                                           ▼ 50Hz
                                                      ┌──────────┐
                                                      │ Rudder   │
                                                      │ Control  │
                                                      └────┬─────┘
                                                           │
                                                           ▼
                                                      ┌──────────┐
                                                      │ H-Bridge │
                                                      │ → Motor  │
                                                      └──────────┘
```

---

## Next Steps

### 1. Training Data Required

The model needs logged sailing data that includes **rudder position**. The data loader supports:
- `candump -L` format logs
- JSON-lines format
- CSV format

**Minimum**: 10 hours of logged helming  
**Recommended**: 50+ hours across varied conditions

### 2. Hardware Setup

1. **IMU MCU**: Flash Arduino/RP2040 with Madgwick filter, output serial at 100Hz
2. **CAN Interface**: Configure CandleLite as `can0`
3. **H-Bridge wiring**: Connect to GPIO 18/19 for PWM

### 3. Training

```bash
# Once you have training data:
cd /Users/ieb/timefields/antigravity/autopilot
pip install -r requirements.txt
python -m src.training.train_imitation data/training/ --epochs 100
```

### 4. Testing

```bash
# Simulation mode (no hardware required):
python -m src.main --simulation --verbose
```

---

## Questions Remaining

1. **Log format**: What format are your existing CAN logs in? I've implemented candump, JSON, and CSV parsers, but may need to adjust for your specific format.

2. **Rudder logging**: Do your logs include rudder position? If not, we'll need to add rudder logging before collecting training data.

3. **IMU MCU firmware**: Do you need help with the Arduino/RP2040 firmware to run the Madgwick filter and output serial data?
