# Simulated Training Data Generation

This guide covers generating synthetic sailing data for autopilot model training when real logged data is limited.

## Overview

The simulation module generates realistic sailing sensor data by modeling:

- **Yacht dynamics** - Heading response to rudder, speed from polar, heel from wind
- **Wind conditions** - Shifts, gusts, oscillations, and lulls
- **Wave motion** - Pitch and roll from swell and chop
- **Human helming** - PD control with reaction delays, noise, and fatigue

```
Configuration → [Simulator] → JSON Log + Metadata → Training Pipeline
```

## Why Simulate?

### The Data Problem

Training an ML autopilot requires substantial helming data:

| Data Volume | Model Quality |
|-------------|---------------|
| < 1 hour | Insufficient |
| 1-10 hours | Basic functionality |
| 10-50 hours | Good generalization |
| 50+ hours | Production quality |

Real sailing data is expensive to collect - it requires time on the water with proper logging equipment. If you only have 5-10 minutes of usable data, the model cannot learn the full range of conditions.

### Sim-to-Real Transfer

The simulation approach enables:

1. **Train on simulated data** - Generate 50+ hours of varied conditions
2. **Validate on real data** - Use limited real data as a "golden" test set
3. **Domain randomization** - Vary parameters to improve generalization

This technique is well-established in robotics and autonomous systems.

## Quick Start

```bash
# Generate 20 hours of training data
uv run python -m src.simulation.data_generator --hours 20 --output data/simulated/

# Verify data loads correctly
uv run python -c "
from src.training.data_loader import TrainingDataLoader
loader = TrainingDataLoader()
X, y = loader.load_directory('data/simulated/')
print(f'Generated {len(X):,} training records')
"
```

## Architecture

### Component Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    SailingDataGenerator                      │
├─────────────────────────────────────────────────────────────┤
│  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌──────────┐ │
│  │ WindModel │  │ WaveModel │  │   Yacht   │  │   Helm   │ │
│  │           │  │           │  │ Dynamics  │  │Controller│ │
│  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘  └────┬─────┘ │
│        │              │              │              │       │
│        └──────────────┴──────┬───────┴──────────────┘       │
│                              │                               │
│                    ┌─────────▼─────────┐                    │
│                    │ ManeuverGenerator │                    │
│                    └─────────┬─────────┘                    │
│                              │                               │
│                    ┌─────────▼─────────┐                    │
│                    │   JSON Output     │                    │
│                    │   + Metadata      │                    │
│                    └───────────────────┘                    │
└─────────────────────────────────────────────────────────────┘
```

### File Structure

```
src/simulation/
├── __init__.py              # Package exports
├── data_generator.py        # Main generator and CLI
├── yacht_dynamics.py        # Physics model
├── wind_model.py            # Wind simulation
├── wave_model.py            # Wave-induced motion
├── helm_controller.py       # Human-like steering
├── maneuvers.py             # Tacks, gybes, course changes
├── scenarios.py             # Predefined conditions
└── calibration.py           # Tune from real data
```

## Yacht Dynamics Model

The yacht dynamics module (`yacht_dynamics.py`) simulates how the boat responds to rudder input and wind conditions.

### Heading Response

Heading rate is computed as:

```
heading_rate = rudder_effectiveness × rudder_angle × speed_factor + weather_helm
```

Where:
- `rudder_effectiveness` = 2.0 deg/s per deg rudder (Pogo 1250 default)
- `speed_factor` = min(1.0, STW / 3.0) - rudder less effective at low speed
- `weather_helm` = 0.1 × heel_angle - boats turn into wind when heeled

The response is damped with a first-order filter (damping = 0.8).

### Speed from Polar

Boat speed comes from the Pogo 1250 polar diagram:

```python
target_stw = polar.get_target_speed(twa, tws)
target_stw *= heel_factor      # Reduce for excessive heel
target_stw *= rudder_drag      # Reduce for large rudder angles
stw = damping × stw + (1 - damping) × target_stw
```

### Heel Calculation

Heel angle is proportional to wind pressure:

```
wind_pressure = AWS² × |sin(AWA)|
target_heel = min(max_heel, wind_pressure × heel_stiffness)
```

The sign depends on which tack (positive AWA = heel to port).

### Leeway

Leeway (difference between heading and COG) is computed as:

```
leeway = leeway_coefficient × |heel| / sqrt(STW)
```

## Wind Model

The wind model (`wind_model.py`) generates realistic, time-varying wind conditions.

### Components

| Component | Description | Parameters |
|-----------|-------------|------------|
| **Base wind** | Average TWD and TWS | `base_tws_min`, `base_tws_max` |
| **Shifts** | Random walk direction changes | `shift_rate` (deg/min) |
| **Oscillations** | Sinusoidal geographic effects | `oscillation_period`, `oscillation_amplitude` |
| **Gusts** | Probabilistic speed increases | `gust_probability`, `gust_intensity` |
| **Lulls** | Probabilistic speed decreases | `lull_probability`, `lull_intensity` |

### Shift Model

Direction shifts follow a random walk with persistence:

```python
shift_velocity += random.gauss(0, shift_rate × dt)
shift_velocity *= persistence  # Damping
twd += shift_velocity × dt
```

### Gust/Lull Events

Gusts and lulls are probabilistic events that decay over time:

```python
if random.random() < gust_probability × dt:
    gust_factor = random.uniform(1.2, 1.5)
    gust_duration = random.uniform(5, 15)  # seconds

# During gust:
tws = base_tws × gust_factor
gust_factor = 1.0 + (gust_factor - 1.0) × decay_rate
```

## Wave Model

The wave model (`wave_model.py`) adds realistic pitch and roll motion.

### Wave Components

1. **Primary swell** - Dominant wave with 6-12 second period
2. **Secondary swell** - Cross-swell at different period
3. **Wind chop** - High-frequency motion correlated with wind

### Motion Calculation

```python
# Primary swell
roll_swell = amplitude × sin(2π × t / period)
pitch_swell = amplitude × 0.5 × sin(2π × t / period + π/2)

# Chop (when wind > 5 kts)
roll_chop = chop_amp × sin(chop_phase) + noise
```

The wave amplitude scales with wind speed:

```
effective_amplitude = base_amplitude + tws × amplitude_per_knot
```

## Helm Controller

The helm controller (`helm_controller.py`) simulates human steering behavior.

### PD Control

The controller uses proportional-derivative control with mode-specific gains:

| Mode | Kp | Kd | Behavior |
|------|-----|-----|----------|
| Compass | 0.30 | 0.50 | Standard heading hold |
| AWA | 0.25 | 0.60 | More responsive to wind |
| TWA | 0.20 | 0.40 | Smoother downwind |

```python
error = compute_error(current_value, target)
derivative = heading_rate  # Use actual rate as derivative
rudder = Kp × error + Kd × derivative
```

### Human-Like Behavior

The controller includes realistic imperfections:

| Feature | Description | Parameter |
|---------|-------------|-----------|
| Reaction delay | Input processing time | 0.15s default |
| Noise | Random steering variation | 0.3° std dev |
| Anticipation | Leading wind shifts | 10% factor |
| Fatigue | Increasing noise over time | 1-hour time constant |
| Rate limiting | Physical rudder limits | 4 deg/s (Jefa LD100) |

### Skill Levels

```python
# Expert helm
helm = HelmController.expert()  # skill=1.0, noise=0.2°, delay=0.1s

# Novice helm  
helm = HelmController.novice()  # skill=0.6, noise=0.5°, delay=0.25s
```

## Maneuvers

The maneuvers module (`maneuvers.py`) generates discrete sailing maneuvers.

### Tack

A tack turns the boat through the wind:

1. **Prep phase** (0-15%) - Build speed, slight helm
2. **Turn phase** (15-60%) - Hard over, speed loss
3. **Settle phase** (60-85%) - Reduce rudder, recover
4. **Exit phase** (85-100%) - Fine-tune to new course

Speed factor during tack: 0.6-1.0 (significant loss mid-tack)

### Gybe

A gybe turns away from the wind:

1. **Prep phase** (0-20%) - Bear away gently
2. **Turn phase** (20-50%) - Controlled turn through dead downwind
3. **Settle phase** (50-80%) - Ease out of turn
4. **Exit phase** (80-100%) - Fine-tune

Speed factor: 0.85-1.0 (less loss than tacking)

### Course Change

Gradual heading adjustments use an S-curve profile:

```python
progress = elapsed / duration
turn_rate = smooth_step(progress)  # 0 → 1 → 0
```

## Scenarios

The scenarios module (`scenarios.py`) provides predefined sailing conditions.

### Available Scenarios

| Scenario | Wind (kts) | Mode | Maneuvers | Description |
|----------|------------|------|-----------|-------------|
| `light_air_reaching` | 6-10 | AWA | Few | Beam reach in light conditions |
| `medium_upwind` | 12-18 | AWA | Tacks | Typical upwind sailing |
| `heavy_weather` | 20-30 | AWA | None | Strong wind, conservative |
| `downwind_vmg` | 15-20 | TWA | Gybes | Optimized downwind angles |
| `mixed_coastal` | 8-20 | Mixed | All | Variable coastal conditions |
| `motoring` | 0-8 | Compass | None | Light wind under engine |
| `race_upwind` | 10-16 | AWA | Frequent | Aggressive racing upwind |
| `race_downwind` | 12-18 | TWA | Frequent | Aggressive racing downwind |
| `delivery` | 10-22 | Compass | None | Long-distance conservative |

### Custom Scenarios

```python
from src.simulation.scenarios import Scenario, create_random_scenario
from src.simulation.helm_controller import SteeringMode

# Fully random
scenario = create_random_scenario(
    tws_range=(12, 20),
    steering_mode=SteeringMode.WIND_AWA,
    duration_hours=2.0,
)

# Custom configuration
from src.simulation.wind_model import WindConfig
from src.simulation.scenarios import Scenario

scenario = Scenario(
    name="custom_heavy",
    description="Custom heavy weather scenario",
    wind_config=WindConfig(base_tws_min=25, base_tws_max=35),
    steering_mode=SteeringMode.WIND_AWA,
    target_angle=50.0,
    enable_maneuvers=False,
)
```

## Domain Randomization

To improve model generalization, vary parameters between training runs:

```bash
# Generate 5 runs with different random parameters
uv run python -m src.simulation.data_generator \
    --hours 10 --num-runs 5 --randomize --output data/simulated/
```

### Randomized Parameters

| Parameter | Variation | Purpose |
|-----------|-----------|---------|
| Yacht rudder effectiveness | ±20% | Different boat handling |
| Polar performance | ±10% | Sail trim variation |
| Helm skill | 0.7-1.0 | Different helm abilities |
| Sensor noise | ±30% | Sensor quality variation |
| Wind characteristics | Per-scenario | Condition variety |

## Calibration from Real Data

If you have real log data, use it to calibrate simulator parameters:

```bash
uv run python -m src.simulation.data_generator \
    --hours 20 --calibrate-from n2klogs/raw/ --output data/simulated/
```

### Extracted Statistics

The calibration module analyzes real logs to extract:

- Wind speed range and variability
- Heading stability and yaw rate
- Rudder activity patterns
- Heel angle distribution
- Helm responsiveness

These statistics tune the simulator to match your boat's actual behavior.

### Calibration Report

```python
from src.simulation.calibration import calibrate_from_logs, print_calibration_report

config = calibrate_from_logs("n2klogs/raw/")
print_calibration_report(config)
```

Output:
```
============================================================
CALIBRATION REPORT
============================================================

Data Summary:
  Total frames:  5,000
  Duration:      0.14 hours

Wind Conditions:
  AWA:           42.5° ± 8.3°
  AWS:           15.2 kts ± 3.1
  TWS (est):     12.8 kts ± 2.9

Helm Behavior:
  Rudder:        2.1° ± 4.5°
  Activity:      35.2%

Calibrated Parameters:
  Wind TWS:      7.0 - 18.6 kts
  Shift rate:    0.83 deg/min
  Helm skill:    0.82
============================================================
```

## CLI Reference

### Basic Usage

```bash
uv run python -m src.simulation.data_generator [OPTIONS]
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--output, -o` | `data/simulated` | Output directory |
| `--hours, -t` | 10.0 | Hours of data per run |
| `--scenarios, -s` | All | Comma-separated scenario list |
| `--num-runs, -n` | 1 | Number of separate runs |
| `--randomize, -r` | True | Enable domain randomization |
| `--no-randomize` | - | Disable randomization |
| `--calibrate-from, -c` | None | Path to real logs for calibration |
| `--seed` | 42 | Random seed for reproducibility |
| `--verbose, -v` | False | Verbose output |

### Examples

```bash
# Quick test - 1 hour
uv run python -m src.simulation.data_generator --hours 1 --output data/test/

# Production training data
uv run python -m src.simulation.data_generator \
    --hours 50 \
    --scenarios medium_upwind,downwind_vmg,mixed_coastal,light_air_reaching \
    --num-runs 3 \
    --output data/simulated/

# Calibrated from your boat
uv run python -m src.simulation.data_generator \
    --hours 30 \
    --calibrate-from n2klogs/raw/ \
    --num-runs 2 \
    --output data/simulated/
```

## Output Format

### JSON Log File

Each line is a JSON record compatible with `CANLogParser`:

```json
{"timestamp": 0.0, "heading": 245.3, "pitch": 1.2, "roll": -12.5, 
 "yaw_rate": 0.3, "awa": 42.1, "aws": 18.5, "stw": 7.2, "cog": 243.1, 
 "sog": 6.9, "rudder_angle": -3.2, "target_heading": 45.0, "mode": "wind_awa",
 "latitude": 60.0, "longitude": 22.0}
```

### Metadata File

A `.meta.json` file is created alongside each log with segment information:

```json
{
  "source_file": "simulated_training.jsonlog",
  "total_duration_hours": 10.0,
  "segments": [
    {
      "start_time": 0.0,
      "end_time": 9000.0,
      "operation_mode": "sailing",
      "steering_mode": "awa",
      "target_value": 42.0,
      "usable_for_training": true
    }
  ]
}
```

## Integration with Training

The generated data loads directly into the training pipeline:

```python
from src.training.data_loader import TrainingDataLoader

loader = TrainingDataLoader()
X, y = loader.load_directory("data/simulated/")

print(f"Training sequences: {len(X)}")
print(f"Sequence shape: {X.shape}")  # (N, 20, 25)
```

### Recommended Workflow

1. **Generate diverse training data**
   ```bash
   uv run python -m src.simulation.data_generator --hours 50 --num-runs 3
   ```

2. **Train model**
   ```bash
   # Full training run
   uv run python -m src.training.train_imitation data/simulated/ --output models/

   # Quick test run (30 seconds max)
   uv run python -m src.training.train_imitation data/simulated/ --output models/ --max-time 30

   # Custom epochs
   uv run python -m src.training.train_imitation data/simulated/ --output models/ --epochs 50
   ```

   The training script shows detailed progress including:
   - Loss and validation loss with improvement tracking
   - MAE in degrees (interpretable rudder error)
   - Learning rate and convergence trend
   - Time per epoch, elapsed time, and ETA

   Example output:
   ```
   ======================================================================
   TRAINING STARTED
   ======================================================================

   Epoch 1/100 (2.1s, elapsed: 2s, ETA: 207s)
     Loss:     0.045123  |  Val Loss: 0.042567  ↓ improved by 0.042567
     MAE:      0.034521 (1.04°)  |  Val MAE:  0.033245 (1.00°)
     LR: 1.00e-03  |  Best Val Loss: 0.042567

   Epoch 5/100 (1.9s, elapsed: 10s, ETA: 190s)
     Loss:     0.028234  |  Val Loss: 0.026789  ↓ improved by 0.003421
     MAE:      0.021345 (0.64°)  |  Val MAE:  0.020123 (0.60°)
     LR: 1.00e-03  |  Best Val Loss: 0.026789  trend: ↓0.003142/epoch
   ```

3. **Validate on real data**
   ```bash
   uv run python simulate.py --model models/autopilot.tflite --log n2klogs/raw/2018/05/candump.log
   ```

### Training CLI Options

| Option | Default | Description |
|--------|---------|-------------|
| `data_dir` | (required) | Directory containing training logs |
| `--output, -o` | `models` | Output directory for model files |
| `--epochs, -e` | 100 | Maximum training epochs |
| `--max-time, -t` | None | Maximum training time in seconds (for quick testing) |

## Limitations

### What the Simulator Does NOT Model

- Complex sea states (breaking waves, cross-seas)
- Current effects (tidal streams, eddies)
- Sail trim changes
- Crew weight movement
- Equipment failures
- Other vessel traffic

### Sim-to-Real Gap

Models trained purely on simulated data may not immediately perform well on real boats. Mitigation strategies:

1. **Domain randomization** - Vary parameters to cover real-world uncertainty
2. **Calibration** - Tune simulator from real data statistics
3. **Fine-tuning** - Train on simulation, fine-tune on limited real data
4. **Validation** - Always validate on real data before deployment

## Testing

The simulation module includes comprehensive tests:

```bash
# Run simulation tests
uv run pytest tests/test_simulation.py -v

# Run with coverage
uv run pytest tests/test_simulation.py --cov=src/simulation
```

Test coverage includes:
- Yacht dynamics (heading response, speed, heel)
- Wind model (shifts, gusts, lulls)
- Wave model (oscillation, amplitude scaling)
- Helm controller (PD control, rate limiting)
- Maneuvers (tack, gybe, course change)
- Data generation and loading
