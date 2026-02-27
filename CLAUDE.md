# Autopilot - Claude Code Project Guide

## Project Overview

ML yacht autopilot for a Pogo 1250. LSTM model trained via imitation learning from simulated expert helming. Outputs rudder commands from sensor observations. Target deployment: Raspberry Pi 4 with ONNX runtime.

## Quick Commands

```bash
# Run all tests
uv run python -m pytest tests/ -x -q

# Run specific test file
uv run python -m pytest tests/test_simulation.py -x -q

# Generate training data
uv run python -u -m src.simulation.data_generator data/simulated --format binary 2>&1

# Train model
uv run python -u -m src.training.train_imitation data/simulated -o models 2>&1

# Run CL validation on saved model
uv run python -u -m src.training.train_imitation --validate models/autopilot.onnx 2>&1

# Install/sync dependencies
uv sync --extra dev
```

## NMEA2000 Sign Conventions

**Never change these without understanding the full system impact.**

| Signal | Sign | Meaning |
|--------|------|---------|
| Rudder angle | Positive | Starboard (right) |
| Rudder angle | Negative | Port (left) |
| AWA/TWA | Positive | Wind from starboard |
| AWA/TWA | Negative | Wind from port |
| Heading | 0-360° | Clockwise from North |

Physical effects:
- **Positive rudder** -> boat turns starboard -> heading increases -> AWA decreases
- **Negative rudder** -> boat turns port -> heading decreases -> AWA increases

Error convention (HelmController aligned):
```python
# Compass: positive error = target to starboard
error = target_heading - current_heading
command = kp * error  # Positive error -> positive rudder

# Wind modes (AWA/TWA):
error = current_awa - target_awa  # Positive = wind too far aft
command = kp * error  # Positive error -> positive rudder (head up)
```

The `TestNMEA2000Conventions` class in `tests/test_simulation.py` validates these conventions. **These tests must pass and should not be modified to make other tests pass.** See `docs/helm_controller_bug_fix.md` for history of sign convention bugs.

## Architecture

### Source Layout (`src/`)

- **`ml/`** - Model architecture and inference
  - `autopilot_model.py` - LSTM model (255k params: 128->64 LSTM, 32 dense)
  - `feature_engineering.py` - Real-time feature computation (must match data_loader)
  - `polar.py` - Yacht polar diagram

- **`training/`** - Training pipeline
  - `data_loader.py` - Feature computation (single source of truth for 22-feature vector)
  - `train_imitation.py` - Training loop, CL validation, loss weighting

- **`simulation/`** - Data generation
  - `data_generator.py` - Scenario runner, binary data writer
  - `yacht_dynamics.py` - Physics model
  - `scenarios.py` - Predefined sailing scenarios
  - `helm_controller.py` - Expert PD controller (kp=1.6, kd=1.5)
  - `wave_model.py`, `wind_model.py` - Environment models

- **`control/`** - Real-time control (rudder controller, mode manager, safety)
- **`sensors/`** - IMU fusion, NMEA2000 interface, ADC reader
- **`n2k/`** - NMEA2000 data logging

### Key Conventions

- **Feature vector**: 22 features, normalized to [-1, 1]. Defined in `data_loader.py:compute_features()` — this is the single source of truth. `feature_engineering.py` must match for inference.
- **Feature indices**: 0=heading_error, 1=mode_flag, 2=heading_rate, 3=roll, 4=pitch, 5=roll_rate, 6=AWA, 7=AWA_rate, 8=AWS, 9=TWA, 10=TWS, 11=STW, 12=SOG, 13=COG_error, 14=rudder_pos(zeroed), 15=rudder_velocity, 16=computed_heading, 17=VMG_up, 18=VMG_down, 19=PD_suggestion, 20=placeholder, 21=wave_period
- **Mode encoding** (feature 1): compass=0.0, wind_awa=0.5, wind_twa=1.0
- **Mirror augmentation**: Port/starboard mirroring negates features in `MIRROR_NEGATE_FEATURES` list
- **All steering modes reduce to compass heading** — wind modes compute a target heading via the wind triangle, then the heading error drives a single PD controller
- **Labels**: Expert rudder angle normalized to [-1, 1] (+-25 deg)

### Training Data

- Binary format (`.bin` files in `data/simulated/`)
- Each record: struct-packed sensor values + rudder label
- Sequences: sliding window of 20 frames at 2 Hz
- Mirror augmentation doubles effective dataset size

### CL (Closed-Loop) Validation

Three test scenarios run after each training:
- `compass_small`: 5 deg heading error, must hold <5 deg SS error (120s)
- `compass_large`: 90 deg heading error, must hold <10 deg SS error (60s)
- `wind_awa_hold`: AWA tracking 10 deg off target, must hold <10 deg SS error (120s)

CL diagnostics print feature values and model output at key timesteps.

## Build & Dependencies

- **Python**: 3.13 (`.python-version`)
- **Package manager**: uv — use `uv run` to execute Python, `uv add` to install packages
- **Virtual env**: `.venv/` (`.venv/bin/python` also works)
- **CI**: GitHub Actions runs `uv run pytest` on push/PR to main
- Dependencies defined in `pyproject.toml`, locked in `uv.lock`

## Security

### Path Sanitization (HTTP handlers)

When handling file paths from HTTP requests, always sanitize to prevent directory traversal:

```python
import os

def sanitize_path(base_dir: Path, name: str) -> Optional[Path]:
    base_path = os.path.abspath(str(base_dir))
    full_path = os.path.normpath(os.path.join(base_path, name))
    if not full_path.startswith(base_path + os.sep) and full_path != base_path:
        return None
    return Path(full_path)
```

Never disclose internal paths, stack traces, or exception messages in HTTP error responses. Log details internally, return generic messages to clients. Reference implementation: `vis/gribs/server.py`.

## Important Patterns

- When changing features in `data_loader.py:compute_features()`, always update `feature_engineering.py` to match
- Training data must be regenerated after any feature change
- The `python` command is not on PATH — use `uv run python`
- Use `python -u` flag when piping training output to `tee` (unbuffered)
- Model saves as both `.pt` (PyTorch) and `.onnx` (deployment)
- MLflow tracks experiments in `mlruns/`

## Claude Code Rules

- **NEVER clear the terminal** — do not run `clear`, `reset`, `printf '\033c'`, or any command that erases terminal scrollback. Training logs and diagnostic output in the terminal are the primary record of experimental results. Clearing the terminal destroys irreplaceable data.
