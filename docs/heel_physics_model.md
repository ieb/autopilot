# Heel Physics Model

This document describes the physics-based heel model implemented in the yacht dynamics simulation.

## Overview

The heel model calculates the boat's heel angle based on the balance between heeling moment (from wind on sails) and righting moment (from hull stability). This replaces the previous simplified model that used an empirical formula.

## Physics

### Heeling Moment

The wind creates a heeling force on the sails:

```
M_h = 0.5 × ρ_air × A_sail × C_L × V² × h_CE × cos(θ)
```

Where:
- `ρ_air` = 1.225 kg/m³ (air density)
- `A_sail` = effective sail area (m²)
- `C_L` = lift coefficient (varies with apparent wind angle)
- `V` = apparent wind speed (m/s)
- `h_CE` = center of effort height above CLR (6m default)
- `θ` = current heel angle (reduces effectiveness as boat heels)

### Righting Moment

The hull resists heeling through stability:

```
M_r = m × g × GM × sin(θ)
```

Where:
- `m` = displacement (5500 kg for Pogo 1250)
- `g` = 9.81 m/s²
- `GM` = metacentric height (1.5m, tuned to real data)
- `θ` = heel angle

### Equilibrium

The model iteratively solves for the heel angle where `M_h = M_r`.

## Sail Configuration

The `SailConfig` dataclass manages sail area:

```python
from src.simulation.yacht_dynamics import SailConfig

sail = SailConfig(
    main_area=55.0,      # m² full mainsail
    jib_area=52.0,       # m² genoa
    reef_level=0,        # 0=full, 1-3=reefed
    center_of_effort=6.0 # m above CLR
)

# Get effective sail area
area = sail.effective_area  # 107 m² at reef level 0
```

### Reef Levels

| Level | Factor | Effective Area |
|-------|--------|----------------|
| 0 (full) | 100% | 107 m² |
| 1 (1st reef) | 75% | 80 m² |
| 2 (2nd reef) | 55% | 59 m² |
| 3 (storm) | 35% | 37 m² |

## Automatic Reefing

When `auto_reef=True` (default), the simulation automatically adjusts sail area:

- **Reef up**: When heel exceeds `max_heel_before_reef` (25° default)
- **Shake out reef**: When heel drops below 50% of threshold

This prevents excessive heel while maximizing sail area for performance.

## Configuration

Stability parameters in `YachtConfig`:

```python
from src.simulation.yacht_dynamics import YachtConfig

config = YachtConfig(
    displacement_kg=5500,        # Total displacement
    metacentric_height=1.5,      # GM in meters
    max_heel=35.0,               # Absolute maximum heel
    max_heel_before_reef=25.0,   # Heel threshold for reefing
    auto_reef=True,              # Enable automatic reefing
    heel_damping=0.85,           # Response smoothing (0-1)
)
```

## Calibration

The model was tuned against real sailing data from the Pogo 1250:

| Condition | AWS | Measured Heel | Simulated Heel |
|-----------|-----|---------------|----------------|
| Close reach | ~10 kts | 5.3° | ~6° |

The metacentric height (GM = 1.5m) was tuned to match this calibration point.

## Usage Example

```python
from src.simulation.yacht_dynamics import YachtDynamics, YachtConfig

# Create yacht with custom stability
config = YachtConfig(
    metacentric_height=1.5,
    auto_reef=True
)
yacht = YachtDynamics(config=config)

# Set up conditions
yacht.reset(heading=0.0, twd=90.0, tws=20.0)

# Simulate
for _ in range(100):
    state = yacht.step(rudder_command=0.0, dt=0.1)
    print(f"Heel: {state.roll:.1f}°, Reef: {state.reef_level}")
```

## Sign Convention

Following NMEA2000 conventions:
- **Positive AWA** (wind from starboard) → **Negative roll** (heel to port)
- **Negative AWA** (wind from port) → **Positive roll** (heel to starboard)

## Limitations

1. **Static model**: Does not capture dynamic roll oscillations
2. **Simplified lift coefficient**: Uses approximate C_L vs AWA relationship
3. **No wave-induced roll**: Wave model affects pitch but not roll
4. **Instant reefing**: Reef changes are instantaneous (no delay)

## Files

- `src/simulation/yacht_dynamics.py` - Main implementation
- `tests/test_simulation.py` - Unit tests (`TestHeelPhysics` class)
