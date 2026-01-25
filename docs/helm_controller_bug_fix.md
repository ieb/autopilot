# Helm Controller Bug Fix

This document describes critical bugs found in the helm controller that affected training data quality.

## Summary

Two sign errors in the helm controller caused the simulated training data to have near-zero correlation between heading error and rudder position. This resulted in a trained ML model that did not respond correctly to heading errors.

## Bugs Found

### Bug 1: Inverted Error Sign in `_compute_error()`

**Location:** `src/simulation/helm_controller.py`, `_compute_error()` method

**Problem:** Error was computed as `heading - target` instead of `target - heading`.

```python
# BEFORE (incorrect)
return self._angle_diff(heading, self.state.target_heading)

# AFTER (correct)
return self._angle_diff(self.state.target_heading, heading)
```

**Impact:** When the boat was pointing right of target (positive heading - target), the controller applied positive rudder (turn further right) instead of negative rudder (turn left to correct).

### Bug 2: Wrong Derivative Sign for COMPASS Mode

**Location:** `src/simulation/helm_controller.py`, `compute_rudder()` method

**Problem:** The derivative term used `heading_rate` directly instead of `-heading_rate`.

```python
# BEFORE (incorrect)
if self.state.mode == SteeringMode.COMPASS:
    derivative = heading_rate

# AFTER (correct)  
if self.state.mode == SteeringMode.COMPASS:
    derivative = -heading_rate  # Negative because d(error)/dt = -d(heading)/dt
```

**Impact:** When the boat was turning (non-zero heading_rate), the derivative term reinforced the motion instead of damping it. This caused oscillations that grew into continuous spinning at ~53°/s.

## Symptoms

Before the fix, the simulated yacht exhibited:
- Continuous spinning at 50+ degrees per second
- Rudder stuck at maximum deflection (-25°)
- No stable convergence to target heading
- Heading error standard deviation of 100°+

After the fix:
- Stable convergence to target heading within 5-10 seconds
- Heading error standard deviation of ~0.6°
- Proper negative feedback control

## Training Data Impact

The bugs caused training data to have:
- Near-zero correlation (-0.028) between heading error and rudder position
- The ML model learned to output approximately constant rudder values
- Model did not respond meaningfully to heading error inputs

## Required Actions

1. **Re-generate training data** using the fixed helm controller
2. **Re-train the ML model** from scratch with the new data
3. **Re-run experiments** to validate the new model

## Verification

To verify the fix works, run:

```bash
uv run python -c "
from src.simulation.helm_controller import HelmController, HelmConfig, SteeringMode
from src.simulation.yacht_dynamics import YachtDynamics

helm = HelmController(HelmConfig(reaction_delay=0.0))
helm.set_mode(SteeringMode.COMPASS, 85.0)

yacht = YachtDynamics()
yacht.state.heading = 95.0  # 10° off target
yacht.state.stw = 6.0

for i in range(50):
    cmd = helm.compute_rudder(yacht.state.heading, 90, 90, yacht.state.heading_rate, 0.1)
    yacht.step(cmd, 0.1)
    if i % 10 == 0:
        print(f't={i*0.1:.1f}s: heading={yacht.state.heading:.1f}° (target=85°)')

print(f'Final error: {85 - yacht.state.heading:.1f}°')
"
```

Expected output: heading converges from 95° to approximately 85° within 5 seconds.

## Date Fixed

January 24, 2026
