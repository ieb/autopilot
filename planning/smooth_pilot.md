# Progressive Rudder Application (SmoothPilot)

## Context

The PD pilot (kp=1.6, kd=1.5) computes rudder commands instantaneously. At 15° heading error: `1.6 * 15 = 24°` — nearly full rudder in a single 0.5s step. This is independent of boat speed, so at 10kn where rudder is highly effective, the boat gets the same aggressive command as at 3kn. The result is uncomfortable helm slams and potential rig/crew safety issues at speed.

**Goal:** Progressive rudder that builds over time, is aware of boat speed, and backs off when the boat is already turning well.

## Approach: SmoothPilot Wrapper

A wrapper pilot (same pattern as `AdaptivePilot`) that sits between the inner PD/PID pilot and the output. Three mechanisms applied in sequence:

### 1. Speed-dependent gain scaling
Reduce the target command at higher boat speeds where rudder is more effective.

```
stw = features[11] * 25.0              # boat speed (knots)
speed_factor = speed_ref / max(stw, 1.0)  # >1 slow, <1 fast
speed_factor = clip(speed_factor, 0.4, 1.0)
target = inner.steer(features) * speed_factor
```

At 6kn (reference): no change. At 10kn: target reduced to 60%. At 3kn: unchanged (capped at 1.0).

### 2. Rate limiting with urgency ramp
Limit how fast the rudder command can change, with the limit increasing as error persists.

```
urgency = min(1.0, time_at_error / 30.0)    # 0→1 over 30s
effective_rate = base_rate * (1 + 2 * urgency)  # 2→6 deg/s
max_change = (effective_rate / 25.0) * dt
output = prev_output + clip(target - prev_output, -max_change, max_change)
```

**Timing at 2Hz, base_rate=2.0 deg/s:** Initial max change = 1.0°/step. To reach 12.5° rudder from zero takes ~6s. After 30s of persistent error, rate triples to 6.0 deg/s. The PD D-term naturally reduces the target as the boat responds, so the output levels off before reaching the initial aggressive target.

### 3. Turn-rate awareness
If the boat is already turning well, slow further rudder application.

```
turn_rate = abs(features[2] * 30.0)   # deg/s
if turn_rate > 3.0:
    excess = (turn_rate - 3.0) / 3.0
    max_change *= 1.0 / (1.0 + 0.5 * excess)
```

At 6°/s turn rate: rate limit drops to ~67%. Prevents overshoot when the boat is already responding.

## Parameters

| Parameter | Default | Rationale |
|-----------|---------|-----------|
| `base_rate_limit` | 2.0 deg/s | 1°/step at 2Hz. Gentle but responsive enough for 5° errors in ~2.5s |
| `speed_ref` | 6.0 kn | Typical Pogo 1250 sailing speed. No scaling at this speed |
| `speed_gain_min` | 0.4 | At 15kn (surfing), max rudder effectively limited to ~10° |
| `speed_gain_max` | 1.0 | Don't amplify beyond what PD asks at low speed |
| `turn_rate_threshold` | 3.0 deg/s | "Boat is turning well" threshold |
| `turn_rate_gain` | 0.5 | Moderate backoff when turning fast |

## Files to Create

### 1. `src/pilots/smooth_pilot.py` (~80 lines)

```python
class SmoothPilot(BasePilot):
    name = "smooth"

    def __init__(self, inner, dt=0.5, base_rate_limit=2.0,
                 speed_ref=6.0, speed_gain_min=0.4, speed_gain_max=1.0,
                 turn_rate_threshold=3.0, turn_rate_gain=0.5): ...
    def steer(self, features) -> float: ...
    def reset(self) -> None: ...
```

State: `_prev_output`, `_time_at_error`

### 2. `tests/test_smooth_pilot.py`

Unit tests:
- `test_rate_limits_step_change` — 90° error first step output << raw PD
- `test_ramps_up_over_time` — constant error produces monotonically increasing output
- `test_urgency_increases_rate` — output ramps faster after sustained error
- `test_high_speed_reduces_output` — 10kn output < 6kn output for same error
- `test_low_speed_no_amplification` — 3kn output matches unscaled
- `test_fast_turn_slows_application` — high heading rate reduces rate limit
- `test_reset_clears_state` — reset zeroes _prev_output and _time_at_error
- `test_stacks_with_adaptive` — `AdaptivePilot(SmoothPilot(PDPilot()))` works

CL validation:
- `test_cl_all_scenarios_pass` — smooth PD passes all 3 CL scenarios

## Files to Modify

### 3. `src/pilots/__init__.py`
Add `from .smooth_pilot import SmoothPilot` and `register(SmoothPilot)`

### 4. `src/pilots/evaluate.py`
Extend wrapper construction to handle `"smooth"`:
```python
if args.pilot in ("adaptive", "smooth"):
    inner = get_pilot(args.inner)
    if args.pilot == "adaptive":
        pilot = AdaptivePilot(inner=inner)
    else:
        pilot = SmoothPilot(inner=inner)
```

## Implementation Order

1. Create `src/pilots/smooth_pilot.py`
2. Register in `src/pilots/__init__.py`
3. Update `src/pilots/evaluate.py` wrapper construction
4. Create `tests/test_smooth_pilot.py`
5. Run tests and CL validation
6. If CL fails, increase `base_rate_limit` (2.0 → 3.0) as first tuning knob

## Verification

```bash
# Unit tests
uv run python -m pytest tests/test_smooth_pilot.py -x -q

# CL validation — smooth PD
uv run python -m src.pilots.evaluate --pilot smooth --inner pd --verbose

# Compare raw PD baseline
uv run python -m src.pilots.evaluate --pilot pd --verbose

# Stacked: adaptive + smooth
uv run python -m src.pilots.evaluate --pilot adaptive --inner smooth --verbose

# Full test suite
uv run python -m pytest tests/ -x -q
```

## Not In Scope
- `main.py` deployment integration (after CL proves the approach)
- Gain scheduling by sea state
- Per-tack/gybe rate profiles
