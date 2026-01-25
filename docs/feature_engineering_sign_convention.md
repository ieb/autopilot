# Feature Engineering Sign Convention Fix

This document describes the sign convention inconsistency found in the ML feature engineering code and its fix.

## Summary

The feature engineering code in `passage_simulator` and `data_loader` used a different error sign convention than the `HelmController`. While mathematically equivalent (compensated by negating the gain), this inconsistency caused confusion and increased the risk of sign errors when modifying the code.

The fix aligns all code to use the HelmController convention, which matches NMEA2000 standards.

## The Problem

### Inconsistent Error Conventions

The `HelmController` (which generates training data) uses:

```python
# HelmController convention (correct, NMEA2000 aligned)
error = target - heading  # Positive error = target to starboard
command = kp * error      # Positive error → positive rudder
```

The feature engineering code was using:

```python
# Feature engineering convention (before fix)
error = heading - target  # Opposite sign!
command = -error * gain   # Compensated with negative gain
```

### Why This Was Problematic

1. **Cognitive load**: Developers had to remember two different conventions
2. **Bug risk**: Easy to introduce sign errors when modifying code
3. **Documentation confusion**: Comments describing behavior were confusing
4. **Maintenance burden**: Any change required understanding both conventions

### Architectural Context

The issue arose during implementation of a hierarchical control architecture where:
- Wind modes (AWA/TWA) compute a target heading from wind angle targets
- The ML model always steers to a computed heading
- The error signal must be consistent across all steering modes

## Files Affected

### 1. `experiments/experiment1/passage_simulator.py`

**Method:** `_compute_features()`

```python
# BEFORE (inconsistent)
error = heading - computed_heading
# ...later in MockAutopilotInference...
command = -heading_error * 0.5

# AFTER (aligned with HelmController)
error = computed_heading - heading
# ...later in MockAutopilotInference...
command = heading_error * 0.5
```

### 2. `src/training/data_loader.py`

**Method:** `_frame_to_features()`

```python
# BEFORE
error = self._angle_diff(frame.heading, computed_heading)

# AFTER
error = self._angle_diff(computed_heading, frame.heading)
```

### 3. `src/ml/autopilot_model.py`

**Class:** `MockAutopilotInference`

```python
# BEFORE
command = -heading_error * 0.5  # Negative gain to compensate

# AFTER
command = heading_error * 0.5   # Positive gain (HelmController aligned)
```

## NMEA2000 Sign Convention Reference

The authoritative sign conventions for this project are:

| Signal | Positive | Negative |
|--------|----------|----------|
| Rudder angle | Starboard | Port |
| AWA/TWA | Wind from starboard | Wind from port |
| Heading | 0-360° clockwise from North | - |

### Physical Effects

- **Positive rudder** → boat turns starboard → heading increases → AWA decreases
- **Negative rudder** → boat turns port → heading decreases → AWA increases

### Error Convention (HelmController)

```python
# Compass mode
error = target_heading - current_heading
# Positive error = target is clockwise (to starboard)
# Positive error → positive rudder → turn starboard toward target

# Wind modes (AWA/TWA)
error = current_awa - target_awa
# Positive error = wind too far aft (need to head up)
# Positive error → positive rudder → turn starboard → AWA decreases
```

## Prevention

A Cursor rule was created at `.cursor/rules/nmea2000-conventions.mdc` to document these conventions and prevent future regressions. This rule is set to `alwaysApply: true` so it will be included in every AI conversation in this project.

## Verification

### Tests

All 369 tests pass, including:
- 10 `TestNMEA2000Conventions` tests
- 6 `TestHelmControllerBugFixes` tests
- `test_mock_inference_proportional_response` (updated to match new convention)

### Mock Experiment

The mock autopilot experiment produces correct behavior:
- No runaway spinning (max heading error ~20°, not 180°)
- Low cross-track error (~0.01 nm)
- Stable course following

## Required Actions

When regenerating training data or retraining the model:

1. Ensure `data_loader._frame_to_features()` uses `error = computed_heading - heading`
2. Ensure training and inference use the same convention
3. Run `TestNMEA2000Conventions` tests to validate

## Related Documentation

- [Helm Controller Bug Fix](helm_controller_bug_fix.md) - Previous sign convention bugs in the helm controller
- `.cursor/rules/nmea2000-conventions.mdc` - Cursor rule documenting conventions

## Date Fixed

January 25, 2026
