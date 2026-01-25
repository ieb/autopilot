# PD Controller

A **PD controller** (Proportional-Derivative controller) is a feedback control system used to steer the yacht toward a target heading. It's a simplified version of the more common PID controller, omitting the Integral term.

## How It Works

The controller computes a control output (rudder angle) based on two terms:

### Proportional (P) Term

Responds to the **current error** between target and actual heading:

```
P = Kp × error
```

If the heading is 10° off target, the proportional term applies a correction proportional to that error.

### Derivative (D) Term

Responds to the **rate of change** of the error, which dampens oscillation by opposing rapid changes:

```
D = Kd × (d_error/dt)
```

This prevents overshooting by reducing the correction as the boat approaches the target heading.

## Implementation

In the experiment's baseline controller:

```python
# Heading error (how far off target)
error = target_heading - heading

# PD control formula
command = kp * error - kd * heading_rate
```

The derivative term uses `heading_rate` directly (how fast the boat is turning) rather than computing the derivative of the error. This is equivalent when the target heading is constant.

### Gains Used

| Gain | Value | Effect |
|------|-------|--------|
| Kp | 0.5 | 0.5° rudder per degree of heading error |
| Kd | 0.3 | 0.3° rudder reduction per degree/second of turn rate |

## Why PD Instead of PID?

A full **PID controller** adds an Integral term that accumulates past errors over time. For heading control on a yacht, PD is often preferred because:

1. **Avoids wind-up**: The integral term can accumulate during sustained errors (e.g., tacking), causing overshoot when conditions change
2. **Natural damping**: The boat's hydrodynamics provide some integration effect naturally
3. **Gusty conditions**: The integral term can cause instability in variable wind
4. **Simpler tuning**: Two gains are easier to tune than three

## Block Diagram

```
                    ┌─────────────┐
  Target Heading ──▶│             │
                    │   Error     │──┬──▶ Kp × error ────────┐
  Current Heading ─▶│  Calculator │  │                       │
                    └─────────────┘  │                       ▼
                                     │              ┌─────────────┐
                                     │              │   Summing   │──▶ Rudder
                                     │              │   Junction  │    Command
                                     │              └─────────────┘
                                     │                       ▲
                    ┌─────────────┐  │                       │
  Heading Rate ────▶│  Kd × rate  │──┴───────────────────────┘
                    └─────────────┘
```

## Tuning Guidelines

- **Increase Kp**: Faster response, but may cause oscillation
- **Decrease Kp**: Slower response, more stable
- **Increase Kd**: More damping, reduces overshoot
- **Decrease Kd**: Less damping, faster settling but may oscillate

A well-tuned PD controller reaches the target heading quickly with minimal overshoot and no sustained oscillation.

## See Also

- [Planned Passage Experiment](planned_passage_experiment.md) - Uses PD controller as baseline
- [Model Architecture](model_architecture.md) - ML model that replaces PD controller
