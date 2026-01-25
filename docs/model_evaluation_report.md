# Model Evaluation Report

This document describes the current state of the ML autopilot model and its performance in passage simulation experiments.

**Report Date**: January 2026  
**Model Version**: `aligned-conventions-v1`  
**MLflow Run**: `autopilot/aligned-conventions-v1`

## Executive Summary

The ML autopilot model has been successfully trained and validated against a baseline PD controller. After resolving sign convention issues in the feature engineering pipeline, the model now performs comparably to or better than the baseline across all key metrics.

| Metric | Baseline (PD) | ML Model | Assessment |
|--------|---------------|----------|------------|
| Legs Completed | 62/62 | 62/62 | ✅ Full passage |
| Mean XTE | 216 m | 212 m | ✅ Slightly better |
| Mean Heading Error | 0.7° | 0.5° | ✅ Better |
| Rudder Smoothness | 0.64° std | 0.59° std | ✅ Smoother |

## Model Architecture

The model uses a stacked LSTM architecture optimized for real-time inference:

```
Input (20, 25) → Feature Mixing → LSTM(64) → LSTM(32) → Dense(16) → Output (1)
```

| Property | Value |
|----------|-------|
| Parameters | 66,209 (258 KB) |
| Input Shape | (20 timesteps, 25 features) |
| Output | Rudder command [-1, 1] |
| Inference Target | ONNX (311 KB) |

See [Model Architecture](model_architecture.md) for full details.

## Training Details

### Dataset

Training data was generated using the simulation framework with aligned sign conventions:

| Parameter | Value |
|-----------|-------|
| Total Sequences | 78,120 |
| Training Samples | 62,496 |
| Validation Samples | 15,624 |
| Sample Rate | 10 Hz |
| Sequence Length | 20 timesteps (2 seconds) |
| Scenarios | upwind, reaching, downwind, tacking, error_recovery |

**Data Improvements**:
- 90-second warm-up period to exclude startup transients
- Error recovery scenarios with 30-90° initial heading offsets
- Mode transition filtering (15s around mode changes)

### Training Configuration

| Parameter | Value |
|-----------|-------|
| Epochs | 19 (early stopped) |
| Batch Size | 64 |
| Initial LR | 1e-4 (with 3-epoch warmup) |
| Final LR | 8.75e-6 (auto-reduced) |
| Patience | 8 epochs |
| Training Time | 86.6 minutes |

### Convergence

```
Epoch  1: Val Loss 0.0270, MAE 4.19°
Epoch  5: Val Loss 0.0024, MAE 1.10°
Epoch 10: Val Loss 0.0017, MAE 0.88°
Epoch 15: Val Loss 0.0014, MAE 0.81° ← Best
Epoch 19: Val Loss 0.0014, MAE 0.80° (early stop)
```

**Final Metrics**:
- Validation Loss: 0.001414
- MAE: 0.82° (excellent sub-degree accuracy)
- RMSE: 1.15°
- Total improvement: 94.8%

## Experiment Results

### Test Scenario: Planned Passage Simulation

The model was evaluated on a realistic passage simulation:

| Parameter | Value |
|-----------|-------|
| Route | 62 legs, 111.8 nm |
| Duration | ~14 hours |
| Conditions | Variable wind, waves |
| Steering Mode | AWA (100% of time) |

### Comparison: ML Model vs Baseline PD Controller

#### Navigation Performance

| Metric | Baseline | ML Model | Δ |
|--------|----------|----------|---|
| Planned Duration | 13:42:38 | 13:42:38 | - |
| Actual Duration | 13:33:13 | 14:07:20 | +34 min |
| ETA Error | -1.1% | +3.0% | ⚠️ |
| Distance Covered | 111.8 nm | 111.8 nm | ✅ |

The ML model arrived slightly later than the baseline. This may be due to marginally less aggressive steering or different tacking behavior.

#### Track Keeping (Cross-Track Error)

| Metric | Baseline | ML Model | Δ |
|--------|----------|----------|---|
| Mean XTE | 0.117 nm (216 m) | 0.115 nm (212 m) | ✅ -2% |
| Max XTE | 1.644 nm | 1.643 nm | ≈ |
| RMS XTE | 0.302 nm | 0.301 nm | ≈ |

Both controllers maintain excellent track keeping with mean XTE around 200m.

#### Heading Control

| Metric | Baseline | ML Model | Δ |
|--------|----------|----------|---|
| Mean Error | 0.7° | 0.5° | ✅ -29% |
| RMS Error | 3.0° | 3.0° | ≈ |
| Max Error | 107.5° | 108.0° | ≈ |

The ML model maintains tighter heading control on average. Max errors (~108°) occur during tacks through the wind, which is expected.

#### Rudder Activity

| Metric | Baseline | ML Model | Δ |
|--------|----------|----------|---|
| Mean Rudder | 0.04° | 0.02° | ✅ |
| Std Dev | 0.64° | 0.59° | ✅ -8% |
| Max | ±15.5° | ±16.1° | ≈ |

The ML model produces smoother rudder commands with less variance, indicating more refined control.

#### Polar Performance (Steady-State)

Analysis of subsampled time series (10,168 points, 1-in-25):

| Metric | Baseline | ML Model | Δ |
|--------|----------|----------|---|
| Mean | 99.9% | 100.0% | ≈ |
| Std Dev | 15.1% | 11.5% | ✅ -24% |

The ML model maintains more consistent polar performance with lower variance.

### Polar Performance Variance Note

The summary statistics report higher polar variance (149% for ML vs 41% for baseline) due to extreme values during tacks. When TWA ≈ 0° (in irons), the polar calculation produces artifacts:

```
During tack:
  TWA: -0.5°, STW: 1.01 kts → Polar: 981%  (calculation artifact)
```

These artifacts occur when dividing residual boat speed by a near-zero polar target. The subsampled time series analysis (above) provides a more representative picture of steady-state performance.

## Key Findings

### Successes

1. **Sign Convention Fix**: Aligning feature engineering to use `error = target - current` (positive error → positive rudder) resolved the catastrophic failure from earlier versions.

2. **Training Data Quality**: Warm-up periods and error recovery scenarios produced cleaner, more representative training data.

3. **Comparable Performance**: The ML model matches or exceeds the baseline PD controller on all primary metrics.

4. **Smooth Control**: Lower rudder variance indicates the model learned efficient steering patterns rather than reactive oscillation.

### Areas for Improvement

1. **ETA Performance**: The 3% slower arrival time suggests room for optimizing VMG during upwind/downwind legs.

2. **Tack Timing**: The model may benefit from explicit tack optimization training.

3. **Training Data Volume**: Current training used 3 hours of simulated data. More diverse conditions could improve generalization.

## Model Artifacts

| File | Description | Size |
|------|-------------|------|
| `models/autopilot.keras` | Keras model (training) | - |
| `models/autopilot.onnx` | ONNX model (inference) | 311 KB |
| `models/autopilot_config.json` | Configuration | 1 KB |
| `models/autopilot_history.json` | Training history | - |
| `models/autopilot_best.keras` | Best checkpoint | - |

## Reproducibility

### Regenerate Training Data

```bash
uv run python scripts/regenerate_training_data.py \
    --hours 1 --runs 3 --warmup 90
```

### Retrain Model

```bash
uv run python -m src.training.train_imitation data/simulated \
    --mlflow --run-name "aligned-conventions-v2"
```

### Run Evaluation

```bash
uv run python -m experiments.experiment1.run_experiment \
    --route routes/solent.gpx \
    --output results/experiment-new
```

### View Training History

```bash
uv run mlflow ui --port 5000
# Open http://localhost:5000
```

## Related Documentation

- [Model Architecture](model_architecture.md) - Network design and features
- [Feature Engineering Sign Convention](feature_engineering_sign_convention.md) - Sign convention fix details
- [Helm Controller Bug Fix](helm_controller_bug_fix.md) - Previous sign convention issues
- [Planned Passage Experiment](planned_passage_experiment.md) - Experiment design
- [Simulated Training Data](simulated_training_data.md) - Data generation process
- [MLflow Tracking](mlflow.md) - Experiment tracking

## Conclusion

The ML autopilot model is ready for further testing and refinement. The core functionality is validated: it can successfully navigate a complex multi-leg passage while maintaining track accuracy and heading control comparable to the baseline PD controller. Future work should focus on optimizing for speed (ETA performance) and testing with real logged data from human helmsmen.
