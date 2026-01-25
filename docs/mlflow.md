# MLflow Experiment Tracking

This document describes how to use MLflow for tracking model training experiments.

## Overview

MLflow is integrated into the training pipeline to provide:

- **Experiment tracking**: Compare different training runs
- **Parameter logging**: All hyperparameters recorded automatically
- **Metric visualization**: Interactive loss/MAE curves
- **Artifact storage**: Models, configs, and training history
- **Model registry**: Version and stage models for deployment

## Installation

MLflow is included in the dev dependencies:

```bash
uv sync --extra dev
```

Or install directly:

```bash
uv pip install "mlflow>=2.10.0"
```

## Training with MLflow

Enable MLflow tracking by adding the `--mlflow` flag:

```bash
uv run python -m src.training.train_imitation data/simulated --mlflow
```

### CLI Options

| Option | Description | Default |
|--------|-------------|---------|
| `--mlflow` | Enable MLflow tracking | disabled |
| `--experiment-name NAME` | MLflow experiment name | `autopilot` |
| `--run-name NAME` | Name for this specific run | auto-generated |

### Examples

Basic training with MLflow:

```bash
uv run python -m src.training.train_imitation data/simulated --mlflow
```

Named run for comparison:

```bash
uv run python -m src.training.train_imitation data/simulated \
    --mlflow \
    --run-name "lstm-64-32-baseline"
```

Custom experiment name:

```bash
uv run python -m src.training.train_imitation data/simulated \
    --mlflow \
    --experiment-name "autopilot-v2" \
    --run-name "increased-dropout"
```

## Viewing Results

### Starting the MLflow UI

```bash
uv run mlflow ui --port 5000
```

Then open http://localhost:5000 in your browser.

### UI Features

1. **Experiments List**: Left sidebar shows all experiments
2. **Runs Table**: Each row is a training run with params and metrics
3. **Run Comparison**: Select multiple runs and click "Compare"
4. **Metric Charts**: Click on a run to see training curves
5. **Artifacts**: Download models, configs, and history files

## What Gets Logged

### Automatic (via autolog)

These are logged automatically by `mlflow.keras.autolog()`:

- Per-epoch metrics: `loss`, `val_loss`, `mae`, `val_mae`
- Fit parameters: `epochs`, `batch_size`
- Model summary and architecture
- Trained model (saved in native `.keras` format)
- Model signature (input/output schema)

### Manual (custom logging)

These are logged by our custom integration:

**Training Config:**
- `learning_rate`, `patience`, `min_delta`
- `warmup_epochs`, `add_noise`, `noise_std`
- `resume_from_checkpoint`

**Model Config:**
- `lstm_units_1`, `lstm_units_2`
- `dense_units`, `dropout_rate`
- `sequence_length`, `feature_dim`

**Data Config:**
- `sample_rate_hz`, `train_split`, `normalize`
- `num_train_samples`, `num_val_samples`

**Evaluation Metrics (in degrees):**
- `eval_mae_degrees`, `eval_rmse_degrees`
- `eval_max_error_degrees`, `eval_std_error`

**Artifacts:**
- ONNX model (`autopilot.onnx`)
- Config JSON (`autopilot_config.json`)
- Training history (`autopilot_history.json`)

## Workflow Example

1. **Train baseline model:**
   ```bash
   uv run python -m src.training.train_imitation data/simulated \
       --mlflow --run-name "baseline"
   ```

2. **Try different hyperparameters:**
   ```bash
   uv run python -m src.training.train_imitation data/simulated \
       --mlflow --run-name "higher-dropout" --epochs 150
   ```

3. **Compare results:**
   ```bash
   uv run mlflow ui
   # Open http://localhost:5000
   # Select both runs in the table
   # Click "Compare" button
   ```

4. **View training curves:**
   - Click on a run name
   - Scroll down to "Metrics" section
   - Click on `val_loss` to see the training curve

## Storage

MLflow stores experiment data locally in `./mlruns/` directory. This is excluded from git via `.gitignore`.

To clean up old experiments:

```bash
rm -rf mlruns/
```

## Troubleshooting

### MLflow not found

```
Warning: MLflow requested but not installed. Run: uv pip install mlflow
```

Install with: `uv pip install "mlflow>=2.10.0"`

### Port already in use

```bash
# Use a different port
uv run mlflow ui --port 5001
```

### No experiments showing

Make sure you're running the UI from the project root where `mlruns/` is located:

```bash
uv run mlflow ui
```
