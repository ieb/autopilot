# Model Training Guide

> **Important**: The current production model uses **residual labels** (expert_rudder - pd_suggestion) and a **blended PD + ML inference** strategy. Training metrics (val loss ~0.013, MAE ~1.7 deg) are not comparable to earlier absolute-label runs (~0.001, ~0.65 deg). CL validation with error-magnitude blending is the only meaningful measure of model quality. See **[blended_pd_ml_controller.md](blended_pd_ml_controller.md)** for the current control law, training guidance, and tuning instructions.

---

This guide covers the complete workflow for training the autopilot model from data to deployable artifact.

## Overview

The training pipeline uses imitation learning: the model learns to reproduce the rudder commands of a human helm (or simulation) given the same sensor inputs.

```
Training Data (.bin or .jsonlog + .meta.json)
    → TrainingDataLoader (sequences, normalization, filtering)
    → ImitationTrainer (PyTorch LSTM, early stopping, checkpointing)
    → Output: .pt model, .onnx edge model, config, history
```

## Prerequisites

```bash
cd autopilot
uv sync
```

PyTorch is required for training. The trainer auto-detects hardware acceleration (MPS on Apple Silicon, CUDA on NVIDIA GPUs, or CPU fallback).

## Preparing Training Data

Training data can come from two sources. Both produce `.bin` files by default (or `.jsonlog` + `.meta.json` with the `--json` flag for legacy). The binary format stores 26×float32 per frame (25 pre-computed features + 1 rudder label = 104 bytes/frame) with a 16-byte header. It is ~7× smaller on disk than JSON and ~20× more memory efficient during training because sequences are constructed on-the-fly.

### Source 1: Simulated data (scenario-based)

Generate synthetic data from predefined sailing scenarios:

```bash
uv run python -m src.simulation.data_generator \
    --hours 50 --num-runs 3 --output data/simulated/
```

See [Simulated Training Data](simulated_training_data.md) for full details.

### Source 2: Simulated data (GRIB-driven)

Generate data from planned routes with real GRIB wind/wave conditions:

```bash
uv run python -m src.simulation.data_generator \
    --route data/experiment1/route/wr_route_1_20260222_102210.kml \
    --gribs data/experiment1/gribs/ \
    --num-runs 3 \
    --output data/simulated_grib/
```

See the "GRIB-Driven Generation" section in [Simulated Training Data](simulated_training_data.md).

### Source 3: Real logged data

Prepare CAN bus logs recorded while sailing:

```bash
# Organize raw logs by date
uv run python -m src.n2k.log_organizer n2klogs/raw/ --execute

# Analyze logs and generate training metadata
uv run python -m src.training.log_analyzer n2klogs/raw/
```

See [Data Preparation](data_preparation.md) for the full log preparation workflow.

### Verifying data loads

Before training, verify your data loads correctly:

```bash
uv run python -c "
from src.training.data_loader import TrainingDataLoader
loader = TrainingDataLoader()
X, y = loader.load_directory('data/simulated/')
print(f'Sequences: {len(X):,}')
print(f'X shape: {X.shape}')   # (N, 20, 25)
print(f'y shape: {y.shape}')   # (N, 1)
"
```

### Data volume guidelines

| Data Volume | Expected Model Quality |
|-------------|----------------------|
| < 1 hour | Insufficient for learning |
| 1-10 hours | Basic functionality |
| 10-50 hours | Good generalization |
| 50+ hours | Production quality |

Combining scenario-based, GRIB-driven, and real data provides the best coverage.

## Training

### Quick start

```bash
# Train on simulated data (auto-resumes from checkpoint if available)
uv run python -m src.training.train_imitation data/simulated/ --output models/
```

### CLI Reference

```bash
uv run python -m src.training.train_imitation DATA_DIR [OPTIONS]
```

| Option | Default | Description |
|--------|---------|-------------|
| `data_dir` | (required) | Directory containing `.bin` or `.jsonlog` + `.meta.json` files. Binary files are auto-detected and preferred. |
| `--json` | False | Force legacy JSON loading (default: auto-detect .bin) |
| `--output, -o` | `models` | Output directory for model files |
| `--epochs, -e` | 100 | Maximum training epochs |
| `--max-time, -t` | None | Maximum training time in seconds |
| `--fresh, -f` | False | Start fresh, ignoring any existing checkpoint |
| `--mlflow` | False | Enable MLflow experiment tracking |
| `--experiment-name` | `autopilot` | MLflow experiment name |
| `--run-name` | None | MLflow run name (optional) |
| `--preprocess` | False | Preprocess data to .npy files and exit |
| `--preprocessed-dir` | None | Directory with preprocessed .npy files (enables streaming) |
| `--streaming` | False | Use memory-mapped streaming (auto-preprocesses if needed) |

### Examples

```bash
# Quick test (30 second time limit)
uv run python -m src.training.train_imitation data/simulated/ --max-time 30

# Full training with MLflow tracking
uv run python -m src.training.train_imitation data/simulated/ \
    --output models/ --mlflow --run-name "sim-50h-v2"

# Start fresh (ignore previous checkpoint)
uv run python -m src.training.train_imitation data/simulated/ \
    --output models/ --fresh

# Train on combined simulated + GRIB data
uv run python -m src.training.train_imitation data/simulated/ data/simulated_grib/ \
    --output models/ --epochs 200
```

## Understanding Training Output

### Console output

The trainer prints progress after each epoch:

```
======================================================================
TRAINING STARTED
======================================================================
Device: mps
Training samples: 1,187,980, Validation samples: 296,995

Epoch 1/100 (2.1s, elapsed: 2s, ETA: 207s)
  Loss:     0.045123  |  Val Loss: 0.042567  ↓ improved by 0.042567
  MAE:      0.034521 (1.04°)  |  Val MAE:  0.033245 (1.00°)
  LR: 1.00e-03  |  Best Val Loss: 0.042567

Epoch 5/100 (1.9s, elapsed: 10s, ETA: 190s)
  Loss:     0.028234  |  Val Loss: 0.026789  ↓ improved by 0.003421
  MAE:      0.021345 (0.64°)  |  Val MAE:  0.020123 (0.60°)
  LR: 1.00e-03  |  Best Val Loss: 0.026789  trend: ↓0.003142/epoch
```

Key values to watch:

| Metric | What it means |
|--------|--------------|
| **Val Loss** | Validation MSE loss -- lower is better. The primary quality metric. |
| **Val MAE (degrees)** | Mean absolute error in rudder degrees. Under 1° is good. |
| **↓ improved by** | How much val loss improved. Small improvements suggest convergence. |
| **no improvement for N epochs** | If this reaches the patience limit (8), training stops. |
| **LR** | Learning rate. Automatically reduced on plateaus. |

### Early stopping

Training stops automatically when validation loss stops improving for 8 consecutive epochs (`patience=8`). This prevents overfitting.

### Checkpointing and resume

- The best model (lowest validation loss) is saved to `models/autopilot_best.pt` after every improvement.
- If training is interrupted, re-running the same command resumes from the checkpoint automatically.
- Use `--fresh` to discard the checkpoint and start over.
- When resuming, the learning rate warmup is skipped since the model is already initialized.

## Large Datasets (Streaming Mode)

The binary format constructs sequences on-the-fly during training, making it ~20× more memory efficient than JSON. For most datasets, binary format alone eliminates the need for streaming mode.

When using legacy `.jsonlog` files or combining multiple large sources, streaming mode may still be useful. By default, all training data is loaded into memory at once. For large datasets (over ~2GB of log files), this can require 10-20GB of RAM. Streaming mode solves this by preprocessing data to disk and memory-mapping it during training, so only the active batch is in RAM.

### Two-step workflow

```bash
# Step 1: Preprocess log files to compact .npy format
uv run python -m src.training.train_imitation data/simulated_grib/ --preprocess

# Step 2: Train using the preprocessed files (low memory)
uv run python -m src.training.train_imitation data/simulated_grib/ \
    --preprocessed-dir data/simulated_grib_preprocessed/ \
    --output models/
```

### One-step workflow

If you add `--streaming`, it will auto-preprocess on the first run:

```bash
uv run python -m src.training.train_imitation data/simulated_grib/ \
    --streaming --output models/
```

### How it works

1. **Preprocess**: Each `.jsonlog` file is parsed and converted into a pair of `.npy` files (X sequences, y labels). A `manifest.json` tracks the file list and total sequence count.
2. **Memory-map**: During training, `.npy` files are opened with `numpy.load(mmap_mode='r')` so the OS pages data in on demand rather than loading everything into RAM.
3. **Batch loading**: PyTorch's `DataLoader` fetches sequences from the memory-mapped arrays as needed. Only the current batch occupies GPU/CPU memory.

### Memory comparison

| Mode | 5GB of .jsonlog files | RAM usage |
|------|----------------------|-----------|
| Default (in-memory) | ~20GB | Everything in RAM |
| Streaming | ~500MB | Only active batch + OS file cache |

### Preprocessed directory structure

```
data/simulated_grib_preprocessed/
├── manifest.json                      # File index with sequence counts
├── grib_wr_route_1_run00_X.npy       # Feature sequences (memory-mappable)
├── grib_wr_route_1_run00_y.npy       # Labels
├── grib_wr_route_1_run01_X.npy
├── grib_wr_route_1_run01_y.npy
└── ...
```

The preprocessed files can be reused across training runs. Re-run `--preprocess` if the source data or feature engineering changes.

## Output Files

After training completes, the output directory contains:

| File | Description |
|------|-------------|
| `autopilot.pt` | Final PyTorch model (for further training) |
| `autopilot_best.pt` | Best checkpoint (lowest val loss during training) |
| `autopilot.onnx` | ONNX model for Raspberry Pi deployment (~311 KB) |
| `autopilot_config.json` | Training, model, and data configuration |
| `autopilot_history.json` | Per-epoch loss, MAE, and learning rate history |

The `.onnx` model is automatically exported at the end of training. See [Model Export (ONNX)](model_export_onnx.md) for deployment details.

## Model Architecture

The autopilot uses a stacked LSTM architecture:

```
Input (20, 25) → Feature Mixing → LSTM(64) → LSTM(32) → Dense(16) → Output (1)
```

| Property | Value |
|----------|-------|
| Parameters | 66,209 (258 KB) |
| Input | 20 timesteps x 25 features |
| Output | Rudder command in [-1, 1] (maps to ±30°) |

See [Model Architecture](model_architecture.md) for full details.

## Training Configuration

The defaults are tuned for the autopilot task. Advanced users can modify `TrainingConfig` in `src/training/train_imitation.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `batch_size` | 64 | Samples per gradient step |
| `learning_rate` | 1e-4 | Initial learning rate |
| `patience` | 8 | Early stopping patience (epochs) |
| `min_delta` | 0.0001 | Minimum improvement to count as progress |
| `warmup_epochs` | 3 | Learning rate warmup period |
| `add_noise` | True | Data augmentation with Gaussian noise |
| `noise_std` | 0.02 | Noise standard deviation for augmentation |

## Evaluating a Trained Model

After training, evaluate the model against planned passages:

```bash
uv run python -m experiments.experiment1.run_experiment \
    --route data/experiment1/route/wr_route_1_20260222_102210.kml \
    --gribs data/experiment1/gribs/ \
    --model models/autopilot_best.onnx \
    --output results/evaluation/
```

See [Planned Passage Experiment](planned_passage_experiment.md) for full evaluation options and [Model Evaluation Report](model_evaluation_report.md) for current results.

## MLflow Tracking

Enable MLflow to compare training runs:

```bash
# Train with tracking
uv run python -m src.training.train_imitation data/simulated/ \
    --mlflow --experiment-name autopilot --run-name "baseline-v1"

# View results
uv run mlflow ui
# Open http://localhost:5000
```

MLflow logs per-epoch metrics, model parameters, and saved artifacts. See [MLflow Tracking](mlflow.md) for setup and usage.

## Recommended Workflow

1. **Generate training data** -- combine multiple sources for diversity:
   ```bash
   # Scenario-based (variety of conditions)
   uv run python -m src.simulation.data_generator --hours 50 --num-runs 3

   # GRIB-driven (real weather on planned routes)
   uv run python -m src.simulation.data_generator \
       --route data/experiment1/route/*.kml \
       --gribs data/experiment1/gribs/ --num-runs 3
   ```

2. **Train the model**:
   ```bash
   uv run python -m src.training.train_imitation data/simulated/ \
       --output models/ --mlflow --run-name "sim-v1"
   ```

3. **Evaluate on planned passages**:
   ```bash
   uv run python -m experiments.experiment1.run_experiment \
       --route data/experiment1/route/wr_route_1_20260222_102210.kml \
       --model models/autopilot_best.onnx
   ```

4. **Iterate** -- generate more data, adjust scenarios, retrain. Each cycle should improve results when evaluated against the same passages.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `PyTorch not available` | Run `uv sync` to install dependencies |
| Training loss not decreasing | Check data quality with the verify script above |
| Val loss increasing while train loss decreases | Overfitting -- add more data or reduce epochs |
| `CUDA out of memory` | Reduce batch_size in TrainingConfig |
| Slow training on Mac | Verify MPS is detected (`Device: mps` in output) |
| Model performs poorly on evaluation | Check training data covers the conditions in the route |
