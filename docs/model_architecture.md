# Autopilot Model Architecture

This document describes the neural network architecture used for the sailing autopilot, including input features, model layers, and training configuration.

## Overview

The autopilot uses an **end-to-end LSTM model** that learns to imitate human helming behavior. It takes a sequence of sensor observations and outputs a rudder command in the range [-1, 1] (corresponding to ±30° rudder deflection).

```
Input: [batch, 20, 25] → LSTM Network → Output: [batch, 1]
       (20 timesteps, 25 features)        (rudder command)
```

## Model Summary

| Property | Value |
|----------|-------|
| Name | `autopilot_lstm` |
| Total Parameters | 47,969 (187.38 KB) |
| Trainable Parameters | 47,809 (186.75 KB) |
| Non-trainable Parameters | 160 (640 bytes) |
| Input Shape | `(20, 25)` |
| Output Shape | `(1,)` |
| Output Range | `[-1.0, 1.0]` (tanh activation) |

## Layer Architecture

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┓
┃ Layer (type)                         ┃ Output Shape                ┃         Param # ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━┩
│ sensor_sequence (InputLayer)         │ (None, 20, 25)              │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ feature_mixing (TimeDistributed)     │ (None, 20, 64)              │           1,664 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ bn_feature (TimeDistributed)         │ (None, 20, 64)              │             256 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ lstm_1 (LSTM)                        │ (None, 20, 64)              │          33,024 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dropout (Dropout)                    │ (None, 20, 64)              │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ lstm_2 (LSTM)                        │ (None, 32)                  │          12,416 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dropout_1 (Dropout)                  │ (None, 32)                  │               0 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ dense_out (Dense)                    │ (None, 16)                  │             528 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ bn_dense (BatchNormalization)        │ (None, 16)                  │              64 │
├──────────────────────────────────────┼─────────────────────────────┼─────────────────┤
│ rudder_command (Dense)               │ (None, 1)                   │              17 │
└──────────────────────────────────────┴─────────────────────────────┴─────────────────┘
```

### Layer Details

#### 1. Input Layer (`sensor_sequence`)
- **Shape**: `(batch, 20, 25)`
- **Description**: Accepts a sequence of 20 timesteps, each with 25 features
- **Sample Rate**: 10 Hz (2 seconds of history)

#### 2. Feature Mixing (`feature_mixing`)
- **Type**: TimeDistributed Dense
- **Units**: 64
- **Activation**: ReLU
- **Parameters**: 1,664 (25×64 weights + 64 biases)
- **Purpose**: Learn feature interactions at each timestep independently

#### 3. Feature BatchNorm (`bn_feature`)
- **Type**: TimeDistributed BatchNormalization
- **Parameters**: 256 (64×4: gamma, beta, moving_mean, moving_var)
- **Purpose**: Normalize feature representations for stable training

#### 4. First LSTM (`lstm_1`)
- **Units**: 64
- **Return Sequences**: True (outputs sequence for stacking)
- **Parameters**: 33,024
- **Purpose**: Capture temporal dependencies in the input sequence

#### 5. Dropout
- **Rate**: 0.2 (20%)
- **Purpose**: Regularization to prevent overfitting

#### 6. Second LSTM (`lstm_2`)
- **Units**: 32
- **Return Sequences**: False (outputs final state only)
- **Parameters**: 12,416
- **Purpose**: Further temporal processing, compressing to fixed representation

#### 7. Dropout
- **Rate**: 0.2 (20%)
- **Purpose**: Regularization

#### 8. Dense Output (`dense_out`)
- **Units**: 16
- **Activation**: ReLU
- **Parameters**: 528 (32×16 + 16)
- **Purpose**: Map LSTM output to output space

#### 9. Dense BatchNorm (`bn_dense`)
- **Parameters**: 64 (16×4)
- **Purpose**: Stabilize output layer training

#### 10. Rudder Command (`rudder_command`)
- **Units**: 1
- **Activation**: tanh
- **Parameters**: 17 (16×1 + 1)
- **Purpose**: Final output, bounded to [-1, 1]

## Input Features (25 total)

All features are normalized to the range [-1, 1].

| Index | Feature | Description | Max Value |
|-------|---------|-------------|-----------|
| 0 | `heading_error` | Error from target heading/angle | ±180° |
| 1 | `heading_error_integral` | Accumulated heading error | ±180° |
| 2 | `heading_rate` | Yaw rate from IMU | ±30°/s |
| 3 | `roll` | Heel angle | ±45° |
| 4 | `pitch` | Pitch angle | ±30° |
| 5 | `roll_rate` | Roll rate from IMU | ±30°/s |
| 6 | `awa` | Apparent wind angle | ±180° |
| 7 | `awa_rate` | Rate of change of AWA | ±10°/s |
| 8 | `aws` | Apparent wind speed | 0-60 kts |
| 9 | `twa` | True wind angle (computed) | ±180° |
| 10 | `tws` | True wind speed (computed) | 0-60 kts |
| 11 | `stw` | Speed through water | 0-25 kts |
| 12 | `sog` | Speed over ground | 0-25 kts |
| 13 | `cog_error` | COG error from target | ±180° |
| 14 | `rudder_position` | Current rudder angle | ±30° |
| 15 | `rudder_velocity` | Rudder movement rate | ±10°/s |
| 16 | `target_angle` | Target angle for current mode | ±180° |
| 17 | `vmg_up` | VMG upwind | 0-15 kts |
| 18 | `vmg_down` | VMG downwind | 0-20 kts |
| 19 | `polar_target` | Target speed from polar | 0-25 kts |
| 20 | `polar_performance` | Current performance ratio | 0-1.2 |
| 21 | `mode_compass` | Flag: compass mode active | 0 or 1 |
| 22 | `mode_wind_awa` | Flag: AWA mode active | 0 or 1 |
| 23 | `mode_wind_twa` | Flag: TWA/VMG mode active | 0 or 1 |
| 24 | `wave_period` | Estimated wave period | 2-15 s |

## Output

- **Shape**: `(1,)` - single scalar value
- **Range**: `[-1.0, 1.0]`
- **Interpretation**: Rudder command as fraction of maximum deflection
  - `-1.0` = full port (−30°)
  - `0.0` = rudder centered
  - `+1.0` = full starboard (+30°)

## Loss Function

```python
def autopilot_loss(y_true, y_pred):
    """Mean Squared Error on rudder position."""
    mse = tf.reduce_mean(tf.square(y_true - y_pred))
    return mse
```

The model is trained to minimize MSE between predicted and actual human rudder commands.

## Training Configuration

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `epochs` | 100 | Maximum training epochs |
| `batch_size` | 64 | Samples per batch |
| `learning_rate` | 1e-4 | Initial learning rate |
| `warmup_epochs` | 3 | LR warmup from 1e-5 to 1e-4 |
| `patience` | 8 | Early stopping patience |
| `min_delta` | 0.0001 | Minimum improvement threshold |
| `noise_std` | 0.02 | Data augmentation noise |
| `dropout_rate` | 0.2 | Dropout probability |

### Optimizer

- **Type**: Adam
- **Gradient Clipping**: clipnorm=1.0 (prevents exploding gradients)

### Callbacks

1. **Learning Rate Warmup**: Linear warmup over first 3 epochs
2. **ReduceLROnPlateau**: Halve LR after 3 epochs without improvement (min 1e-6)
3. **EarlyStopping**: Stop after 8 epochs without improvement, restore best weights
4. **ModelCheckpoint**: Save best model based on validation loss

## Deployment

### TensorFlow Lite Conversion

The trained model is converted to TensorFlow Lite for efficient inference on Raspberry Pi:

```python
convert_to_tflite(model, "autopilot.tflite", quantize=True)
```

- **Quantization**: Dynamic range quantization (float16/int8)
- **Model Size**: ~50 KB (quantized)
- **Inference Time**: ~1-2 ms on Raspberry Pi 4

### Inference API

```python
from src.ml.autopilot_model import AutopilotInference

inference = AutopilotInference("models/autopilot.tflite")

# sequence: numpy array of shape (20, 25)
rudder_command = inference.predict(sequence)  # Returns float in [-1, 1]
```

## Model Configuration

The model architecture is configurable via `ModelConfig`:

```python
@dataclass
class ModelConfig:
    sequence_length: int = 20      # Timesteps of history
    feature_dim: int = 25          # Features per timestep
    lstm_units_1: int = 64         # First LSTM layer
    lstm_units_2: int = 32         # Second LSTM layer
    dense_units: int = 16          # Dense layer before output
    dropout_rate: float = 0.2      # Dropout for regularization
```

## Design Rationale

1. **LSTM Architecture**: Chosen for its ability to learn temporal dependencies in sailing dynamics (wave patterns, boat response lag, wind shifts)

2. **Two LSTM Layers**: Stacked LSTMs can learn hierarchical temporal features - low-level (immediate response) and high-level (trend recognition)

3. **TimeDistributed Feature Mixing**: Allows the model to learn feature interactions before temporal processing

4. **BatchNormalization**: Stabilizes training and allows faster convergence

5. **tanh Output**: Naturally bounds output to [-1, 1], matching rudder deflection limits

6. **Sequence Length of 20**: At 10 Hz, this represents 2 seconds of history - enough to capture boat response dynamics while remaining responsive

7. **Dropout**: Prevents overfitting to specific sailing conditions, improving generalization
