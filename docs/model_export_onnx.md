# Model Export: ONNX for Edge Deployment

This document describes the model export format for deploying the autopilot model on Raspberry Pi.

## Background

After training completes, the model needs to be exported to a lightweight format for efficient inference on the Raspberry Pi. The original plan was to use TensorFlow Lite (TFLite), but compatibility issues led us to adopt ONNX instead.

## TFLite Compatibility Issue

### The Error

After training, TFLite conversion failed with:

```
loc(fused["ReadVariableOp:", "autopilot_lstm_1/bn_feature_1/Cast_11/ReadVariableOp@__inference_serving_default_911242"]): error: missing attribute 'value'
LLVM ERROR: Failed to infer result type(s).
```

### Root Cause

This is a **known bug in Keras 3** affecting TFLite conversion:
- GitHub Issue: [keras-team/keras#19108](https://github.com/keras-team/keras/issues/19108)
- Reported: January 2024
- Status: Still unresolved as of January 2026
- Affects: TensorFlow 2.15+ with Keras 3.x

The bug occurs during the MLIR compilation phase of TFLite conversion. It affects even simple Dense layers, not just our LSTM + BatchNormalization architecture.

### Attempted Workarounds

The following approaches were tried and **did not work**:

1. **SELECT_TF_OPS fallback** - Same LLVM error
2. **Concrete function export** - Same LLVM error
3. **Legacy converter** - Incompatible with Keras 3
4. **Removing BatchNormalization** - Same error on Dense layers

## Solution: ONNX Export

ONNX (Open Neural Network Exchange) provides a working alternative for edge deployment.

### Why ONNX Works

- Uses a different compilation pathway (tf2onnx) that doesn't have the MLIR bug
- Wide industry support and active maintenance
- Optimized runtime for ARM/Raspberry Pi via ONNX Runtime

### Export Process

The model is exported with CPU-only operations to avoid GPU-specific ops (CudnnRNN) that wouldn't run on the Pi:

```python
from src.ml.autopilot_model import convert_to_onnx, autopilot_loss
import tensorflow as tf

model = tf.keras.models.load_model(
    'models/autopilot.keras',
    custom_objects={'autopilot_loss': autopilot_loss}
)
convert_to_onnx(model, 'models/autopilot.onnx')
```

The training script now automatically exports both formats:
- `autopilot.keras` - For training, evaluation, fine-tuning
- `autopilot.onnx` - For Raspberry Pi deployment

## Performance Comparison

Benchmarks on Apple M2 Pro (similar ARM architecture to Pi):

| Format | File Size | Inference Time | Memory Footprint |
|--------|-----------|----------------|------------------|
| Keras (.keras) | 626 KB | 27.53 ms | ~600 MB (TensorFlow) |
| ONNX (.onnx) | 240 KB | 0.11 ms | ~20-50 MB |

**ONNX is 243x faster** due to:
- Optimized inference-only runtime
- No Python/Keras overhead
- SIMD optimizations (NEON on ARM)

### Numerical Accuracy

ONNX outputs match Keras exactly:
- Maximum difference: 0.00000019
- Mean difference: 0.00000008

## Raspberry Pi Deployment

### Installation

On the Raspberry Pi, install dependencies using the `rpi` extra (no TensorFlow required):

```bash
uv sync --extra rpi
```

Or install ONNX Runtime directly:

```bash
pip install onnxruntime
```

This is ~16 MB vs ~500 MB for TensorFlow.

### Usage

```python
from src.ml.autopilot_model import AutopilotInference

# Load ONNX model
autopilot = AutopilotInference('autopilot.onnx')

# Run inference
# Input: sensor sequence of shape [20, 25] (sequence_length, features)
# Output: rudder command in range [-1, 1]
rudder_command = autopilot.predict(sensor_sequence)
```

### Performance Optimization

ONNX Runtime automatically uses graph optimizations. For best performance:

```python
import onnxruntime as ort

sess_options = ort.SessionOptions()
sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
session = ort.InferenceSession('autopilot.onnx', sess_options)
```

## GPU Considerations

The Raspberry Pi has a VideoCore GPU (not NVIDIA/CUDA). For our model size (~240 KB):
- CPU with NEON SIMD is typically **faster** than the VideoCore GPU
- GPU delegate overhead isn't worthwhile for models under ~1 MB
- ONNX Runtime uses XNNPACK for ARM SIMD acceleration automatically

## Files Changed

The following files were updated to support ONNX:

1. **`src/ml/autopilot_model.py`**
   - Added `convert_to_onnx()` function
   - Updated `AutopilotInference` to load `.onnx` files
   - Added ONNX Runtime detection (`HAS_ONNX`)

2. **`src/training/train_imitation.py`**
   - Updated `save_model()` to export ONNX instead of TFLite

## Dependencies

Added to `pyproject.toml`:
- `onnx==1.16.0` - ONNX model format
- `tf2onnx>=1.16.0` - TensorFlow to ONNX converter
- `onnxruntime` - Inference runtime

## Future Considerations

If the Keras 3 TFLite bug is fixed in a future TensorFlow release:
- TFLite could be reconsidered for smaller model size
- The `convert_to_tflite()` function still exists but is currently broken
- ONNX remains a solid choice regardless

## References

- [ONNX Runtime Documentation](https://onnxruntime.ai/)
- [tf2onnx GitHub](https://github.com/onnx/tensorflow-onnx)
- [Keras TFLite Bug #19108](https://github.com/keras-team/keras/issues/19108)
