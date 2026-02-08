# Model Export: ONNX for Edge Deployment

This document describes the model export format for deploying the autopilot model on Raspberry Pi.

## Overview

The autopilot model is trained using PyTorch and exported to ONNX format for efficient inference on the Raspberry Pi. ONNX (Open Neural Network Exchange) provides a portable, optimized format that runs efficiently on ARM processors.

## Why ONNX

- **Lightweight runtime**: ONNX Runtime is ~16 MB vs ~500 MB for PyTorch
- **Optimized for ARM**: Uses SIMD (NEON) acceleration automatically
- **Framework-agnostic**: No PyTorch dependency needed for inference
- **Fast inference**: Sub-millisecond on Raspberry Pi 4

## Export Process

After training, the model is automatically exported to ONNX:

```python
from src.ml.autopilot_model import build_autopilot_model, convert_to_onnx

# Build and train model
model = build_autopilot_model()
# ... training ...

# Export to ONNX
convert_to_onnx(model, 'models/autopilot.onnx')
```

The training script (`train_imitation.py`) exports both formats automatically:
- `autopilot.pt` - PyTorch checkpoint (for training, evaluation, fine-tuning)
- `autopilot.onnx` - ONNX format (for Raspberry Pi deployment)

### Export Parameters

The ONNX export uses:
- **opset_version=14**: For good compatibility with ONNX Runtime
- **dynamo=False**: Uses the legacy TorchScript exporter for LSTM compatibility
- **dynamic_axes**: Supports variable batch sizes

## Performance

Benchmarks on Apple M2 Pro (similar ARM architecture to Pi):

| Format | File Size | Inference Time | Memory Footprint |
|--------|-----------|----------------|------------------|
| PyTorch (.pt) | ~400 KB | ~1 ms | ~200 MB |
| ONNX (.onnx) | ~240 KB | 0.11 ms | ~20-50 MB |

ONNX Runtime is optimized for inference with:
- Graph optimizations (constant folding, node fusion)
- SIMD vectorization (NEON on ARM)
- Minimal runtime overhead

## Raspberry Pi Deployment

### Installation

On the Raspberry Pi, install dependencies using the `rpi` extra (no PyTorch required):

```bash
uv sync --extra rpi
```

Or install ONNX Runtime directly:

```bash
pip install onnxruntime
```

This is ~16 MB vs ~200+ MB for PyTorch.

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

## Manual Export

To manually export an existing model:

```python
from src.ml.autopilot_model import load_model, convert_to_onnx

# Load trained PyTorch model
model = load_model('models/autopilot.pt')

# Export to ONNX
convert_to_onnx(model, 'models/autopilot.onnx')
```

## GPU Considerations

The Raspberry Pi has a VideoCore GPU (not NVIDIA/CUDA). For our model size (~240 KB):
- CPU with NEON SIMD is typically **faster** than the VideoCore GPU
- GPU delegate overhead isn't worthwhile for models under ~1 MB
- ONNX Runtime uses XNNPACK for ARM SIMD acceleration automatically

## Dependencies

In `pyproject.toml`:

**For training (main dependencies):**
- `torch>=2.2.0` - PyTorch framework
- `onnx==1.17.0` - ONNX model format
- `onnxscript>=0.6.0` - Required by PyTorch ONNX export
- `onnxruntime>=1.20.1` - Inference runtime

**For Pi deployment (rpi extra):**
- `onnxruntime>=1.20.1` - Inference runtime only

## References

- [ONNX Runtime Documentation](https://onnxruntime.ai/)
- [PyTorch ONNX Export](https://pytorch.org/docs/stable/onnx.html)
- [ONNX Runtime ARM Optimization](https://onnxruntime.ai/docs/execution-providers/CPU-Execution-Provider.html)
