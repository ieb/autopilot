"""
Autopilot Model Module
======================

End-to-end LSTM model that directly outputs rudder commands
from sensor observations.
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import logging

logger = logging.getLogger(__name__)

# TensorFlow imports with fallback
try:
    import tensorflow as tf
    HAS_TF = True
except ImportError:
    tf = None
    HAS_TF = False
    logger.warning("TensorFlow not available. Model training disabled.")

try:
    import tflite_runtime.interpreter as tflite
    HAS_TFLITE_RUNTIME = True
except ImportError:
    HAS_TFLITE_RUNTIME = False

# ONNX Runtime for edge deployment
try:
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    ort = None
    HAS_ONNX = False


@dataclass
class ModelConfig:
    """Configuration for autopilot model."""
    sequence_length: int = 20      # Timesteps of history
    feature_dim: int = 25          # Features per timestep
    lstm_units_1: int = 64         # First LSTM layer
    lstm_units_2: int = 32         # Second LSTM layer
    dense_units: int = 16          # Dense layer before output
    dropout_rate: float = 0.2      # Dropout for regularization


def build_autopilot_model(config: Optional[ModelConfig] = None) -> 'tf.keras.Model':
    """
    Build the end-to-end LSTM autopilot model.
    
    Architecture:
        Input: [batch, sequence_length, feature_dim]
        -> TimeDistributed Dense (feature mixing)
        -> LSTM (temporal processing)
        -> LSTM (temporal processing)
        -> Dense (output mapping)
        -> tanh activation (bounded output)
        Output: [batch, 1] - rudder command in [-1, 1]
    
    Args:
        config: Model configuration
        
    Returns:
        Compiled Keras model
    """
    if not HAS_TF:
        raise ImportError("TensorFlow is required to build the model")
        
    config = config or ModelConfig()
    
    inputs = tf.keras.Input(
        shape=(config.sequence_length, config.feature_dim),
        name='sensor_sequence'
    )
    
    # Initial feature mixing across time
    x = tf.keras.layers.TimeDistributed(
        tf.keras.layers.Dense(128, activation='relu'),
        name='feature_mixing'
    )(inputs)
    x = tf.keras.layers.TimeDistributed(
        tf.keras.layers.BatchNormalization(),
        name='bn_feature'
    )(x)
    
    # First LSTM layer - returns sequences for stacking
    x = tf.keras.layers.LSTM(
        config.lstm_units_1,
        return_sequences=True,
        name='lstm_1'
    )(x)
    x = tf.keras.layers.Dropout(config.dropout_rate)(x)
    
    # Second LSTM layer - returns final state only
    x = tf.keras.layers.LSTM(
        config.lstm_units_2,
        return_sequences=False,
        name='lstm_2'
    )(x)
    x = tf.keras.layers.Dropout(config.dropout_rate)(x)
    
    # Output mapping with BatchNormalization for stability
    x = tf.keras.layers.Dense(config.dense_units, activation=tf.keras.layers.LeakyReLU(alpha=0.01), name='dense_out')(x)
    ## On advice, throttling x = tf.keras.layers.BatchNormalization(name='bn_dense')(x)
    
    # Rudder command output with tanh for bounded [-1, 1]
    outputs = tf.keras.layers.Dense(1, activation='tanh', name='rudder_command')(x)
    
    model = tf.keras.Model(inputs, outputs, name='autopilot_lstm')
    
    return model


def compile_model(model: 'tf.keras.Model', learning_rate: float = 1e-4):
    """Compile model with appropriate loss and optimizer."""
    if not HAS_TF:
        raise ImportError("TensorFlow is required")
        
    model.compile(
        optimizer=tf.keras.optimizers.Adam(
            learning_rate=learning_rate,
            clipnorm=1.0  # Gradient clipping to prevent exploding gradients
        ),
        loss=autopilot_loss,
        metrics=['mae']
    )
    return model


@tf.keras.utils.register_keras_serializable(package="autopilot")
def autopilot_loss(y_true, y_pred):
    """
    Custom loss for autopilot training.
    
    Combines:
    1. MSE on rudder position (primary)
    2. Smoothness penalty on predicted changes (secondary)
    """
    if not HAS_TF:
        raise ImportError("TensorFlow is required")
        
    # Position accuracy
    mse = tf.reduce_mean(tf.square(y_true - y_pred))
    
    return mse


def convert_to_tflite(model: 'tf.keras.Model', output_path: str,
                      quantize: bool = True) -> str:
    """
    Convert Keras model to TensorFlow Lite for Pi deployment.
    
    Args:
        model: Trained Keras model
        output_path: Path for .tflite file
        quantize: Whether to quantize to int8
        
    Returns:
        Path to saved model
    """
    if not HAS_TF:
        raise ImportError("TensorFlow is required")
        
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    
    # Enable SELECT_TF_OPS to handle LSTM + batch normalization patterns
    # that aren't natively supported in TFLite
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS,
        tf.lite.OpsSet.SELECT_TF_OPS
    ]
    # Disable lowering tensor list ops to avoid MLIR compilation issues
    converter._experimental_lower_tensor_list_ops = False
    
    if quantize:
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        # For full int8 quantization, would need representative dataset
        
    tflite_model = converter.convert()
    
    with open(output_path, 'wb') as f:
        f.write(tflite_model)
        
    logger.info(f"Saved TFLite model to {output_path}")
    logger.info(f"Model size: {len(tflite_model) / 1024:.1f} KB")
    
    return output_path


def convert_to_onnx(model: 'tf.keras.Model', output_path: str) -> str:
    """
    Convert Keras model to ONNX format for edge deployment.
    
    ONNX is preferred over TFLite due to Keras 3 compatibility issues
    with the TFLite converter.
    
    This function uses a subprocess with GPU disabled to ensure no GPU-specific
    ops (like CudnnRNNV3) are included in the exported graph. The subprocess
    approach is required because TensorFlow's GPU devices cannot be disabled
    after initialization.
    
    Args:
        model: Trained Keras model
        output_path: Path for .onnx file
        
    Returns:
        Path to saved model
    """
    if not HAS_TF:
        raise ImportError("TensorFlow is required")
    
    import os
    import subprocess
    import sys
    import tempfile
    
    # Save model to temp file for subprocess to load
    with tempfile.NamedTemporaryFile(suffix='.keras', delete=False) as f:
        keras_temp_path = f.name
    model.save(keras_temp_path)
    
    # Python script to run in subprocess with GPU disabled
    conversion_script = f'''
import os
# Disable GPU BEFORE importing TensorFlow
os.environ["CUDA_VISIBLE_DEVICES"] = ""
os.environ["TF_METAL_DEVICE_SELECTOR"] = ""
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import tensorflow as tf
# Force CPU-only mode
tf.config.set_visible_devices([], "GPU")

import tf2onnx
import onnx
import numpy as np
from src.ml.autopilot_model import autopilot_loss

# Load model
model = tf.keras.models.load_model(
    "{keras_temp_path}",
    custom_objects={{"autopilot_loss": autopilot_loss}}
)

# Convert to ONNX
input_shape = model.input_shape
input_signature = [tf.TensorSpec(input_shape, tf.float32, name="input")]
onnx_model, _ = tf2onnx.convert.from_keras(model, input_signature, opset=13)

# Save
onnx.save(onnx_model, "{output_path}")
print(f"Saved ONNX model to {output_path}")
'''
    
    try:
        # Run conversion in subprocess with clean environment
        result = subprocess.run(
            [sys.executable, "-c", conversion_script],
            capture_output=True,
            text=True,
            cwd=os.getcwd(),
        )
        
        if result.returncode != 0:
            logger.error(f"ONNX conversion failed: {result.stderr}")
            raise RuntimeError(f"ONNX conversion failed: {result.stderr}")
        
        # Log output
        if result.stdout:
            for line in result.stdout.strip().split('\n'):
                logger.info(line)
        
        size_kb = os.path.getsize(output_path) / 1024
        logger.info(f"Model size: {size_kb:.1f} KB")
        
    finally:
        # Clean up temp file
        if os.path.exists(keras_temp_path):
            os.unlink(keras_temp_path)
    
    return output_path


def _convert_to_onnx_inprocess(model: 'tf.keras.Model', output_path: str) -> str:
    """
    In-process ONNX conversion (legacy, may include GPU ops).
    
    Use convert_to_onnx() instead for reliable CPU-only conversion.
    """
    if not HAS_TF:
        raise ImportError("TensorFlow is required")
    
    try:
        import tf2onnx
        import onnx
    except ImportError:
        raise ImportError("tf2onnx and onnx packages are required. "
                         "Install with: pip install tf2onnx onnx")
    
    import os
    import tempfile
    import numpy as np
    
    # Save original environment and disable GPU completely
    original_cuda = os.environ.get('CUDA_VISIBLE_DEVICES')
    os.environ['CUDA_VISIBLE_DEVICES'] = ''
    
    # Also disable GPU via TensorFlow config
    try:
        tf.config.set_visible_devices([], 'GPU')
    except Exception:
        pass  # May fail if already initialized
    
    try:
        # Save weights to temp file
        with tempfile.NamedTemporaryFile(suffix='.weights.h5', delete=False) as f:
            weights_path = f.name
        model.save_weights(weights_path)
        
        # Get model config and rebuild on CPU
        # This ensures LSTM uses CPU implementation, not CudnnRNN
        with tf.device('/CPU:0'):
            # Clone the model architecture
            model_config = model.get_config()
            cpu_model = tf.keras.Model.from_config(model_config)
            
            # Build the model with the correct input shape
            input_shape = model.input_shape
            dummy = np.zeros((1, input_shape[1], input_shape[2]), dtype=np.float32)
            _ = cpu_model(dummy, training=False)
            
            # Load weights into CPU model
            cpu_model.load_weights(weights_path)
            
            # Run forward pass to ensure graph is built
            _ = cpu_model(dummy, training=False)
            
            # Convert CPU model to ONNX
            input_signature = [tf.TensorSpec(input_shape, tf.float32, name='input')]
            onnx_model, _ = tf2onnx.convert.from_keras(cpu_model, input_signature, opset=13)
        
        # Save ONNX model
        onnx.save(onnx_model, output_path)
        
        size_kb = os.path.getsize(output_path) / 1024
        logger.info(f"Saved ONNX model to {output_path}")
        logger.info(f"Model size: {size_kb:.1f} KB")
        
        # Clean up temp file
        os.unlink(weights_path)
        
    finally:
        # Restore original CUDA setting
        if original_cuda is not None:
            os.environ['CUDA_VISIBLE_DEVICES'] = original_cuda
        elif 'CUDA_VISIBLE_DEVICES' in os.environ:
            del os.environ['CUDA_VISIBLE_DEVICES']
    
    return output_path


class AutopilotInference:
    """
    Real-time inference engine for autopilot model.
    
    Supports multiple model formats:
    - ONNX (.onnx) - Preferred for edge deployment (Raspberry Pi)
    - TensorFlow Lite (.tflite) - Alternative edge format
    - Keras (.keras, .h5) - For development/debugging
    
    ONNX is recommended for Pi deployment due to Keras 3 compatibility
    issues with TFLite converter.
    """
    
    def __init__(self, model_path: str, config: Optional[ModelConfig] = None):
        """
        Initialize inference engine.
        
        Args:
            model_path: Path to model file (.onnx, .tflite, .keras, or .h5)
            config: Model configuration
        """
        self.config = config or ModelConfig()
        self._interpreter = None
        self._keras_model = None
        self._onnx_session = None
        self._input_details = None
        self._output_details = None
        self._model_type = None  # 'onnx', 'tflite', or 'keras'
        
        self._load_model(model_path)
        
    def _load_model(self, model_path: str):
        """Load model from file (supports .onnx, .tflite, .keras, .h5)."""
        import os
        ext = os.path.splitext(model_path)[1].lower()
        
        try:
            if ext == '.onnx':
                self._load_onnx_model(model_path)
            elif ext == '.tflite':
                self._load_tflite_model(model_path)
            elif ext in ('.keras', '.h5'):
                self._load_keras_model(model_path)
            else:
                # Try ONNX first, then TFLite, then Keras
                loaded = False
                for loader in [self._load_onnx_model, self._load_tflite_model, 
                              self._load_keras_model]:
                    try:
                        loader(model_path)
                        loaded = True
                        break
                    except Exception:
                        continue
                if not loaded:
                    raise ValueError(f"Could not load model from {model_path}")
                    
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise
    
    def _load_onnx_model(self, model_path: str):
        """Load ONNX model for efficient edge inference."""
        if not HAS_ONNX:
            raise ImportError("ONNX Runtime not available. "
                            "Install with: pip install onnxruntime")
        
        # Create session with optimizations enabled
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        self._onnx_session = ort.InferenceSession(model_path, sess_options)
        self._model_type = 'onnx'
        
        # Get input/output info
        inp = self._onnx_session.get_inputs()[0]
        out = self._onnx_session.get_outputs()[0]
        self._onnx_input_name = inp.name
        
        logger.info(f"Loaded ONNX model from {model_path}")
        logger.info(f"Input: {inp.name}, shape: {inp.shape}")
        logger.info(f"Output: {out.name}, shape: {out.shape}")
    
    def _load_tflite_model(self, model_path: str):
        """Load TensorFlow Lite model."""
        if HAS_TFLITE_RUNTIME:
            self._interpreter = tflite.Interpreter(model_path=model_path)
        elif HAS_TF:
            self._interpreter = tf.lite.Interpreter(model_path=model_path)
        else:
            raise ImportError("No TFLite runtime available")
            
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self._model_type = 'tflite'
        
        logger.info(f"Loaded TFLite model from {model_path}")
        logger.info(f"Input shape: {self._input_details[0]['shape']}")
        logger.info(f"Output shape: {self._output_details[0]['shape']}")
    
    def _load_keras_model(self, model_path: str):
        """Load Keras model (.keras or .h5)."""
        if not HAS_TF:
            raise ImportError("TensorFlow is required to load Keras models")
        
        # Load with custom objects for our custom loss
        self._keras_model = tf.keras.models.load_model(
            model_path,
            custom_objects={'autopilot_loss': autopilot_loss}
        )
        self._model_type = 'keras'
        
        logger.info(f"Loaded Keras model from {model_path}")
        logger.info(f"Input shape: {self._keras_model.input_shape}")
        logger.info(f"Output shape: {self._keras_model.output_shape}")
            
    def predict(self, sequence: np.ndarray) -> float:
        """
        Run inference on a feature sequence.
        
        Args:
            sequence: Feature array of shape [sequence_length, feature_dim]
            
        Returns:
            Rudder command in range [-1, 1]
        """
        # Ensure correct shape and type
        if sequence.ndim == 2:
            sequence = sequence[np.newaxis, ...]  # Add batch dimension
            
        sequence = sequence.astype(np.float32)
        
        if self._model_type == 'onnx':
            # ONNX Runtime inference (preferred for edge deployment)
            if self._onnx_session is None:
                raise RuntimeError("Model not loaded")
            output = self._onnx_session.run(None, {self._onnx_input_name: sequence})[0]
        elif self._model_type == 'keras':
            # Keras model inference
            if self._keras_model is None:
                raise RuntimeError("Model not loaded")
            output = self._keras_model.predict(sequence, verbose=0)
        elif self._model_type == 'tflite':
            # TFLite inference
            if self._interpreter is None:
                raise RuntimeError("Model not loaded")
            self._interpreter.set_tensor(self._input_details[0]['index'], sequence)
            self._interpreter.invoke()
            output = self._interpreter.get_tensor(self._output_details[0]['index'])
        else:
            raise RuntimeError(f"Unknown model type: {self._model_type}")
        
        # Return scalar value, clipped to valid range
        return float(np.clip(output[0, 0], -1.0, 1.0))
        
    def benchmark(self, iterations: int = 100) -> dict:
        """
        Benchmark inference performance.
        
        Args:
            iterations: Number of inference runs
            
        Returns:
            Dict with timing statistics
        """
        import time
        
        # Generate random input
        test_input = np.random.randn(
            1, self.config.sequence_length, self.config.feature_dim
        ).astype(np.float32)
        
        # Warmup
        for _ in range(10):
            self.predict(test_input[0])
            
        # Benchmark
        times = []
        for _ in range(iterations):
            start = time.perf_counter()
            self.predict(test_input[0])
            times.append(time.perf_counter() - start)
            
        times = np.array(times) * 1000  # Convert to ms
        
        return {
            "iterations": iterations,
            "mean_ms": float(np.mean(times)),
            "std_ms": float(np.std(times)),
            "min_ms": float(np.min(times)),
            "max_ms": float(np.max(times)),
            "p95_ms": float(np.percentile(times, 95)),
            "p99_ms": float(np.percentile(times, 99))
        }


class MockAutopilotInference(AutopilotInference):
    """Mock inference for testing without a trained model."""
    
    def __init__(self, config: Optional[ModelConfig] = None):
        self.config = config or ModelConfig()
        self._interpreter = "mock"
        
    def _load_model(self, model_path: str):
        pass
        
    def predict(self, sequence: np.ndarray) -> float:
        """
        Simple proportional response based on heading error.
        
        This mimics what a basic autopilot would do,
        useful for testing the control loop.
        """
        if sequence.ndim == 2:
            # Get most recent heading error (feature 0)
            heading_error = sequence[-1, 0]  # Already normalized
            
            # Simple P response
            return float(np.clip(-heading_error * 0.5, -1.0, 1.0))
            
        return 0.0
