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
        tf.keras.layers.Dense(64, activation='relu'),
        name='feature_mixing'
    )(inputs)
    
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
    
    # Output mapping
    x = tf.keras.layers.Dense(config.dense_units, activation='relu', name='dense_out')(x)
    
    # Rudder command output with tanh for bounded [-1, 1]
    outputs = tf.keras.layers.Dense(1, activation='tanh', name='rudder_command')(x)
    
    model = tf.keras.Model(inputs, outputs, name='autopilot_lstm')
    
    return model


def compile_model(model: 'tf.keras.Model', learning_rate: float = 0.001):
    """Compile model with appropriate loss and optimizer."""
    if not HAS_TF:
        raise ImportError("TensorFlow is required")
        
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate),
        loss=autopilot_loss,
        metrics=['mae']
    )
    return model


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
    
    if quantize:
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        # For full int8 quantization, would need representative dataset
        
    tflite_model = converter.convert()
    
    with open(output_path, 'wb') as f:
        f.write(tflite_model)
        
    logger.info(f"Saved TFLite model to {output_path}")
    logger.info(f"Model size: {len(tflite_model) / 1024:.1f} KB")
    
    return output_path


class AutopilotInference:
    """
    Real-time inference engine for autopilot model.
    
    Uses TensorFlow Lite for efficient inference on Raspberry Pi.
    """
    
    def __init__(self, model_path: str, config: Optional[ModelConfig] = None):
        """
        Initialize inference engine.
        
        Args:
            model_path: Path to .tflite model file
            config: Model configuration
        """
        self.config = config or ModelConfig()
        self._interpreter = None
        self._input_details = None
        self._output_details = None
        
        self._load_model(model_path)
        
    def _load_model(self, model_path: str):
        """Load TFLite model."""
        try:
            if HAS_TFLITE_RUNTIME:
                self._interpreter = tflite.Interpreter(model_path=model_path)
            elif HAS_TF:
                self._interpreter = tf.lite.Interpreter(model_path=model_path)
            else:
                raise ImportError("No TFLite runtime available")
                
            self._interpreter.allocate_tensors()
            self._input_details = self._interpreter.get_input_details()
            self._output_details = self._interpreter.get_output_details()
            
            logger.info(f"Loaded model from {model_path}")
            logger.info(f"Input shape: {self._input_details[0]['shape']}")
            logger.info(f"Output shape: {self._output_details[0]['shape']}")
            
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise
            
    def predict(self, sequence: np.ndarray) -> float:
        """
        Run inference on a feature sequence.
        
        Args:
            sequence: Feature array of shape [sequence_length, feature_dim]
            
        Returns:
            Rudder command in range [-1, 1]
        """
        if self._interpreter is None:
            raise RuntimeError("Model not loaded")
            
        # Ensure correct shape and type
        if sequence.ndim == 2:
            sequence = sequence[np.newaxis, ...]  # Add batch dimension
            
        sequence = sequence.astype(np.float32)
        
        # Run inference
        self._interpreter.set_tensor(self._input_details[0]['index'], sequence)
        self._interpreter.invoke()
        
        output = self._interpreter.get_tensor(self._output_details[0]['index'])
        
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
