"""
Autopilot Model Module
======================

End-to-end LSTM model that directly outputs rudder commands
from sensor observations.

Uses PyTorch for training and exports to ONNX for edge deployment.
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np
import logging
import os

logger = logging.getLogger(__name__)

# PyTorch imports with fallback
try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True
except ImportError:
    torch = None
    nn = None
    HAS_TORCH = False
    logger.warning("PyTorch not available. Model training disabled.")

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


class AutopilotLSTM(nn.Module):
    """
    End-to-end LSTM autopilot model.
    
    Architecture:
        Input: [batch, sequence_length, feature_dim]
        -> Linear + ReLU (feature mixing)
        -> BatchNorm
        -> LSTM (temporal processing)
        -> Dropout
        -> LSTM (temporal processing)
        -> Dropout
        -> Linear + LeakyReLU
        -> Linear + Tanh (bounded output)
        Output: [batch, 1] - rudder command in [-1, 1]
    """
    
    def __init__(self, config: Optional[ModelConfig] = None):
        """
        Initialize the autopilot model.
        
        Args:
            config: Model configuration
        """
        if not HAS_TORCH:
            raise ImportError("PyTorch is required to build the model")
            
        super().__init__()
        self.config = config or ModelConfig()
        
        # Feature mixing layer (applied to each timestep)
        self.feature_mix = nn.Sequential(
            nn.Linear(self.config.feature_dim, 128),
            nn.ReLU(),
        )
        
        # Batch normalization across sequence dimension
        self.bn = nn.BatchNorm1d(self.config.sequence_length)
        
        # First LSTM layer
        self.lstm1 = nn.LSTM(
            input_size=128,
            hidden_size=self.config.lstm_units_1,
            batch_first=True
        )
        self.dropout1 = nn.Dropout(self.config.dropout_rate)
        
        # Second LSTM layer
        self.lstm2 = nn.LSTM(
            input_size=self.config.lstm_units_1,
            hidden_size=self.config.lstm_units_2,
            batch_first=True
        )
        self.dropout2 = nn.Dropout(self.config.dropout_rate)
        
        # Output layers
        self.output = nn.Sequential(
            nn.Linear(self.config.lstm_units_2, self.config.dense_units),
            nn.LeakyReLU(0.01),
            nn.Linear(self.config.dense_units, 1),
            nn.Tanh()
        )
        
    def forward(self, x: 'torch.Tensor') -> 'torch.Tensor':
        """
        Forward pass.
        
        Args:
            x: Input tensor of shape [batch, sequence_length, feature_dim]
            
        Returns:
            Output tensor of shape [batch, 1] with values in [-1, 1]
        """
        # Feature mixing (apply to each timestep)
        # x: [batch, seq, features] -> [batch, seq, 128]
        x = self.feature_mix(x)
        
        # Batch normalization
        x = self.bn(x)
        
        # First LSTM
        x, _ = self.lstm1(x)
        x = self.dropout1(x)
        
        # Second LSTM
        x, _ = self.lstm2(x)
        x = self.dropout2(x)
        
        # Take only the last timestep output
        x = x[:, -1, :]
        
        # Output mapping
        return self.output(x)
    
    def count_parameters(self) -> int:
        """Count trainable parameters."""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)


def build_autopilot_model(config: Optional[ModelConfig] = None) -> 'AutopilotLSTM':
    """
    Build the end-to-end LSTM autopilot model.
    
    Args:
        config: Model configuration
        
    Returns:
        PyTorch model
    """
    if not HAS_TORCH:
        raise ImportError("PyTorch is required to build the model")
        
    return AutopilotLSTM(config)


def get_device() -> 'torch.device':
    """Get the best available device (MPS for Apple Silicon, CUDA, or CPU)."""
    if not HAS_TORCH:
        raise ImportError("PyTorch is required")
        
    if torch.backends.mps.is_available():
        return torch.device("mps")
    elif torch.cuda.is_available():
        return torch.device("cuda")
    else:
        return torch.device("cpu")


def convert_to_onnx(model: 'AutopilotLSTM', output_path: str,
                    config: Optional[ModelConfig] = None) -> str:
    """
    Convert PyTorch model to ONNX format for edge deployment.
    
    Args:
        model: Trained PyTorch model
        output_path: Path for .onnx file
        config: Model configuration (uses model.config if not provided)
        
    Returns:
        Path to saved model
    """
    if not HAS_TORCH:
        raise ImportError("PyTorch is required")
    
    config = config or model.config
    
    # Move model to CPU and set to eval mode
    model = model.cpu()
    model.eval()
    
    # Create dummy input
    dummy_input = torch.randn(1, config.sequence_length, config.feature_dim)
    
    # Export to ONNX using legacy exporter for LSTM compatibility
    # The new dynamo-based exporter has issues with LSTM + ONNX Runtime
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        },
        opset_version=14,
        do_constant_folding=True,
        dynamo=False,  # Use legacy exporter for better ONNX Runtime compatibility
    )
    
    size_kb = os.path.getsize(output_path) / 1024
    logger.info(f"Saved ONNX model to {output_path}")
    logger.info(f"Model size: {size_kb:.1f} KB")
    
    return output_path


def save_model(model: 'AutopilotLSTM', path: str):
    """
    Save PyTorch model checkpoint.
    
    Args:
        model: Model to save
        path: Path for .pt file
    """
    if not HAS_TORCH:
        raise ImportError("PyTorch is required")
        
    torch.save({
        'model_state_dict': model.state_dict(),
        'config': model.config,
    }, path)
    logger.info(f"Saved model checkpoint to {path}")


def load_model(path: str, device: Optional['torch.device'] = None) -> 'AutopilotLSTM':
    """
    Load PyTorch model from checkpoint.
    
    Args:
        path: Path to .pt file
        device: Device to load model to
        
    Returns:
        Loaded model
    """
    if not HAS_TORCH:
        raise ImportError("PyTorch is required")
        
    device = device or get_device()
    checkpoint = torch.load(path, map_location=device)
    
    config = checkpoint.get('config', ModelConfig())
    model = AutopilotLSTM(config)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()
    
    logger.info(f"Loaded model from {path}")
    return model


class AutopilotInference:
    """
    Real-time inference engine for autopilot model.
    
    Supports multiple model formats:
    - ONNX (.onnx) - Preferred for edge deployment (Raspberry Pi)
    - PyTorch (.pt) - For development/debugging
    
    ONNX is recommended for Pi deployment for maximum compatibility
    and performance.
    """
    
    def __init__(self, model_path: str, config: Optional[ModelConfig] = None):
        """
        Initialize inference engine.
        
        Args:
            model_path: Path to model file (.onnx or .pt)
            config: Model configuration
        """
        self.config = config or ModelConfig()
        self._onnx_session = None
        self._torch_model = None
        self._model_type = None  # 'onnx' or 'torch'
        self._onnx_input_name = None
        
        self._load_model(model_path)
        
    def _load_model(self, model_path: str):
        """Load model from file (supports .onnx and .pt)."""
        ext = os.path.splitext(model_path)[1].lower()
        
        try:
            if ext == '.onnx':
                self._load_onnx_model(model_path)
            elif ext == '.pt':
                self._load_torch_model(model_path)
            else:
                # Try ONNX first, then PyTorch
                loaded = False
                for loader in [self._load_onnx_model, self._load_torch_model]:
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
    
    def _load_torch_model(self, model_path: str):
        """Load PyTorch model."""
        if not HAS_TORCH:
            raise ImportError("PyTorch is required to load .pt models")
        
        self._torch_model = load_model(model_path)
        self._torch_model.eval()
        self._model_type = 'torch'
        
        logger.info(f"Loaded PyTorch model from {model_path}")
            
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
        elif self._model_type == 'torch':
            # PyTorch inference
            if self._torch_model is None:
                raise RuntimeError("Model not loaded")
            with torch.no_grad():
                tensor = torch.from_numpy(sequence)
                output = self._torch_model(tensor).numpy()
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
        self._onnx_session = None
        self._torch_model = None
        self._model_type = 'mock'
        
    def _load_model(self, model_path: str):
        pass
        
    def predict(self, sequence: np.ndarray) -> float:
        """
        Simple proportional response based on heading error.
        
        ARCHITECTURE: Feature[0] is ALWAYS heading error (HelmController convention).
        error = computed_heading - heading (target - current)
        All steering modes use the same sign convention:
        - Positive error = target to starboard
        - Positive rudder turns boat starboard
        - So: rudder = +error * gain
        
        This works for all modes because wind targets are converted to heading
        targets before computing the error.
        """
        if sequence.ndim == 2:
            # Get most recent heading error (feature 0) - already normalized
            heading_error = sequence[-1, 0]
            
            # Simple P control: positive error → positive rudder
            # (turn toward target heading, which is to starboard)
            command = heading_error * 0.5
            
            return float(np.clip(command, -1.0, 1.0))
            
        return 0.0
