"""
Unit tests for Autopilot Model module.

Tests model architecture shapes, mock inference, and input handling.
"""

import pytest
import numpy as np

from src.ml.autopilot_model import (
    ModelConfig, build_autopilot_model, compile_model,
    MockAutopilotInference, HAS_TF
)


# Skip TensorFlow tests if not available
pytestmark = pytest.mark.skipif(not HAS_TF, reason="TensorFlow not installed")


class TestModelConfig:
    """Tests for ModelConfig defaults."""
    
    def test_default_config_values(self):
        """Default configuration should match expected values."""
        config = ModelConfig()
        
        assert config.sequence_length == 20
        assert config.feature_dim == 25
        assert config.lstm_units_1 == 64
        assert config.lstm_units_2 == 32
        assert config.dense_units == 16
        assert config.dropout_rate == 0.2
    
    def test_custom_config(self):
        """Custom configuration should override defaults."""
        config = ModelConfig(
            sequence_length=30,
            feature_dim=32,
            lstm_units_1=128
        )
        
        assert config.sequence_length == 30
        assert config.feature_dim == 32
        assert config.lstm_units_1 == 128
        # Others remain default
        assert config.lstm_units_2 == 32


@pytest.mark.skipif(not HAS_TF, reason="TensorFlow not installed")
class TestBuildModel:
    """Tests for build_autopilot_model function."""
    
    def test_model_input_shape(self):
        """Model input shape should be (None, 20, 25)."""
        model = build_autopilot_model()
        
        # Input shape is (batch, sequence_length, feature_dim)
        assert model.input_shape == (None, 20, 25)
    
    def test_model_output_shape(self):
        """Model output shape should be (None, 1)."""
        model = build_autopilot_model()
        
        assert model.output_shape == (None, 1)
    
    def test_model_custom_config_shapes(self):
        """Model should respect custom config for shapes."""
        config = ModelConfig(sequence_length=30, feature_dim=32)
        model = build_autopilot_model(config)
        
        assert model.input_shape == (None, 30, 32)
        assert model.output_shape == (None, 1)
    
    def test_model_has_expected_layers(self):
        """Model should have expected layer types."""
        model = build_autopilot_model()
        layer_types = [type(layer).__name__ for layer in model.layers]
        
        assert 'InputLayer' in layer_types
        assert 'TimeDistributed' in layer_types
        assert 'LSTM' in layer_types
        assert 'Dense' in layer_types
        assert 'Dropout' in layer_types
    
    def test_model_name(self):
        """Model should have correct name."""
        model = build_autopilot_model()
        assert model.name == 'autopilot_lstm'
    
    def test_model_trainable_params(self):
        """Model should have reasonable number of trainable parameters."""
        model = build_autopilot_model()
        
        # Should have parameters (not empty)
        assert model.count_params() > 0
        # Should be under 100k for efficient inference
        assert model.count_params() < 100000


@pytest.mark.skipif(not HAS_TF, reason="TensorFlow not installed")
class TestCompileModel:
    """Tests for compile_model function."""
    
    def test_compile_model_success(self):
        """compile_model should set optimizer and loss."""
        model = build_autopilot_model()
        compiled = compile_model(model)
        
        assert compiled.optimizer is not None
        assert compiled.loss is not None
    
    def test_compile_model_custom_learning_rate(self):
        """compile_model should accept custom learning rate."""
        model = build_autopilot_model()
        compiled = compile_model(model, learning_rate=0.0001)
        
        # Check learning rate is set (TF2 API)
        lr = float(compiled.optimizer.learning_rate)
        assert lr == pytest.approx(0.0001)


@pytest.mark.skipif(not HAS_TF, reason="TensorFlow not installed")
class TestModelInference:
    """Tests for model forward pass."""
    
    def test_model_forward_pass(self):
        """Model should produce output from valid input."""
        model = build_autopilot_model()
        
        # Create random input batch
        batch_size = 4
        input_data = np.random.randn(batch_size, 20, 25).astype(np.float32)
        
        output = model.predict(input_data, verbose=0)
        
        assert output.shape == (batch_size, 1)
    
    def test_model_output_bounded(self):
        """Model output should be bounded by tanh to [-1, 1]."""
        model = build_autopilot_model()
        
        # Create input that might produce extreme output
        input_data = np.ones((1, 20, 25), dtype=np.float32) * 10.0
        
        output = model.predict(input_data, verbose=0)
        
        assert -1.0 <= output[0, 0] <= 1.0


class TestMockAutopilotInference:
    """Tests for MockAutopilotInference class."""
    
    def test_mock_inference_no_model_needed(self):
        """Mock inference should work without loading a model file."""
        mock = MockAutopilotInference()
        assert mock._interpreter == "mock"
    
    def test_mock_inference_2d_input(self):
        """Mock inference should handle 2D input (no batch dim)."""
        mock = MockAutopilotInference()
        
        # Create 2D input: [sequence_length, feature_dim]
        input_seq = np.random.randn(20, 25).astype(np.float32)
        
        output = mock.predict(input_seq)
        
        assert isinstance(output, float)
        assert -1.0 <= output <= 1.0
    
    def test_mock_inference_3d_input(self):
        """Mock inference should handle 3D input (with batch dim)."""
        mock = MockAutopilotInference()
        
        # Create 3D input: [batch, sequence_length, feature_dim]
        input_seq = np.random.randn(1, 20, 25).astype(np.float32)
        
        output = mock.predict(input_seq)
        
        assert isinstance(output, float)
        assert -1.0 <= output <= 1.0
    
    def test_mock_inference_proportional_response(self):
        """Mock inference should produce proportional response to heading error."""
        mock = MockAutopilotInference()
        
        # Create input with positive heading error (feature 0)
        input_seq = np.zeros((20, 25), dtype=np.float32)
        input_seq[-1, 0] = 0.5  # Positive heading error
        
        output = mock.predict(input_seq)
        
        # Should respond with negative output (steer to correct)
        assert output < 0
    
    def test_mock_inference_zero_error_zero_output(self):
        """Mock inference should produce ~0 output for zero error."""
        mock = MockAutopilotInference()
        
        # Create input with zero heading error
        input_seq = np.zeros((20, 25), dtype=np.float32)
        
        output = mock.predict(input_seq)
        
        assert output == pytest.approx(0.0)
    
    def test_mock_inference_output_clipped(self):
        """Mock inference output should be clipped to [-1, 1]."""
        mock = MockAutopilotInference()
        
        # Create input with extreme heading error
        input_seq = np.zeros((20, 25), dtype=np.float32)
        input_seq[-1, 0] = 10.0  # Very large (unnormalized) error
        
        output = mock.predict(input_seq)
        
        # Should be clipped
        assert -1.0 <= output <= 1.0
    
    def test_mock_config_defaults(self):
        """Mock inference should use default config."""
        mock = MockAutopilotInference()
        
        assert mock.config.sequence_length == 20
        assert mock.config.feature_dim == 25
    
    def test_mock_custom_config(self):
        """Mock inference should accept custom config."""
        config = ModelConfig(sequence_length=30, feature_dim=32)
        mock = MockAutopilotInference(config=config)
        
        assert mock.config.sequence_length == 30
        assert mock.config.feature_dim == 32
