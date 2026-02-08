"""
Unit tests for Autopilot Model module.

Tests model architecture shapes, mock inference, and input handling.
"""

import pytest
import numpy as np

from src.ml.autopilot_model import (
    ModelConfig, build_autopilot_model, AutopilotLSTM,
    MockAutopilotInference, HAS_TORCH
)

# PyTorch imports for testing
if HAS_TORCH:
    import torch


# Skip PyTorch tests if not available
pytestmark = pytest.mark.skipif(not HAS_TORCH, reason="PyTorch not installed")


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


@pytest.mark.skipif(not HAS_TORCH, reason="PyTorch not installed")
class TestBuildModel:
    """Tests for build_autopilot_model function."""
    
    def test_model_is_pytorch_module(self):
        """Model should be a PyTorch nn.Module."""
        model = build_autopilot_model()
        
        assert isinstance(model, torch.nn.Module)
        assert isinstance(model, AutopilotLSTM)
    
    def test_model_forward_pass_shape(self):
        """Model should produce correct output shape."""
        model = build_autopilot_model()
        model.eval()
        
        # Create random input batch
        batch_size = 4
        input_data = torch.randn(batch_size, 20, 25)
        
        with torch.no_grad():
            output = model(input_data)
        
        assert output.shape == (batch_size, 1)
    
    def test_model_custom_config_shapes(self):
        """Model should respect custom config for shapes."""
        config = ModelConfig(sequence_length=30, feature_dim=32)
        model = build_autopilot_model(config)
        model.eval()
        
        # Create input with custom dimensions
        input_data = torch.randn(2, 30, 32)
        
        with torch.no_grad():
            output = model(input_data)
        
        assert output.shape == (2, 1)
    
    def test_model_has_expected_modules(self):
        """Model should have expected module types."""
        model = build_autopilot_model()
        module_names = [name for name, _ in model.named_modules() if name]
        
        # Check key module names exist
        assert 'feature_mix' in module_names
        assert 'bn' in module_names
        assert 'lstm1' in module_names
        assert 'lstm2' in module_names
        assert 'output' in module_names
        assert 'dropout1' in module_names
        assert 'dropout2' in module_names
    
    def test_model_trainable_params(self):
        """Model should have reasonable number of trainable parameters."""
        model = build_autopilot_model()
        
        num_params = model.count_parameters()
        
        # Should have parameters (not empty)
        assert num_params > 0
        # Should be under 100k for efficient inference
        assert num_params < 100000
    
    def test_model_count_params_method(self):
        """Model should have count_parameters method."""
        model = build_autopilot_model()
        
        # count_parameters should match sum of parameters
        expected = sum(p.numel() for p in model.parameters() if p.requires_grad)
        assert model.count_parameters() == expected


@pytest.mark.skipif(not HAS_TORCH, reason="PyTorch not installed")
class TestModelInference:
    """Tests for model forward pass."""
    
    def test_model_forward_pass(self):
        """Model should produce output from valid input."""
        model = build_autopilot_model()
        model.eval()
        
        # Create random input batch
        batch_size = 4
        input_data = torch.randn(batch_size, 20, 25)
        
        with torch.no_grad():
            output = model(input_data)
        
        assert output.shape == (batch_size, 1)
    
    def test_model_output_bounded(self):
        """Model output should be bounded by tanh to [-1, 1]."""
        model = build_autopilot_model()
        model.eval()
        
        # Create input that might produce extreme output
        input_data = torch.ones(1, 20, 25) * 10.0
        
        with torch.no_grad():
            output = model(input_data)
        
        assert -1.0 <= output[0, 0].item() <= 1.0
    
    def test_model_training_mode(self):
        """Model should behave differently in train vs eval mode."""
        model = build_autopilot_model()
        
        input_data = torch.randn(4, 20, 25)
        
        # Training mode (dropout active)
        model.train()
        with torch.no_grad():
            output_train = model(input_data)
        
        # Eval mode (dropout inactive)
        model.eval()
        with torch.no_grad():
            output_eval = model(input_data)
        
        # Both should produce valid output
        assert output_train.shape == (4, 1)
        assert output_eval.shape == (4, 1)


class TestMockAutopilotInference:
    """Tests for MockAutopilotInference class."""
    
    def test_mock_inference_no_model_needed(self):
        """Mock inference should work without loading a model file."""
        mock = MockAutopilotInference()
        assert mock._model_type == "mock"
    
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
        """Mock inference should produce proportional response to heading error.
        
        HelmController convention: error = target - heading
        Positive error = target to starboard → positive rudder
        """
        mock = MockAutopilotInference()
        
        # Create input with positive heading error (feature 0)
        # Positive error means target is to starboard (clockwise from heading)
        input_seq = np.zeros((20, 25), dtype=np.float32)
        input_seq[-1, 0] = 0.5  # Positive heading error (target to starboard)
        
        output = mock.predict(input_seq)
        
        # Should respond with positive output (turn starboard toward target)
        assert output > 0
    
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


@pytest.mark.skipif(not HAS_TORCH, reason="PyTorch not installed")
class TestONNXExport:
    """Tests for ONNX model export."""
    
    def test_onnx_export(self, tmp_path):
        """Model should export to ONNX format."""
        from src.ml.autopilot_model import convert_to_onnx
        
        model = build_autopilot_model()
        model.eval()
        
        onnx_path = str(tmp_path / "test_model.onnx")
        result_path = convert_to_onnx(model, onnx_path)
        
        assert result_path == onnx_path
        assert (tmp_path / "test_model.onnx").exists()
    
    def test_onnx_inference(self, tmp_path):
        """Exported ONNX model should produce valid inference."""
        from src.ml.autopilot_model import convert_to_onnx, AutopilotInference
        
        model = build_autopilot_model()
        model.eval()
        
        onnx_path = str(tmp_path / "test_model.onnx")
        convert_to_onnx(model, onnx_path)
        
        # Load and run inference
        inference = AutopilotInference(onnx_path)
        input_seq = np.random.randn(20, 25).astype(np.float32)
        
        output = inference.predict(input_seq)
        
        assert isinstance(output, float)
        assert -1.0 <= output <= 1.0
