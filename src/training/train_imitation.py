"""
Imitation Learning Trainer
==========================

Trains the autopilot model by imitating human helming behavior
from logged data.
"""

import os
import json
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any
import numpy as np
import logging

from .data_loader import TrainingDataLoader, DataConfig
from ..ml.autopilot_model import (
    build_autopilot_model, 
    compile_model, 
    convert_to_tflite,
    ModelConfig
)

logger = logging.getLogger(__name__)

# TensorFlow imports
try:
    import tensorflow as tf
    HAS_TF = True
except ImportError:
    tf = None
    HAS_TF = False


@dataclass
class TrainingConfig:
    """Training configuration."""
    # Model
    sequence_length: int = 20
    feature_dim: int = 25
    
    # Training
    epochs: int = 100
    batch_size: int = 64
    learning_rate: float = 0.001
    
    # Early stopping
    patience: int = 10
    min_delta: float = 0.0001
    
    # Output
    model_dir: str = "models"
    model_name: str = "autopilot"
    
    # Data augmentation
    add_noise: bool = True
    noise_std: float = 0.02


class ImitationTrainer:
    """
    Trains autopilot model using imitation learning.
    
    Learns to predict human rudder commands from sensor observations.
    """
    
    def __init__(self, 
                 training_config: Optional[TrainingConfig] = None,
                 model_config: Optional[ModelConfig] = None,
                 data_config: Optional[DataConfig] = None):
        
        if not HAS_TF:
            raise ImportError("TensorFlow required for training. Install with: pip install tensorflow")
            
        self.training_config = training_config or TrainingConfig()
        self.model_config = model_config or ModelConfig(
            sequence_length=self.training_config.sequence_length,
            feature_dim=self.training_config.feature_dim
        )
        self.data_config = data_config or DataConfig(
            sequence_length=self.training_config.sequence_length,
            feature_dim=self.training_config.feature_dim
        )
        
        self._model = None
        self._history = None
        self._data_loader = TrainingDataLoader(self.data_config)
        
    def load_data(self, data_dir: str) -> tuple:
        """Load training data from directory."""
        X, y = self._data_loader.load_directory(data_dir)
        
        if len(X) == 0:
            raise ValueError(f"No training data found in {data_dir}")
            
        return self._data_loader.split_data(X, y)
        
    def build_model(self):
        """Build and compile the model."""
        self._model = build_autopilot_model(self.model_config)
        compile_model(self._model, self.training_config.learning_rate)
        
        logger.info("Model built:")
        self._model.summary(print_fn=logger.info)
        
        return self._model
        
    def train(self, 
              X_train: np.ndarray, 
              y_train: np.ndarray,
              X_val: np.ndarray, 
              y_val: np.ndarray) -> Dict[str, Any]:
        """
        Train the model.
        
        Args:
            X_train: Training sequences [n, seq_len, features]
            y_train: Training labels [n, 1]
            X_val: Validation sequences
            y_val: Validation labels
            
        Returns:
            Training history dict
        """
        if self._model is None:
            self.build_model()
            
        # Data augmentation
        if self.training_config.add_noise:
            X_train = self._augment_data(X_train)
            
        # Callbacks
        callbacks = [
            tf.keras.callbacks.EarlyStopping(
                monitor='val_loss',
                patience=self.training_config.patience,
                min_delta=self.training_config.min_delta,
                restore_best_weights=True
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss',
                factor=0.5,
                patience=5,
                min_lr=1e-6
            ),
            tf.keras.callbacks.ModelCheckpoint(
                filepath=os.path.join(
                    self.training_config.model_dir,
                    f"{self.training_config.model_name}_best.h5"
                ),
                monitor='val_loss',
                save_best_only=True
            )
        ]
        
        # Create model directory
        os.makedirs(self.training_config.model_dir, exist_ok=True)
        
        # Train
        logger.info(f"Training on {len(X_train)} samples, validating on {len(X_val)}")
        
        self._history = self._model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            epochs=self.training_config.epochs,
            batch_size=self.training_config.batch_size,
            callbacks=callbacks,
            verbose=1
        )
        
        return self._history.history
        
    def _augment_data(self, X: np.ndarray) -> np.ndarray:
        """Apply data augmentation."""
        # Add Gaussian noise
        noise = np.random.normal(
            0, self.training_config.noise_std, X.shape
        )
        X_augmented = X + noise
        
        # Clip to valid range
        return np.clip(X_augmented, -1.0, 1.0)
        
    def evaluate(self, X_test: np.ndarray, y_test: np.ndarray) -> Dict[str, float]:
        """Evaluate model on test data."""
        if self._model is None:
            raise RuntimeError("Model not trained")
            
        results = self._model.evaluate(X_test, y_test, verbose=0)
        
        metrics = {
            'loss': results[0],
            'mae': results[1]
        }
        
        # Compute additional metrics
        predictions = self._model.predict(X_test, verbose=0)
        errors = predictions.flatten() - y_test.flatten()
        
        metrics['rmse'] = float(np.sqrt(np.mean(errors ** 2)))
        metrics['max_error'] = float(np.max(np.abs(errors)))
        metrics['std_error'] = float(np.std(errors))
        
        # Convert back to degrees for interpretability
        metrics['mae_degrees'] = metrics['mae'] * 30.0
        metrics['rmse_degrees'] = metrics['rmse'] * 30.0
        metrics['max_error_degrees'] = metrics['max_error'] * 30.0
        
        logger.info(f"Evaluation: MAE={metrics['mae_degrees']:.2f}°, "
                   f"RMSE={metrics['rmse_degrees']:.2f}°")
        
        return metrics
        
    def save_model(self, include_tflite: bool = True) -> Dict[str, str]:
        """
        Save trained model.
        
        Returns:
            Dict with paths to saved files
        """
        if self._model is None:
            raise RuntimeError("Model not trained")
            
        paths = {}
        model_dir = self.training_config.model_dir
        model_name = self.training_config.model_name
        
        # Save Keras model
        h5_path = os.path.join(model_dir, f"{model_name}.h5")
        self._model.save(h5_path)
        paths['keras'] = h5_path
        logger.info(f"Saved Keras model to {h5_path}")
        
        # Save TFLite model
        if include_tflite:
            tflite_path = os.path.join(model_dir, f"{model_name}.tflite")
            convert_to_tflite(self._model, tflite_path, quantize=True)
            paths['tflite'] = tflite_path
            
        # Save training config
        config_path = os.path.join(model_dir, f"{model_name}_config.json")
        config = {
            'training': asdict(self.training_config),
            'model': asdict(self.model_config),
            'data': asdict(self.data_config)
        }
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        paths['config'] = config_path
        
        # Save training history
        if self._history:
            history_path = os.path.join(model_dir, f"{model_name}_history.json")
            with open(history_path, 'w') as f:
                # Convert numpy types to Python types
                history = {k: [float(v) for v in vals] 
                          for k, vals in self._history.history.items()}
                json.dump(history, f, indent=2)
            paths['history'] = history_path
            
        return paths
        
    def load_model(self, model_path: str):
        """Load a previously trained model."""
        self._model = tf.keras.models.load_model(model_path)
        logger.info(f"Loaded model from {model_path}")


def train_from_logs(data_dir: str, 
                    output_dir: str = "models",
                    epochs: int = 100) -> Dict[str, str]:
    """
    Convenience function to train model from logged data.
    
    Args:
        data_dir: Directory containing log files
        output_dir: Directory for output models
        epochs: Number of training epochs
        
    Returns:
        Dict with paths to saved model files
    """
    config = TrainingConfig(
        epochs=epochs,
        model_dir=output_dir
    )
    
    trainer = ImitationTrainer(training_config=config)
    
    # Load and split data
    X_train, y_train, X_val, y_val = trainer.load_data(data_dir)
    
    # Build and train
    trainer.build_model()
    trainer.train(X_train, y_train, X_val, y_val)
    
    # Evaluate
    trainer.evaluate(X_val, y_val)
    
    # Save
    return trainer.save_model()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Train autopilot model")
    parser.add_argument("data_dir", help="Directory containing training logs")
    parser.add_argument("--output", "-o", default="models", help="Output directory")
    parser.add_argument("--epochs", "-e", type=int, default=100, help="Training epochs")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    paths = train_from_logs(args.data_dir, args.output, args.epochs)
    
    print("\nTraining complete! Model files:")
    for name, path in paths.items():
        print(f"  {name}: {path}")
