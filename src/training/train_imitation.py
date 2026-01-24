"""
Imitation Learning Trainer
==========================

Trains the autopilot model by imitating human helming behavior
from logged data.
"""

import os
import json
import time
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


class WarmupSchedule(tf.keras.callbacks.Callback):
    """
    Learning rate warmup schedule.
    
    Linearly increases learning rate from a small value to the target
    over the first few epochs. This helps stabilize early training.
    """
    
    def __init__(self, warmup_epochs: int = 3, target_lr: float = 1e-4):
        super().__init__()
        self.warmup_epochs = warmup_epochs
        self.target_lr = target_lr
        self.initial_lr = target_lr / 10  # Start at 10% of target
        
    def on_epoch_begin(self, epoch, logs=None):
        if epoch < self.warmup_epochs:
            # Linear warmup
            lr = self.initial_lr + (self.target_lr - self.initial_lr) * (epoch / self.warmup_epochs)
            self.model.optimizer.learning_rate.assign(lr)
            print(f"Warmup: Setting learning rate to {lr:.2e}")


class ProgressCallback(tf.keras.callbacks.Callback):
    """
    Custom callback for detailed training progress monitoring.
    
    Shows convergence information including loss trends, improvement rates,
    and estimated time remaining.
    """
    
    def __init__(self, max_time_seconds: Optional[float] = None):
        super().__init__()
        self.max_time_seconds = max_time_seconds
        self.start_time = None
        self.epoch_times = []
        self.best_val_loss = float('inf')
        self.epochs_without_improvement = 0
        self.loss_history = []
        self.val_loss_history = []
        
    def on_train_begin(self, logs=None):
        self.start_time = time.time()
        print("\n" + "=" * 70)
        print("TRAINING STARTED")
        print("=" * 70)
        if self.max_time_seconds:
            print(f"Time limit: {self.max_time_seconds:.0f} seconds")
        print()
        
    def on_epoch_begin(self, epoch, logs=None):
        self.epoch_start_time = time.time()
        
    def on_epoch_end(self, epoch, logs=None):
        epoch_time = time.time() - self.epoch_start_time
        self.epoch_times.append(epoch_time)
        elapsed = time.time() - self.start_time
        
        # Track losses
        loss = logs.get('loss', 0)
        val_loss = logs.get('val_loss', 0)
        mae = logs.get('mae', 0)
        val_mae = logs.get('val_mae', 0)
        lr = float(tf.keras.backend.get_value(self.model.optimizer.learning_rate))
        
        self.loss_history.append(loss)
        self.val_loss_history.append(val_loss)
        
        # Check for improvement
        improved = val_loss < self.best_val_loss
        if improved:
            improvement = self.best_val_loss - val_loss
            self.best_val_loss = val_loss
            self.epochs_without_improvement = 0
            improvement_str = f"  ↓ improved by {improvement:.6f}"
        else:
            self.epochs_without_improvement += 1
            improvement_str = f"  (no improvement for {self.epochs_without_improvement} epochs)"
        
        # Calculate trend (average loss change over last 5 epochs)
        trend_str = ""
        if len(self.val_loss_history) >= 5:
            recent = self.val_loss_history[-5:]
            trend = (recent[-1] - recent[0]) / 4  # Average change per epoch
            if trend < 0:
                trend_str = f"  trend: ↓{abs(trend):.6f}/epoch"
            else:
                trend_str = f"  trend: ↑{trend:.6f}/epoch (may be overfitting)"
        
        # Estimate remaining time
        avg_epoch_time = np.mean(self.epoch_times)
        total_epochs = self.params.get('epochs', 100)
        remaining_epochs = total_epochs - epoch - 1
        eta = avg_epoch_time * remaining_epochs
        
        # Convert MAE to degrees for readability
        mae_deg = mae * 30.0
        val_mae_deg = val_mae * 30.0
        
        # Print progress
        print(f"\nEpoch {epoch + 1}/{total_epochs} ({epoch_time:.1f}s, elapsed: {elapsed:.0f}s, ETA: {eta:.0f}s)")
        print(f"  Loss:     {loss:.6f}  |  Val Loss: {val_loss:.6f}{improvement_str}")
        print(f"  MAE:      {mae:.6f} ({mae_deg:.2f}°)  |  Val MAE:  {val_mae:.6f} ({val_mae_deg:.2f}°)")
        print(f"  LR: {lr:.2e}  |  Best Val Loss: {self.best_val_loss:.6f}{trend_str}")
        
        # Check time limit
        if self.max_time_seconds and elapsed >= self.max_time_seconds:
            print(f"\n*** Time limit of {self.max_time_seconds:.0f}s reached. Stopping training. ***")
            self.model.stop_training = True
            
    def on_train_end(self, logs=None):
        elapsed = time.time() - self.start_time
        print("\n" + "=" * 70)
        print("TRAINING COMPLETE")
        print("=" * 70)
        print(f"Total time: {elapsed:.1f}s ({elapsed/60:.1f} minutes)")
        print(f"Final validation loss: {self.val_loss_history[-1]:.6f}")
        print(f"Best validation loss: {self.best_val_loss:.6f}")
        
        if len(self.val_loss_history) > 1:
            total_improvement = self.val_loss_history[0] - self.best_val_loss
            print(f"Total improvement: {total_improvement:.6f} ({total_improvement/self.val_loss_history[0]*100:.1f}%)")
        print()


@dataclass
class TrainingConfig:
    """Training configuration."""
    # Model
    sequence_length: int = 20
    feature_dim: int = 25
    
    # Training
    epochs: int = 100
    batch_size: int = 64
    learning_rate: float = 1e-4  # Lower LR for stability
    
    # Early stopping
    patience: int = 8  # Reduced from 10 - stop earlier if no improvement
    min_delta: float = 0.0001
    
    # Learning rate warmup
    warmup_epochs: int = 3
    
    # Time limit (None = no limit)
    max_time_seconds: Optional[float] = None
    
    # Output
    model_dir: str = "models"
    model_name: str = "autopilot"
    
    # Data augmentation
    add_noise: bool = True
    noise_std: float = 0.02
    
    # Resume from checkpoint
    resume_from_checkpoint: bool = True  # Auto-resume if checkpoint exists


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
    
    def get_checkpoint_path(self) -> str:
        """Get the path to the best checkpoint file."""
        return os.path.join(
            self.training_config.model_dir,
            f"{self.training_config.model_name}_best.keras"
        )
    
    def try_load_checkpoint(self) -> bool:
        """
        Try to load from existing checkpoint.
        
        Returns:
            True if checkpoint was loaded, False otherwise
        """
        checkpoint_path = self.get_checkpoint_path()
        
        if os.path.exists(checkpoint_path):
            try:
                # Import custom loss for loading
                from ..ml.autopilot_model import autopilot_loss
                
                # Load the saved model with custom objects
                self._model = tf.keras.models.load_model(
                    checkpoint_path,
                    custom_objects={'autopilot_loss': autopilot_loss}
                )
                # Re-compile with current learning rate settings
                compile_model(self._model, self.training_config.learning_rate)
                
                print(f"\n{'='*70}")
                print(f"RESUMING FROM CHECKPOINT")
                print(f"{'='*70}")
                print(f"Loaded model from: {checkpoint_path}")
                print(f"Model will continue training with current settings.")
                print()
                
                logger.info(f"Resumed from checkpoint: {checkpoint_path}")
                return True
                
            except Exception as e:
                logger.warning(f"Failed to load checkpoint {checkpoint_path}: {e}")
                print(f"Warning: Could not load checkpoint, starting fresh. Error: {e}")
                return False
        else:
            logger.info(f"No checkpoint found at {checkpoint_path}, building new model")
            return False
    
    def build_or_load_model(self) -> bool:
        """
        Build a new model or load from checkpoint if available.
        
        Returns:
            True if loaded from checkpoint, False if built fresh
        """
        if self.training_config.resume_from_checkpoint:
            if self.try_load_checkpoint():
                return True
        
        self.build_model()
        return False
        
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
            resumed = self.build_or_load_model()
            # Skip warmup if resuming from checkpoint
            if resumed:
                self.training_config.warmup_epochs = 0
            
        # Data augmentation
        if self.training_config.add_noise:
            X_train = self._augment_data(X_train)
            
        # Callbacks
        callbacks = [
            WarmupSchedule(
                warmup_epochs=self.training_config.warmup_epochs,
                target_lr=self.training_config.learning_rate
            ),
            ProgressCallback(max_time_seconds=self.training_config.max_time_seconds),
            tf.keras.callbacks.EarlyStopping(
                monitor='val_loss',
                patience=self.training_config.patience,
                min_delta=self.training_config.min_delta,
                restore_best_weights=True
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss',
                factor=0.5,
                patience=3,  # Reduced from 5 - respond faster to plateaus
                min_lr=1e-6,
                verbose=1
            ),
            tf.keras.callbacks.ModelCheckpoint(
                filepath=os.path.join(
                    self.training_config.model_dir,
                    f"{self.training_config.model_name}_best.keras"
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
            verbose=0  # Using custom ProgressCallback for output
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
        
        # Save Keras model (native format)
        keras_path = os.path.join(model_dir, f"{model_name}.keras")
        self._model.save(keras_path)
        paths['keras'] = keras_path
        logger.info(f"Saved Keras model to {keras_path}")
        
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
                    epochs: int = 100,
                    max_time_seconds: Optional[float] = None,
                    resume: bool = True) -> Dict[str, str]:
    """
    Convenience function to train model from logged data.
    
    Args:
        data_dir: Directory containing log files
        output_dir: Directory for output models
        epochs: Number of training epochs
        max_time_seconds: Maximum training time in seconds (None = no limit)
        resume: Whether to resume from checkpoint if available
        
    Returns:
        Dict with paths to saved model files
    """
    config = TrainingConfig(
        epochs=epochs,
        model_dir=output_dir,
        max_time_seconds=max_time_seconds,
        resume_from_checkpoint=resume
    )
    
    trainer = ImitationTrainer(training_config=config)
    
    # Load and split data
    print(f"Loading training data from: {data_dir}")
    X_train, y_train, X_val, y_val = trainer.load_data(data_dir)
    print(f"Loaded {len(X_train)} training samples, {len(X_val)} validation samples")
    
    # Train (automatically loads checkpoint if available)
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
    parser.add_argument("--max-time", "-t", type=float, default=None,
                       help="Maximum training time in seconds (for quick testing)")
    parser.add_argument("--fresh", "-f", action="store_true",
                       help="Start fresh, ignoring any existing checkpoint")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    paths = train_from_logs(
        args.data_dir, 
        args.output, 
        args.epochs, 
        args.max_time,
        resume=not args.fresh
    )
    
    print("\nModel files saved:")
    for name, path in paths.items():
        print(f"  {name}: {path}")
