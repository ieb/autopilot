"""
Imitation Learning Trainer
==========================

Trains the autopilot model by imitating human helming behavior
from logged data.

Uses PyTorch for training with MPS (Apple Silicon) or CUDA acceleration.
"""

import os
import json
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List
import numpy as np
import logging

from .data_loader import TrainingDataLoader, DataConfig
from ..ml.autopilot_model import (
    AutopilotLSTM,
    build_autopilot_model,
    convert_to_onnx,
    save_model as save_pytorch_model,
    load_model as load_pytorch_model,
    get_device,
    ModelConfig,
    HAS_TORCH
)

logger = logging.getLogger(__name__)

# PyTorch imports
if HAS_TORCH:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset

# MLflow imports (optional)
try:
    import mlflow
    import mlflow.pytorch
    HAS_MLFLOW = True
except ImportError:
    mlflow = None
    HAS_MLFLOW = False


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
    
    # MLflow tracking
    use_mlflow: bool = False
    mlflow_experiment_name: str = "autopilot"
    mlflow_run_name: Optional[str] = None


class TrainingHistory:
    """Tracks training metrics history."""
    
    def __init__(self):
        self.loss: List[float] = []
        self.val_loss: List[float] = []
        self.mae: List[float] = []
        self.val_mae: List[float] = []
        self.lr: List[float] = []
        
    def append(self, loss: float, val_loss: float, mae: float, val_mae: float, lr: float):
        self.loss.append(loss)
        self.val_loss.append(val_loss)
        self.mae.append(mae)
        self.val_mae.append(val_mae)
        self.lr.append(lr)
        
    def to_dict(self) -> Dict[str, List[float]]:
        return {
            'loss': self.loss,
            'val_loss': self.val_loss,
            'mae': self.mae,
            'val_mae': self.val_mae,
            'lr': self.lr
        }


class EarlyStopping:
    """Early stopping to prevent overfitting."""
    
    def __init__(self, patience: int = 8, min_delta: float = 0.0001):
        self.patience = patience
        self.min_delta = min_delta
        self.best_loss = float('inf')
        self.best_model_state = None
        self.counter = 0
        self.should_stop = False
        
    def __call__(self, val_loss: float, model: nn.Module) -> bool:
        """
        Check if training should stop.
        
        Args:
            val_loss: Current validation loss
            model: Model to checkpoint
            
        Returns:
            True if training should stop
        """
        if val_loss < self.best_loss - self.min_delta:
            self.best_loss = val_loss
            self.best_model_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            self.counter = 0
        else:
            self.counter += 1
            if self.counter >= self.patience:
                self.should_stop = True
                
        return self.should_stop
    
    def restore_best(self, model: nn.Module):
        """Restore the best model weights."""
        if self.best_model_state is not None:
            model.load_state_dict(self.best_model_state)


class ImitationTrainer:
    """
    Trains autopilot model using imitation learning.
    
    Learns to predict human rudder commands from sensor observations.
    """
    
    def __init__(self, 
                 training_config: Optional[TrainingConfig] = None,
                 model_config: Optional[ModelConfig] = None,
                 data_config: Optional[DataConfig] = None):
        
        if not HAS_TORCH:
            raise ImportError("PyTorch required for training. Install with: pip install torch")
            
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
        self._history = TrainingHistory()
        self._data_loader = TrainingDataLoader(self.data_config)
        self._device = get_device()
        
        logger.info(f"Using device: {self._device}")
        
    def load_data(self, data_dir: str) -> tuple:
        """Load training data from directory."""
        X, y = self._data_loader.load_directory(data_dir)
        
        if len(X) == 0:
            raise ValueError(f"No training data found in {data_dir}")
            
        return self._data_loader.split_data(X, y)
        
    def build_model(self) -> AutopilotLSTM:
        """Build the model."""
        self._model = build_autopilot_model(self.model_config)
        self._model.to(self._device)
        
        num_params = self._model.count_parameters()
        logger.info(f"Model built with {num_params:,} trainable parameters")
        print(f"\nModel architecture: {self._model}")
        print(f"Parameters: {num_params:,}")
        
        return self._model
    
    def get_checkpoint_path(self) -> str:
        """Get the path to the best checkpoint file."""
        return os.path.join(
            self.training_config.model_dir,
            f"{self.training_config.model_name}_best.pt"
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
                self._model = load_pytorch_model(checkpoint_path, self._device)
                
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
        
        # Create data loaders
        train_dataset = TensorDataset(
            torch.from_numpy(X_train).float(),
            torch.from_numpy(y_train).float()
        )
        val_dataset = TensorDataset(
            torch.from_numpy(X_val).float(),
            torch.from_numpy(y_val).float()
        )
        
        train_loader = DataLoader(
            train_dataset, 
            batch_size=self.training_config.batch_size,
            shuffle=True
        )
        val_loader = DataLoader(
            val_dataset,
            batch_size=self.training_config.batch_size,
            shuffle=False
        )
        
        # Setup training
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(
            self._model.parameters(),
            lr=self.training_config.learning_rate
        )
        
        # Learning rate schedulers
        # Warmup scheduler
        warmup_scheduler = torch.optim.lr_scheduler.LinearLR(
            optimizer,
            start_factor=0.1,
            end_factor=1.0,
            total_iters=self.training_config.warmup_epochs
        )
        # Reduce on plateau scheduler
        plateau_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            optimizer,
            mode='min',
            factor=0.5,
            patience=3,
            min_lr=1e-6,
            verbose=True
        )
        
        # Early stopping
        early_stopping = EarlyStopping(
            patience=self.training_config.patience,
            min_delta=self.training_config.min_delta
        )
        
        # Create model directory
        os.makedirs(self.training_config.model_dir, exist_ok=True)
        
        # Training state
        start_time = time.time()
        best_val_loss = float('inf')
        
        print("\n" + "=" * 70)
        print("TRAINING STARTED")
        print("=" * 70)
        if self.training_config.max_time_seconds:
            print(f"Time limit: {self.training_config.max_time_seconds:.0f} seconds")
        print(f"Device: {self._device}")
        print(f"Training samples: {len(X_train)}, Validation samples: {len(X_val)}")
        print()
        
        logger.info(f"Training on {len(X_train)} samples, validating on {len(X_val)}")
        
        for epoch in range(self.training_config.epochs):
            epoch_start_time = time.time()
            
            # Training phase
            self._model.train()
            train_loss = 0.0
            train_mae = 0.0
            num_batches = 0
            
            for X_batch, y_batch in train_loader:
                X_batch = X_batch.to(self._device)
                y_batch = y_batch.to(self._device)
                
                optimizer.zero_grad()
                output = self._model(X_batch)
                loss = criterion(output, y_batch)
                loss.backward()
                
                # Gradient clipping
                torch.nn.utils.clip_grad_norm_(self._model.parameters(), 1.0)
                
                optimizer.step()
                
                train_loss += loss.item()
                train_mae += torch.mean(torch.abs(output - y_batch)).item()
                num_batches += 1
            
            train_loss /= num_batches
            train_mae /= num_batches
            
            # Validation phase
            self._model.eval()
            val_loss = 0.0
            val_mae = 0.0
            num_val_batches = 0
            
            with torch.no_grad():
                for X_batch, y_batch in val_loader:
                    X_batch = X_batch.to(self._device)
                    y_batch = y_batch.to(self._device)
                    
                    output = self._model(X_batch)
                    loss = criterion(output, y_batch)
                    
                    val_loss += loss.item()
                    val_mae += torch.mean(torch.abs(output - y_batch)).item()
                    num_val_batches += 1
            
            val_loss /= num_val_batches
            val_mae /= num_val_batches
            
            # Get current learning rate
            current_lr = optimizer.param_groups[0]['lr']
            
            # Record history
            self._history.append(train_loss, val_loss, train_mae, val_mae, current_lr)
            
            # Update learning rate
            if epoch < self.training_config.warmup_epochs:
                warmup_scheduler.step()
                print(f"Warmup: Setting learning rate to {optimizer.param_groups[0]['lr']:.2e}")
            else:
                plateau_scheduler.step(val_loss)
            
            # Check for improvement and save checkpoint
            if val_loss < best_val_loss:
                improvement = best_val_loss - val_loss
                best_val_loss = val_loss
                improvement_str = f"  ↓ improved by {improvement:.6f}"
                
                # Save best checkpoint
                checkpoint_path = self.get_checkpoint_path()
                save_pytorch_model(self._model, checkpoint_path)
            else:
                improvement_str = f"  (no improvement for {early_stopping.counter + 1} epochs)"
            
            # Early stopping check
            if early_stopping(val_loss, self._model):
                print(f"\n*** Early stopping triggered after {epoch + 1} epochs ***")
                break
            
            # Calculate timing
            epoch_time = time.time() - epoch_start_time
            elapsed = time.time() - start_time
            avg_epoch_time = elapsed / (epoch + 1)
            remaining_epochs = self.training_config.epochs - epoch - 1
            eta = avg_epoch_time * remaining_epochs
            
            # Calculate trend
            trend_str = ""
            if len(self._history.val_loss) >= 5:
                recent = self._history.val_loss[-5:]
                trend = (recent[-1] - recent[0]) / 4
                if trend < 0:
                    trend_str = f"  trend: ↓{abs(trend):.6f}/epoch"
                else:
                    trend_str = f"  trend: ↑{trend:.6f}/epoch (may be overfitting)"
            
            # Convert MAE to degrees for readability
            mae_deg = train_mae * 30.0
            val_mae_deg = val_mae * 30.0
            
            # Print progress
            print(f"\nEpoch {epoch + 1}/{self.training_config.epochs} "
                  f"({epoch_time:.1f}s, elapsed: {elapsed:.0f}s, ETA: {eta:.0f}s)")
            print(f"  Loss:     {train_loss:.6f}  |  Val Loss: {val_loss:.6f}{improvement_str}")
            print(f"  MAE:      {train_mae:.6f} ({mae_deg:.2f}°)  |  "
                  f"Val MAE:  {val_mae:.6f} ({val_mae_deg:.2f}°)")
            print(f"  LR: {current_lr:.2e}  |  Best Val Loss: {best_val_loss:.6f}{trend_str}")
            
            # Check time limit
            if self.training_config.max_time_seconds and elapsed >= self.training_config.max_time_seconds:
                print(f"\n*** Time limit of {self.training_config.max_time_seconds:.0f}s reached. "
                      f"Stopping training. ***")
                break
        
        # Restore best model
        early_stopping.restore_best(self._model)
        
        # Print summary
        elapsed = time.time() - start_time
        print("\n" + "=" * 70)
        print("TRAINING COMPLETE")
        print("=" * 70)
        print(f"Total time: {elapsed:.1f}s ({elapsed/60:.1f} minutes)")
        print(f"Final validation loss: {self._history.val_loss[-1]:.6f}")
        print(f"Best validation loss: {best_val_loss:.6f}")
        
        if len(self._history.val_loss) > 1:
            total_improvement = self._history.val_loss[0] - best_val_loss
            print(f"Total improvement: {total_improvement:.6f} "
                  f"({total_improvement/self._history.val_loss[0]*100:.1f}%)")
        print()
        
        return self._history.to_dict()
        
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
        
        self._model.eval()
        
        # Convert to tensors
        X_tensor = torch.from_numpy(X_test).float().to(self._device)
        y_tensor = torch.from_numpy(y_test).float().to(self._device)
        
        with torch.no_grad():
            predictions = self._model(X_tensor)
            loss = nn.MSELoss()(predictions, y_tensor).item()
            mae = torch.mean(torch.abs(predictions - y_tensor)).item()
        
        # Compute additional metrics
        predictions_np = predictions.cpu().numpy()
        errors = predictions_np.flatten() - y_test.flatten()
        
        metrics = {
            'loss': loss,
            'mae': mae,
            'rmse': float(np.sqrt(np.mean(errors ** 2))),
            'max_error': float(np.max(np.abs(errors))),
            'std_error': float(np.std(errors)),
        }
        
        # Convert back to degrees for interpretability
        metrics['mae_degrees'] = metrics['mae'] * 30.0
        metrics['rmse_degrees'] = metrics['rmse'] * 30.0
        metrics['max_error_degrees'] = metrics['max_error'] * 30.0
        
        logger.info(f"Evaluation: MAE={metrics['mae_degrees']:.2f}°, "
                   f"RMSE={metrics['rmse_degrees']:.2f}°")
        
        return metrics
        
    def save_model(self, include_onnx: bool = True) -> Dict[str, str]:
        """
        Save trained model.
        
        Args:
            include_onnx: Whether to also export ONNX format for edge deployment
        
        Returns:
            Dict with paths to saved files
        """
        if self._model is None:
            raise RuntimeError("Model not trained")
            
        paths = {}
        model_dir = self.training_config.model_dir
        model_name = self.training_config.model_name
        
        # Save PyTorch model
        pt_path = os.path.join(model_dir, f"{model_name}.pt")
        save_pytorch_model(self._model, pt_path)
        paths['pytorch'] = pt_path
        logger.info(f"Saved PyTorch model to {pt_path}")
        
        # Save ONNX model for edge deployment (Raspberry Pi)
        if include_onnx:
            onnx_path = os.path.join(model_dir, f"{model_name}.onnx")
            try:
                convert_to_onnx(self._model, onnx_path, self.model_config)
                paths['onnx'] = onnx_path
            except Exception as e:
                logger.warning(f"ONNX conversion failed: {e}")
            
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
        if self._history.loss:
            history_path = os.path.join(model_dir, f"{model_name}_history.json")
            with open(history_path, 'w') as f:
                json.dump(self._history.to_dict(), f, indent=2)
            paths['history'] = history_path
            
        return paths
        
    def load_model(self, model_path: str):
        """Load a previously trained model."""
        self._model = load_pytorch_model(model_path, self._device)
        logger.info(f"Loaded model from {model_path}")


def train_from_logs(data_dir: str, 
                    output_dir: str = "models",
                    epochs: int = 100,
                    max_time_seconds: Optional[float] = None,
                    resume: bool = True,
                    use_mlflow: bool = False,
                    mlflow_experiment_name: str = "autopilot",
                    mlflow_run_name: Optional[str] = None) -> Dict[str, str]:
    """
    Convenience function to train model from logged data.
    
    Args:
        data_dir: Directory containing log files
        output_dir: Directory for output models
        epochs: Number of training epochs
        max_time_seconds: Maximum training time in seconds (None = no limit)
        resume: Whether to resume from checkpoint if available
        use_mlflow: Enable MLflow experiment tracking
        mlflow_experiment_name: MLflow experiment name
        mlflow_run_name: Optional name for this MLflow run
        
    Returns:
        Dict with paths to saved model files
    """
    config = TrainingConfig(
        epochs=epochs,
        model_dir=output_dir,
        max_time_seconds=max_time_seconds,
        resume_from_checkpoint=resume,
        use_mlflow=use_mlflow,
        mlflow_experiment_name=mlflow_experiment_name,
        mlflow_run_name=mlflow_run_name,
    )
    
    trainer = ImitationTrainer(training_config=config)
    
    # Load and split data
    print(f"Loading training data from: {data_dir}")
    X_train, y_train, X_val, y_val = trainer.load_data(data_dir)
    print(f"Loaded {len(X_train)} training samples, {len(X_val)} validation samples")
    
    # Setup MLflow if enabled
    if use_mlflow:
        if not HAS_MLFLOW:
            print("Warning: MLflow requested but not installed. Run: uv pip install mlflow")
        else:
            mlflow.set_experiment(mlflow_experiment_name)
            print(f"MLflow tracking enabled. Experiment: {mlflow_experiment_name}")
    
    # Run training with optional MLflow context
    if use_mlflow and HAS_MLFLOW:
        with mlflow.start_run(run_name=mlflow_run_name):
            # Log custom parameters
            _log_mlflow_params(config, trainer.model_config, trainer.data_config,
                             len(X_train), len(X_val))
            
            # Train
            history = trainer.train(X_train, y_train, X_val, y_val)
            
            # Log training metrics per epoch
            for epoch, (loss, val_loss) in enumerate(zip(history['loss'], history['val_loss'])):
                mlflow.log_metrics({'loss': loss, 'val_loss': val_loss}, step=epoch)
            
            # Evaluate and log metrics
            metrics = trainer.evaluate(X_val, y_val)
            _log_mlflow_metrics(metrics)
            
            # Save and log artifacts
            paths = trainer.save_model()
            _log_mlflow_artifacts(paths, trainer._model)
            
            return paths
    else:
        # Train without MLflow
        trainer.train(X_train, y_train, X_val, y_val)
        trainer.evaluate(X_val, y_val)
        return trainer.save_model()


def _log_mlflow_params(training_config: TrainingConfig, 
                       model_config: 'ModelConfig',
                       data_config: 'DataConfig',
                       num_train: int,
                       num_val: int):
    """Log custom parameters to MLflow."""
    if not HAS_MLFLOW:
        return
        
    # Training config
    mlflow.log_params({
        "learning_rate": training_config.learning_rate,
        "batch_size": training_config.batch_size,
        "epochs": training_config.epochs,
        "patience": training_config.patience,
        "min_delta": training_config.min_delta,
        "warmup_epochs": training_config.warmup_epochs,
        "add_noise": training_config.add_noise,
        "noise_std": training_config.noise_std,
        "resume_from_checkpoint": training_config.resume_from_checkpoint,
    })
    
    # Model config
    mlflow.log_params({
        "lstm_units_1": model_config.lstm_units_1,
        "lstm_units_2": model_config.lstm_units_2,
        "dense_units": model_config.dense_units,
        "dropout_rate": model_config.dropout_rate,
        "sequence_length": model_config.sequence_length,
        "feature_dim": model_config.feature_dim,
    })
    
    # Data config
    mlflow.log_params({
        "sample_rate_hz": data_config.sample_rate_hz,
        "train_split": data_config.train_split,
        "normalize": data_config.normalize,
    })
    
    # Dataset stats
    mlflow.log_params({
        "num_train_samples": num_train,
        "num_val_samples": num_val,
    })


def _log_mlflow_metrics(metrics: Dict[str, float]):
    """Log evaluation metrics to MLflow."""
    if not HAS_MLFLOW:
        return
        
    # Log metrics converted to degrees (more interpretable)
    mlflow.log_metrics({
        "eval_mae_degrees": metrics.get("mae_degrees", 0),
        "eval_rmse_degrees": metrics.get("rmse_degrees", 0),
        "eval_max_error_degrees": metrics.get("max_error_degrees", 0),
        "eval_std_error": metrics.get("std_error", 0),
    })


def _log_mlflow_artifacts(paths: Dict[str, str], model: 'AutopilotLSTM'):
    """Log model artifacts to MLflow."""
    if not HAS_MLFLOW:
        return
    
    # Log PyTorch model
    mlflow.pytorch.log_model(model, "model")
        
    # Log ONNX model
    if "onnx" in paths and os.path.exists(paths["onnx"]):
        mlflow.log_artifact(paths["onnx"])
        
    # Log config
    if "config" in paths and os.path.exists(paths["config"]):
        mlflow.log_artifact(paths["config"])
        
    # Log training history
    if "history" in paths and os.path.exists(paths["history"]):
        mlflow.log_artifact(paths["history"])


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
    
    # MLflow options
    parser.add_argument("--mlflow", action="store_true",
                       help="Enable MLflow experiment tracking")
    parser.add_argument("--experiment-name", default="autopilot",
                       help="MLflow experiment name (default: autopilot)")
    parser.add_argument("--run-name", default=None,
                       help="MLflow run name (optional)")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    paths = train_from_logs(
        args.data_dir, 
        args.output, 
        args.epochs, 
        args.max_time,
        resume=not args.fresh,
        use_mlflow=args.mlflow,
        mlflow_experiment_name=args.experiment_name,
        mlflow_run_name=args.run_name,
    )
    
    print("\nModel files saved:")
    for name, path in paths.items():
        print(f"  {name}: {path}")
