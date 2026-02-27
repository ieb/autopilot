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
from typing import Optional, Dict, Any, List, Tuple
import numpy as np
import logging

from .data_loader import TrainingDataLoader, DataConfig, MemmapDataset, FrameDataset
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
    feature_dim: int = 22
    
    # Training
    epochs: int = 100
    batch_size: int = 512
    learning_rate: float = 1e-4  # Lower LR for stability
    
    # Regularization
    weight_decay: float = 1e-4

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
        """Load training data from directory (all in memory)."""
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
        Train the model from in-memory numpy arrays.
        
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
            if resumed:
                self.training_config.warmup_epochs = 0
            
        if self.training_config.add_noise:
            X_train = self._augment_data(X_train)
        
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
        
        return self._train_loop(train_loader, val_loader,
                                len(X_train), len(X_val))
    
    def train_streaming(self, 
                        train_loader: DataLoader,
                        val_loader: DataLoader) -> Dict[str, Any]:
        """
        Train the model from DataLoaders (binary mode).
        
        Args:
            train_loader: Training DataLoader 
            val_loader: Validation DataLoader
            
        Returns:
            Training history dict
        """
        if self._model is None:
            resumed = self.build_or_load_model()
            if resumed:
                self.training_config.warmup_epochs = 0
        
        n_train = len(train_loader.dataset)
        n_val = len(val_loader.dataset)
        
        return self._train_loop(train_loader, val_loader, n_train, n_val)
    
    def _train_loop(self, train_loader: DataLoader, val_loader: DataLoader,
                    n_train: int, n_val: int) -> Dict[str, Any]:
        """Core training loop shared by in-memory and streaming modes."""
        
        # Setup training
        # Use element-wise MSE (no reduction) so we can apply per-sample
        # weighting based on heading error magnitude.  Large errors get
        # higher weight so the model learns assertive corrections.
        criterion = nn.MSELoss(reduction='none')
        optimizer = torch.optim.Adam(
            self._model.parameters(),
            lr=self.training_config.learning_rate,
            weight_decay=self.training_config.weight_decay,
        )
        
        # Learning rate schedulers
        # Warmup scheduler (use start_factor=1.0 when no warmup to avoid
        # LinearLR silently reducing the initial LR on construction)
        warmup_epochs = self.training_config.warmup_epochs
        warmup_scheduler = torch.optim.lr_scheduler.LinearLR(
            optimizer,
            start_factor=0.1 if warmup_epochs > 0 else 1.0,
            end_factor=1.0,
            total_iters=max(warmup_epochs, 1),
        )
        # Reduce on plateau scheduler
        plateau_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            optimizer,
            mode='min',
            factor=0.5,
            patience=3,
            min_lr=1e-6,
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
        print(f"Training samples: {n_train:,}, Validation samples: {n_val:,}")
        print()
        
        logger.info(f"Training on {n_train} samples, validating on {n_val}")
        
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
                per_sample_loss = criterion(output, y_batch)

                # Weight by heading error magnitude: samples with large
                # errors get up to 2x weight so the model learns assertive
                # rudder commands without overwhelming small-error precision.
                # heading_error is feature index 0, normalized to [-1,1].
                heading_err = X_batch[:, -1, 0].abs()  # last timestep
                weights = torch.clamp(1.0 + heading_err, min=1.0, max=2.0)
                loss = (per_sample_loss.squeeze() * weights).mean()

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
                    # Unweighted mean for honest validation comparison
                    loss = criterion(output, y_batch).mean()

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
            
            # Live MLflow logging (if active)
            if HAS_MLFLOW and mlflow and mlflow.active_run():
                mlflow.log_metrics({
                    'loss': train_loss,
                    'val_loss': val_loss,
                    'mae_degrees': mae_deg,
                    'val_mae_degrees': val_mae_deg,
                    'lr': current_lr,
                }, step=epoch)
            
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
        
        # Final closed-loop validation
        final_cl = closed_loop_validate_pytorch(self._model, self._device)
        final_cl_score = _cl_mean_error(final_cl)
        _print_cl_results(final_cl)

        # Print summary
        elapsed = time.time() - start_time
        print("=" * 70)
        print("TRAINING COMPLETE")
        print("=" * 70)
        print(f"Total time: {elapsed:.1f}s ({elapsed/60:.1f} minutes)")
        print(f"Final validation loss: {self._history.val_loss[-1]:.6f}")
        print(f"Best validation loss: {best_val_loss:.6f}")
        print(f"Final CL score: {final_cl_score:.1f}°")
        
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

        batch_size = self.training_config.batch_size
        n = len(X_test)
        all_preds = []
        total_loss = 0.0
        total_mae = 0.0

        with torch.no_grad():
            for start in range(0, n, batch_size):
                end = min(start + batch_size, n)
                xb = torch.from_numpy(X_test[start:end]).float().to(self._device)
                yb = torch.from_numpy(y_test[start:end]).float().to(self._device)
                preds = self._model(xb)
                total_loss += nn.MSELoss(reduction='sum')(preds, yb).item()
                total_mae += torch.sum(torch.abs(preds - yb)).item()
                all_preds.append(preds.cpu().numpy())

        loss = total_loss / n
        mae = total_mae / n
        predictions_np = np.concatenate(all_preds, axis=0)
        errors = predictions_np.flatten() - y_test.flatten()

        return self._compute_eval_metrics(loss, mae, errors)

    def evaluate_loader(self, data_loader: DataLoader) -> Dict[str, float]:
        """Evaluate model using a DataLoader (binary / streaming mode)."""
        if self._model is None:
            raise RuntimeError("Model not trained")

        self._model.eval()

        n = len(data_loader.dataset)
        all_preds = []
        all_targets = []
        total_loss = 0.0
        total_mae = 0.0

        with torch.no_grad():
            for X_batch, y_batch in data_loader:
                X_batch = X_batch.to(self._device)
                y_batch = y_batch.to(self._device)
                preds = self._model(X_batch)
                total_loss += nn.MSELoss(reduction='sum')(preds, y_batch).item()
                total_mae += torch.sum(torch.abs(preds - y_batch)).item()
                all_preds.append(preds.cpu().numpy())
                all_targets.append(y_batch.cpu().numpy())

        loss = total_loss / n
        mae = total_mae / n
        predictions_np = np.concatenate(all_preds, axis=0)
        targets_np = np.concatenate(all_targets, axis=0)
        errors = predictions_np.flatten() - targets_np.flatten()

        return self._compute_eval_metrics(loss, mae, errors)

    @staticmethod
    def _compute_eval_metrics(loss: float, mae: float,
                              errors: np.ndarray) -> Dict[str, float]:
        """Compute evaluation metrics from raw loss, MAE, and error vector."""
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


def _has_binary_files(data_dir: str) -> bool:
    """Check if a directory contains .bin binary frame files."""
    return any(Path(data_dir).glob('**/*.bin'))


def train_from_logs(data_dir: str, 
                    output_dir: str = "models",
                    epochs: int = 100,
                    max_time_seconds: Optional[float] = None,
                    resume: bool = True,
                    use_mlflow: bool = False,
                    mlflow_experiment_name: str = "autopilot",
                    mlflow_run_name: Optional[str] = None,
                    force_json: bool = False) -> Dict[str, str]:
    """
    Convenience function to train model from logged data.
    
    Auto-detects binary .bin files and uses the efficient FrameDataset
    path by default.  Falls back to JSON loading if no .bin files are
    found or if force_json is True.
    
    Args:
        data_dir: Directory containing log files (.bin or .jsonlog)
        output_dir: Directory for output models
        epochs: Number of training epochs
        max_time_seconds: Maximum training time in seconds (None = no limit)
        resume: Whether to resume from checkpoint if available
        use_mlflow: Enable MLflow experiment tracking
        mlflow_experiment_name: MLflow experiment name
        mlflow_run_name: Optional name for this MLflow run
        force_json: Force legacy JSON loading even when .bin files exist
        
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

    # Determine data loading mode:
    #   1. Binary .bin files (default, most efficient)
    #   2. Streaming (memory-mapped .npy, legacy)
    #   3. In-memory JSON (legacy)
    use_binary = _has_binary_files(data_dir)

    if use_binary:
        print(f"Loading binary training data from: {data_dir}")
        file_data = TrainingDataLoader.load_binary_directory(data_dir)

        # Round-robin split files into train / val
        train_files, val_files = [], []
        val_every = max(2, round(1.0 / (1.0 - trainer.data_config.train_split)))
        for i, fd in enumerate(file_data):
            if i % val_every == (val_every - 1):
                val_files.append(fd)
            else:
                train_files.append(fd)
        if not val_files:
            val_files.append(train_files.pop())



        seq_len = trainer.data_config.sequence_length
        train_ds = FrameDataset.from_file_list(train_files, seq_len=seq_len)
        val_ds = FrameDataset.from_file_list(val_files, seq_len=seq_len)


        use_pin = trainer._device.type == 'cuda'
        train_loader = DataLoader(train_ds,
                                  batch_size=trainer.training_config.batch_size,
                                  shuffle=True, num_workers=0,
                                  pin_memory=use_pin)
        val_loader = DataLoader(val_ds,
                                batch_size=trainer.training_config.batch_size,
                                shuffle=False, num_workers=0,
                                pin_memory=use_pin)

        print(f"Binary mode: {len(train_ds):,} train, "
              f"{len(val_ds):,} val sequences (on-the-fly, mirror={True})")

    else:
        print(f"Loading training data (JSON) from: {data_dir}")
        X_train, y_train, X_val, y_val = trainer.load_data(data_dir)
        print(f"Loaded {len(X_train):,} training samples, "
              f"{len(X_val):,} validation samples")
    
    # Setup MLflow if enabled
    if use_mlflow:
        if not HAS_MLFLOW:
            print("Warning: MLflow requested but not installed. Run: uv pip install mlflow")
        else:
            mlflow.set_experiment(mlflow_experiment_name)
            print(f"MLflow tracking enabled. Experiment: {mlflow_experiment_name}")
    
    def _do_train():
        if use_binary:
            return trainer.train_streaming(train_loader, val_loader)
        else:
            return trainer.train(X_train, y_train, X_val, y_val)
    
    def _do_evaluate():
        if use_binary:
            return trainer.evaluate_loader(val_loader)
        else:
            return trainer.evaluate(X_val, y_val)

    # Run training with optional MLflow context
    uses_loaders = use_binary
    if use_mlflow and HAS_MLFLOW:
        n_train = len(train_loader.dataset) if uses_loaders else len(X_train)
        n_val = len(val_loader.dataset) if uses_loaders else len(X_val)
        
        with mlflow.start_run(run_name=mlflow_run_name):
            _log_mlflow_params(config, trainer.model_config, trainer.data_config,
                             n_train, n_val)
            
            history = _do_train()
            
            metrics = _do_evaluate()
            if metrics:
                _log_mlflow_metrics(metrics)
            
            paths = trainer.save_model()
            _log_mlflow_artifacts(paths, trainer._model)
            return paths
    else:
        _do_train()
        _do_evaluate()
        return trainer.save_model()


# Heading error (degrees) above which the model correction fades to zero,
# leaving the PD controller in full control of large transients.
BLEND_FADE_DEG = 5.0


def _blend_alpha(heading_error_feat: float) -> float:
    """Compute blending factor for model correction vs PD controller.

    Returns 1.0 (full model influence) when heading error is zero,
    linearly fading to 0.0 (PD only) at BLEND_FADE_DEG degrees.
    """
    h_err_deg = abs(heading_error_feat) * 90.0  # feature 0 is normalised /90
    return max(0.0, 1.0 - h_err_deg / BLEND_FADE_DEG)


def _run_cl_scenario(predict_fn, name, initial_heading, twd, tws,
                     mode, target_heading, target_awa, target_twa,
                     run_sec, acceptable_error_deg,
                     physics_dt=0.1, inference_hz=2.0):
    """Run a single closed-loop steering scenario.

    The physics always runs at 1/physics_dt Hz for stability.
    Features are recorded and the model is queried at inference_hz,
    matching the training data rate.

    Args:
        predict_fn: callable(np.ndarray[20, 22]) -> float in [-1, 1]
        physics_dt: Physics timestep in seconds (default 0.1 = 10Hz)
        inference_hz: Rate at which features are recorded and the model
            is queried (should match training data recording rate)
    """
    from src.simulation.yacht_dynamics import YachtDynamics, YachtConfig
    from src.simulation.wind_model import WindModel, WindConfig
    from src.simulation.wave_model import WaveModel, WaveConfig
    from .data_loader import compute_features

    yacht = YachtDynamics(YachtConfig())
    yacht.state.heading = initial_heading
    yacht.state.stw = 6.0
    yacht.state.sog = 6.0
    wind = WindModel(WindConfig(base_twd=twd,
                                base_tws_min=tws, base_tws_max=tws,
                                shift_rate=0.0, oscillation_enabled=False))
    wind.reset()
    wind_state = wind.step(0)
    yacht.set_wind(wind_state.twd, wind_state.tws)

    # Wave model — match training data distribution
    waves = WaveModel(WaveConfig())
    waves.reset()

    feature_history: list = []
    tracking_errors: list = []
    rudder_cmds: list = []
    diagnostics: list = []  # (time, heading_error_feat, mode_feat, rudder_norm, awa, heading)

    # Rate tracking for derived features
    prev_awa = yacht.state.awa
    prev_rudder = yacht.state.rudder_angle
    prev_roll = yacht.state.roll

    # Physics steps per inference step
    steps_per_inference = max(1, round(1.0 / (inference_hz * physics_dt)))
    inference_dt = steps_per_inference * physics_dt
    n_steps = int(run_sec / physics_dt)
    rudder_deg = 0.0

    for step_i in range(n_steps):
        wind_state = wind.step(physics_dt)
        yacht.set_wind(wind_state.twd, wind_state.tws)

        # Step wave model
        twa = wind_state.twd - yacht.state.heading
        while twa > 180: twa -= 360
        while twa < -180: twa += 360
        wave_roll, wave_pitch, wave_yaw = waves.step(
            physics_dt, tws=wind_state.tws,
            heading=yacht.state.heading, twa=twa,
            heel=yacht.state.roll)

        yacht.step(rudder_deg, physics_dt, wave_yaw=wave_yaw)
        total_roll = yacht.state.roll + wave_roll
        total_pitch = yacht.state.pitch + wave_pitch

        # Record features and query model at inference_hz
        if step_i % steps_per_inference == 0:
            # Compute rates over the inference interval
            awa_rate = (yacht.state.awa - prev_awa) / inference_dt
            if awa_rate > 180 / inference_dt:
                awa_rate -= 360 / inference_dt
            elif awa_rate < -180 / inference_dt:
                awa_rate += 360 / inference_dt
            rudder_vel = (yacht.state.rudder_angle - prev_rudder) / inference_dt
            r_rate = (total_roll - prev_roll) / inference_dt
            prev_awa = yacht.state.awa
            prev_rudder = yacht.state.rudder_angle
            prev_roll = total_roll

            features = compute_features(
                heading=yacht.state.heading,
                pitch=total_pitch,
                roll=total_roll,
                yaw_rate=yacht.state.heading_rate,
                awa=yacht.state.awa,
                aws=yacht.state.aws,
                stw=yacht.state.stw,
                cog=yacht.state.cog,
                sog=yacht.state.sog,
                rudder_angle=yacht.state.rudder_angle,
                target_heading=target_heading,
                target_awa=target_awa,
                target_twa=target_twa,
                mode=mode,
                roll_rate=r_rate,
                awa_rate=awa_rate,
                rudder_velocity=rudder_vel,
                wave_period=waves.state.swell_period,
            )
            feature_history.append(features)
            if len(feature_history) > 20:
                feature_history.pop(0)
            while len(feature_history) < 20:
                feature_history.insert(0, feature_history[0])

            seq = np.array(feature_history, dtype=np.float32)
            correction = predict_fn(seq)
            alpha = _blend_alpha(features[0])
            rudder_norm = float(np.clip(
                features[19] + alpha * correction, -1.0, 1.0))
            rudder_deg = rudder_norm * 25.0
            rudder_cmds.append(rudder_deg)

            t = step_i * physics_dt
            diagnostics.append((
                t, features[0], features[1], features[19], rudder_norm,
                yacht.state.awa, yacht.state.heading, rudder_deg, alpha,
            ))

        # Track error every physics step for accurate measurement
        if mode == "wind_awa":
            err = yacht.state.awa - target_awa
        elif mode == "wind_twa":
            from .data_loader import _compute_true_wind
            twa, _ = _compute_true_wind(yacht.state.awa, yacht.state.aws,
                                        yacht.state.stw)
            err = twa - target_twa
        else:
            err = target_heading - yacht.state.heading
            while err > 180: err -= 360
            while err < -180: err += 360
        tracking_errors.append(abs(err))

    errors = np.array(tracking_errors)
    ss_start = int(0.75 * len(errors))
    ss_errors = errors[ss_start:]
    mean_ss = float(ss_errors.mean())
    max_ss = float(ss_errors.max())
    passed = mean_ss < acceptable_error_deg

    # Print diagnostic snapshots at key moments
    if diagnostics:
        print(f"    CL Diagnostics for {name} (blend_fade={BLEND_FADE_DEG}°):")
        print(f"    {'time':>6s}  {'h_err_f':>7s}  {'pd_sug':>7s}  {'alpha':>5s}  "
              f"{'rud_n':>7s}  {'rud_deg':>7s}  {'AWA':>7s}  {'heading':>7s}")
        # Print at t=0, 5, 10, 20, 30, 60, 90, and end
        snap_times = [0, 5, 10, 20, 30, 60, 90, run_sec - 1]
        for snap_t in snap_times:
            # Find closest diagnostic entry
            closest = min(diagnostics, key=lambda d: abs(d[0] - snap_t))
            t, h_err, mode_f, pd_sug, rud_n, awa, hdg, rud_d, alph = closest
            print(f"    {t:6.1f}  {h_err:+7.3f}  {pd_sug:+7.3f}  {alph:5.2f}  "
                  f"{rud_n:+7.3f}  {rud_d:+7.1f}  {awa:+7.1f}  {hdg:7.1f}")

    return {
        "name": name,
        "passed": passed,
        "mean_ss_error_deg": round(mean_ss, 2),
        "max_ss_error_deg": round(max_ss, 2),
        "max_transient_error_deg": round(float(errors.max()), 2),
        "mean_rudder_deg": round(float(np.mean(rudder_cmds)), 2),
        "acceptable_deg": acceptable_error_deg,
    }


# Standard closed-loop test suite
_CL_SCENARIOS = [
    # (name, initial_heading, twd, tws, mode, target_heading, target_awa, target_twa, run_sec, accept_deg)
    ("compass_small",  90.0, 180.0, 12.0, "compass",  95.0,   0.0,   0.0, 120.0,  5.0),
    ("compass_large",  45.0, 180.0, 12.0, "compass", 135.0,   0.0,   0.0,  60.0, 10.0),
    # Wind from 180°, heading 225° → AWA ≈ -45° (port). Start 10° off.
    ("wind_awa_hold", 215.0, 180.0, 15.0, "wind_awa", 225.0, -45.0,  0.0, 120.0, 10.0),
]


def _run_cl_suite(predict_fn, scenarios=None):
    """Run the closed-loop test suite and return results dict."""
    scenarios = scenarios or _CL_SCENARIOS
    results = []
    for args in scenarios:
        r = _run_cl_scenario(predict_fn, *args)
        results.append(r)
    return results


def _cl_mean_error(results):
    """Aggregate score from closed-loop results (lower is better)."""
    return sum(r["mean_ss_error_deg"] for r in results) / max(len(results), 1)


def _print_cl_results(results):
    """Pretty-print closed-loop validation results."""
    all_passed = all(r["passed"] for r in results)
    print("\n" + "=" * 60)
    print("CLOSED-LOOP VALIDATION")
    print("=" * 60)
    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        print(f"  [{status}] {r['name']}: "
              f"SS error {r['mean_ss_error_deg']:.1f}° "
              f"(max {r['max_ss_error_deg']:.1f}°, "
              f"accept <{r['acceptable_deg']}°) "
              f"rudder_mean={r['mean_rudder_deg']:.1f}°")
    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")
    print("=" * 60 + "\n")
    return all_passed


def dagger_collect(model, device, data_dir: str, iteration: int) -> str:
    """Collect DAgger data by running the model in closed-loop simulation.

    The MODEL steers the boat (capturing distribution-shifted states) while
    an expert PD controller provides the ideal rudder label for each frame.
    Frames are written as binary records so they are automatically included
    in the next training round.

    Args:
        model: Trained PyTorch model (will be copied to CPU).
        device: Current training device (unused, model copied to CPU).
        data_dir: Directory to write the .bin file into.
        iteration: DAgger iteration number (used in filename).

    Returns:
        Path to the written .bin file.
    """
    import copy
    import struct
    import random as _rng
    from src.simulation.yacht_dynamics import YachtDynamics, YachtConfig, YachtState
    from src.simulation.wind_model import WindModel, WindConfig
    from src.simulation.wave_model import WaveModel, WaveConfig
    from src.simulation.helm_controller import HelmController, HelmConfig, SteeringMode
    from .data_loader import compute_features

    # Prepare model for inference on CPU
    cpu_model = copy.deepcopy(model).cpu().eval()

    def _predict(sequence):
        t = torch.from_numpy(sequence[np.newaxis]).float()
        with torch.no_grad():
            out = cpu_model(t)
        return float(out.numpy()[0, 0])  # raw correction; caller adds pd_sug

    # Expert controller — clean PD, no noise/fatigue/delay
    expert = HelmController(HelmConfig(
        compass_kp=1.6, compass_kd=1.5,
        skill_level=1.0,
        noise_std=0.0,
        fatigue_enabled=False,
        reaction_delay=0.0,
        max_rudder_rate=4.0,
    ))

    # Build diverse scenario list
    # (name, initial_heading, twd, tws, mode, target_heading, target_awa, target_twa, run_sec)
    scenarios = []

    # ~40 compass scenarios
    for i in range(40):
        tws = _rng.uniform(8, 25)
        twd = _rng.uniform(0, 360)
        initial_heading = _rng.uniform(0, 360)
        error_mag = _rng.uniform(5, 90)
        sign = _rng.choice([-1, 1])
        target_heading = (initial_heading + sign * error_mag) % 360
        scenarios.append((
            f"dagger_compass_{i}", initial_heading, twd, tws,
            "compass", target_heading, 0.0, 0.0, 120.0,
        ))

    # ~40 wind_awa scenarios — balanced tack
    for i in range(40):
        tws = _rng.uniform(8, 25)
        twd = _rng.uniform(0, 360)
        target_awa_mag = _rng.uniform(30, 90)
        tack = _rng.choice([-1, 1])  # port or starboard
        target_awa = tack * target_awa_mag
        # Start heading offset by 10-30° from equilibrium
        equilibrium_heading = (twd - target_awa) % 360
        offset = _rng.uniform(10, 30) * _rng.choice([-1, 1])
        initial_heading = (equilibrium_heading + offset) % 360
        scenarios.append((
            f"dagger_awa_{i}", initial_heading, twd, tws,
            "wind_awa", 0.0, target_awa, 0.0, 120.0,
        ))

    # ~20 wind_twa scenarios
    for i in range(20):
        tws = _rng.uniform(10, 25)
        twd = _rng.uniform(0, 360)
        target_twa_mag = _rng.uniform(90, 170)
        tack = _rng.choice([-1, 1])
        target_twa = tack * target_twa_mag
        equilibrium_heading = (twd - target_twa) % 360
        offset = _rng.uniform(10, 30) * _rng.choice([-1, 1])
        initial_heading = (equilibrium_heading + offset) % 360
        scenarios.append((
            f"dagger_twa_{i}", initial_heading, twd, tws,
            "wind_twa", 0.0, 0.0, target_twa, 120.0,
        ))

    # Binary file constants (match data_generator.py)
    BIN_MAGIC = b'APFD'
    BIN_VERSION = 1
    BIN_COLS = 23  # 22 features + 1 label

    # Use timestamp to avoid overwriting DAgger files from previous runs
    import time as _time
    ts = _time.strftime('%Y%m%d%H%M%S')
    out_path = Path(data_dir) / f"dagger_iter{iteration}_{ts}.bin"
    total_frames = 0

    physics_dt = 0.1
    inference_hz = 2.0
    steps_per_inference = max(1, round(1.0 / (inference_hz * physics_dt)))
    inference_dt = steps_per_inference * physics_dt

    with open(out_path, 'wb') as f:
        # Write header
        f.write(BIN_MAGIC)
        f.write(struct.pack('<III', BIN_VERSION, BIN_COLS - 1, 0))

        for sc in scenarios:
            (name, initial_heading, twd, tws,
             mode, target_heading, target_awa, target_twa, run_sec) = sc

            # Setup physics
            yacht = YachtDynamics(YachtConfig())
            yacht.state.heading = initial_heading
            yacht.state.stw = 6.0
            yacht.state.sog = 6.0
            wind = WindModel(WindConfig(
                base_twd=twd,
                base_tws_min=tws, base_tws_max=tws,
                shift_rate=0.0, oscillation_enabled=False,
            ))
            wind.reset()
            wind_state = wind.step(0)
            yacht.set_wind(wind_state.twd, wind_state.tws)

            waves = WaveModel(WaveConfig())
            waves.reset()

            # Setup expert for this scenario
            expert.reset()
            if mode == "compass":
                expert.set_mode(SteeringMode.COMPASS, target_heading)
            elif mode == "wind_awa":
                expert.set_mode(SteeringMode.WIND_AWA, target_awa)
            elif mode == "wind_twa":
                expert.set_mode(SteeringMode.WIND_TWA, target_twa)

            feature_history: list = []
            prev_awa = yacht.state.awa
            prev_rudder = yacht.state.rudder_angle
            prev_roll = yacht.state.roll

            n_steps = int(run_sec / physics_dt)
            rudder_deg = 0.0  # model's rudder command

            for step_i in range(n_steps):
                wind_state = wind.step(physics_dt)
                yacht.set_wind(wind_state.twd, wind_state.tws)

                twa_local = wind_state.twd - yacht.state.heading
                while twa_local > 180: twa_local -= 360
                while twa_local < -180: twa_local += 360
                wave_roll, wave_pitch, wave_yaw = waves.step(
                    physics_dt, tws=wind_state.tws,
                    heading=yacht.state.heading, twa=twa_local,
                    heel=yacht.state.roll)

                yacht.step(rudder_deg, physics_dt, wave_yaw=wave_yaw)
                total_roll = yacht.state.roll + wave_roll
                total_pitch = yacht.state.pitch + wave_pitch

                # Record features and query model at inference_hz
                if step_i % steps_per_inference == 0:
                    awa_rate = (yacht.state.awa - prev_awa) / inference_dt
                    if awa_rate > 180 / inference_dt:
                        awa_rate -= 360 / inference_dt
                    elif awa_rate < -180 / inference_dt:
                        awa_rate += 360 / inference_dt
                    rudder_vel = (yacht.state.rudder_angle - prev_rudder) / inference_dt
                    r_rate = (total_roll - prev_roll) / inference_dt
                    prev_awa = yacht.state.awa
                    prev_rudder = yacht.state.rudder_angle
                    prev_roll = total_roll

                    features = compute_features(
                        heading=yacht.state.heading,
                        pitch=total_pitch,
                        roll=total_roll,
                        yaw_rate=yacht.state.heading_rate,
                        awa=yacht.state.awa,
                        aws=yacht.state.aws,
                        stw=yacht.state.stw,
                        cog=yacht.state.cog,
                        sog=yacht.state.sog,
                        rudder_angle=yacht.state.rudder_angle,
                        target_heading=target_heading,
                        target_awa=target_awa,
                        target_twa=target_twa,
                        mode=mode,
                        roll_rate=r_rate,
                        awa_rate=awa_rate,
                        rudder_velocity=rudder_vel,
                        wave_period=waves.state.swell_period,
                    )
                    feature_history.append(features)
                    if len(feature_history) > 20:
                        feature_history.pop(0)
                    while len(feature_history) < 20:
                        feature_history.insert(0, feature_history[0])

                    # MODEL steers (residual: blended correction + pd_suggestion)
                    seq = np.array(feature_history, dtype=np.float32)
                    correction = _predict(seq)
                    alpha = _blend_alpha(features[0])
                    rudder_norm = float(np.clip(
                        features[19] + alpha * correction, -1.0, 1.0))
                    rudder_deg = rudder_norm * 25.0

                    # EXPERT labels
                    expert_rudder = expert.compute_rudder(
                        heading=yacht.state.heading,
                        awa=yacht.state.awa,
                        twa=twa_local,
                        heading_rate=yacht.state.heading_rate,
                        dt=inference_dt,
                        aws=yacht.state.aws,
                        stw=yacht.state.stw,
                    )
                    expert_label = np.clip(expert_rudder / 25.0, -1.0, 1.0)

                    # Write binary frame: features + expert label
                    row = np.empty(BIN_COLS, dtype=np.float32)
                    row[:BIN_COLS - 1] = features
                    row[BIN_COLS - 1] = expert_label
                    f.write(row.tobytes())
                    total_frames += 1

    # Write companion .meta.json
    import json as _json
    meta = {
        'format': 'binary_frame_v1',
        'frame_count': total_frames,
        'feature_dim': 22,
        'columns': 23,
        'transition_times': [],
        'dagger_iteration': iteration,
        'num_scenarios': len(scenarios),
    }
    meta_path = out_path.with_suffix('.meta.json')
    with open(meta_path, 'w') as mf:
        _json.dump(meta, mf, indent=2)

    print(f"\nDAgger iteration {iteration}: wrote {total_frames:,} frames "
          f"from {len(scenarios)} scenarios -> {out_path}")
    return str(out_path)


def closed_loop_validate(model_path: str) -> Dict[str, Any]:
    """Run closed-loop steering tests against a saved model file.

    Supports .onnx and .pt model files.

    Returns dict with per-test results and an overall PASS/FAIL.
    """
    from src.ml.autopilot_model import AutopilotInference
    autopilot = AutopilotInference(model_path)
    results = _run_cl_suite(autopilot.predict)
    all_passed = _print_cl_results(results)
    return {"passed": all_passed, "tests": results}


def closed_loop_validate_pytorch(model, device=None):
    """Run closed-loop steering tests against an in-memory PyTorch model.

    Used during training to evaluate closed-loop performance without
    saving/loading from disk.  The model is temporarily moved to CPU
    to avoid interfering with GPU training memory.
    """
    import copy
    cpu_model = copy.deepcopy(model).cpu().eval()

    def _predict(sequence):
        t = torch.from_numpy(sequence[np.newaxis]).float()
        with torch.no_grad():
            out = cpu_model(t)
        return float(out.numpy()[0, 0])  # raw correction; _run_cl_scenario adds pd_sug

    results = _run_cl_suite(_predict)
    return results


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
    import sys
    
    parser = argparse.ArgumentParser(description="Train autopilot model")
    parser.add_argument("data_dir", nargs='?', default=None,
                       help="Directory containing training logs")
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
    
    # Data format options
    parser.add_argument("--json", action="store_true",
                       help="Force legacy JSON loading (default: auto-detect .bin)")
    parser.add_argument("--preprocess", action="store_true",
                       help="Preprocess data to .npy files and exit (for streaming)")
    parser.add_argument("--preprocessed-dir", default=None,
                       help="Directory with preprocessed .npy files (auto-enables streaming)")
    parser.add_argument("--streaming", action="store_true",
                       help="Use memory-mapped streaming for large datasets")

    # DAgger
    parser.add_argument("--dagger", type=int, default=0, metavar="N",
                       help="Number of DAgger iterations after initial training (default: 0)")

    # Standalone validation mode
    parser.add_argument("--validate", default=None, metavar="MODEL_PATH",
                       help="Run closed-loop validation on a model (.onnx) and exit")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)

    # Validate-only mode
    if args.validate:
        result = closed_loop_validate(args.validate)
        sys.exit(0 if result["passed"] else 1)

    if not args.data_dir:
        parser.error("data_dir is required for training")

    # Preprocess-only mode
    if args.preprocess:
        from .data_loader import TrainingDataLoader
        loader = TrainingDataLoader()
        preprocess_dir = args.preprocessed_dir or (args.data_dir.rstrip('/') + '_preprocessed')
        manifest = loader.preprocess_to_disk(args.data_dir, preprocess_dir)
        print(f"\nPreprocessed {manifest['total_sequences']:,} sequences "
              f"from {len(manifest['files'])} files -> {preprocess_dir}")
        print(f"\nTo train with streaming:")
        print(f"  uv run python -m src.training.train_imitation "
              f"{args.data_dir} --preprocessed-dir {preprocess_dir}")
        sys.exit(0)
    
    paths = train_from_logs(
        args.data_dir,
        args.output,
        args.epochs,
        args.max_time,
        resume=not args.fresh,
        use_mlflow=args.mlflow,
        mlflow_experiment_name=args.experiment_name,
        mlflow_run_name=args.run_name,
        force_json=args.json,
    )

    print("\nModel files saved:")
    for name, path in paths.items():
        print(f"  {name}: {path}")

    # DAgger iterations
    if args.dagger > 0:
        device = get_device()
        for dag_iter in range(1, args.dagger + 1):
            print("\n" + "=" * 70)
            print(f"DAGGER ITERATION {dag_iter}/{args.dagger}")
            print("=" * 70)

            # Load the best model from the previous round
            checkpoint_path = os.path.join(args.output, "autopilot_best.pt")
            model = load_pytorch_model(checkpoint_path, device)

            # Collect DAgger data (model steers, expert labels)
            dagger_collect(model, device, args.data_dir, dag_iter)

            # Retrain on ALL .bin files (original + DAgger)
            paths = train_from_logs(
                args.data_dir,
                args.output,
                args.epochs,
                args.max_time,
                resume=True,  # resume from checkpoint
                use_mlflow=args.mlflow,
                mlflow_experiment_name=args.experiment_name,
                mlflow_run_name=f"dagger_iter{dag_iter}" if not args.run_name
                    else f"{args.run_name}_dagger{dag_iter}",
                force_json=args.json,
            )

            print(f"\nDAgger iteration {dag_iter} complete. Model files:")
            for name, path in paths.items():
                print(f"  {name}: {path}")
