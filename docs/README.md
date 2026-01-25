# Documentation

This folder contains operational documentation for the ML Yacht Autopilot project.

## Guides

| Document | Description |
|----------|-------------|
| [Data Preparation](data_preparation.md) | Complete guide for preprocessing logs and preparing training data |
| [Simulated Training Data](simulated_training_data.md) | Guide for generating simulated training data |
| [Hardware Simulators](hardware_simulators.md) | IMU, Actuator, and CAN simulators for testing |
| [Model Architecture](model_architecture.md) | LSTM autopilot model architecture and features |
| [Model Evaluation Report](model_evaluation_report.md) | Current model performance and experiment results |
| [Model Export (ONNX)](model_export_onnx.md) | ONNX export for Raspberry Pi deployment |
| [MLflow Tracking](mlflow.md) | Experiment tracking setup and usage |
| [PD Controller](pd_controller.md) | Proportional-Derivative controller explanation |
| [Planned Passage Experiment](planned_passage_experiment.md) | How to run the passage following experiment |
| [Helm Controller Bug Fix](helm_controller_bug_fix.md) | Critical bug fix in simulation helm controller |
| [Feature Engineering Sign Convention](feature_engineering_sign_convention.md) | Sign convention alignment for error signals |

## Quick Reference

### Log Organization

```bash
# Preview what will be organized
uv run python -m src.n2k.log_organizer n2klogs/raw/

# Execute organization
uv run python -m src.n2k.log_organizer n2klogs/raw/ --execute
```

### Log Analysis

```bash
# Analyze logs and generate metadata
uv run python -m src.training.log_analyzer n2klogs/raw/

# Dry run (preview only)
uv run python -m src.training.log_analyzer n2klogs/raw/ --dry-run
```

## Additional Resources

- [planning/](../planning/) - Design documents and implementation plans
- [specification.md](../specification.md) - System specification
- [README.md](../README.md) - Project overview
