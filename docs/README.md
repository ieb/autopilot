# Documentation

This folder contains operational documentation for the ML Yacht Autopilot project.

# Standard Pilots

* PD
* PID

## Evaluting



PD pilot — passes all 3 CL scenarios (CL score: 3.6°):

  - compass_small: PASS 0.4° (max 0.7°)
  - compass_large: PASS 9.4° (max 39.9°) — oscillates hard but settles in time
  - wind_awa_hold: PASS 1.0° (max 4.8°)

PID pilot (ki=0.1) — passes 2/3 (CL score: 7.6°):

  - compass_small: PASS 0.1° — best of any controller so far
  - compass_large: FAIL 21.8° — integral term causes overshoot in large transients
  - wind_awa_hold: PASS 1.0°

  CLI Usage

        uv run python -m src.pilots.evaluate --pilot pd
        uv run python -m src.pilots.evaluate --pilot pid --ki 0.2
        uv run python -m src.pilots.evaluate --pilot pd --kp 2.0 --kd 1.8 --verbose
        uv run python -m src.pilots.evaluate --list

# Overall process of training the ML Model for the Autopilot

## Training Data
* Gather training datasets while sailing. Remember that the model will only be as good as the training dataset, so use your best helm, which might be the existing autopilot. see [Data Preparation](data_preparation.md) 
* Generate simulated training data. While this can be an easy fast way to generate data, it will only ever be as good a training dataset as the input material, which is the polar, the physics for the boat and if gribs are available to drive the simulation, those gribs. see [Simulated Training Data](simulated_training_data.md)

## Model Training
* Train the model, using MLFlow to track convergence. After this you will have a trained model based on the training datasets. This model is only going to be able to predict the autopilot actions for the conditions it was trained on. If it wasnt trained for 3m swells and 30kn of wind it will be no better than an empirical autopilot approach in those conditions. Similarly if there is no training data for motoring flat calm, then it may send the boat round in circles under power. See [Model Training Guide](model_training.md) for the full workflow.

## Evaluate

* Evaluate the model against planned passages see [Planned Passage Experiment](planned_passage_experiment.md) to see if it can steer the course in the conditions. Planned passages that use isochrone and polar techniques will provide a realistic target for the pilot to achieve taking into account that the boat has a predicted speed and wont sail directly upwind.

## repeat

Each cycle of training, where the evaluations gve better results than the previous iteration should result in a better model. How many iterations will be require is tbd.

## Guides

| Document | Description |
|----------|-------------|
| [Blended PD ML Controller](blended_pd_ml_controller.md) | Current best controller | 
| [Data Preparation](data_preparation.md) | Complete guide for preprocessing logs and preparing training data |
| [Simulated Training Data](simulated_training_data.md) | Guide for generating simulated training data |
| [Hardware Simulators](hardware_simulators.md) | IMU, Actuator, and CAN simulators for testing |
| [Heel Physics Model](heel_physics_model.md) | Physics-based heel/roll model with auto-reefing |
| [Model Training](model_training.md) | End-to-end guide for training the autopilot model |
| [Model Architecture](model_architecture.md) | LSTM autopilot model architecture and features |
| [Model Evaluation Report](model_evaluation_report.md) | Current model performance and experiment results |
| [Model Export (ONNX)](model_export_onnx.md) | ONNX export for Raspberry Pi deployment |
| [MLflow Tracking](mlflow.md) | Experiment tracking setup and usage |
| [PD Controller](pd_controller.md) | Proportional-Derivative controller explanation |
| [Planned Passage Experiment](planned_passage_experiment.md) | How to run the passage following experiment |
| [Helm Controller Bug Fix](helm_controller_bug_fix.md) | Critical bug fix in simulation helm controller |
| [Feature Engineering Sign Convention](feature_engineering_sign_convention.md) | Sign convention alignment for error signals |
| [CAN Logger](can_logger.md) | Raw CAN frame datalogger with web UI and Pi deployment |

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
