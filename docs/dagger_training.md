# DAgger Training: Closing the Sim-to-Real Loop

> **Experimental — did not succeed as a standalone approach.** DAgger was tested across 6 iterations in runs 26-10 and 26-11. It fixed compass mode perfectly (0.5 deg SS error) but made wind mode dramatically worse (49 deg -> 68 deg). The model learned to be passive in wind mode because DAgger-collected states had uniformly saturated expert labels. The production system now uses a **blended PD + ML controller** with residual labels instead. See **[blended_pd_ml_controller.md](blended_pd_ml_controller.md)** for the current approach and validated results. DAgger collection code remains in the codebase and may be useful in combination with the blended approach in future.

---

## Background: Why Behavioral Cloning Falls Short

The autopilot model is trained via imitation learning (behavioral cloning): the model observes what an expert PD controller does in simulated sailing scenarios and learns to reproduce those rudder commands from the same sensor inputs.

This approach works well in **open-loop** evaluation (predicting rudder for isolated frames from the training distribution) but consistently fails in **closed-loop** (CL) deployment where the model's own outputs steer the boat. The failure mode is well-documented in robotics literature as **distribution shift** or **compounding errors**.

### The Problem in Detail

In training, the model sees states visited by the expert. The expert's PD controller (kp=1.6, kd=1.5) steers smoothly, so the training data contains:
- Mostly steady-state sailing (heading error near zero, small rudder)
- Occasional large corrections during error recovery scenarios
- Clean transitions from error to recovery (the expert never overshoots badly)

In closed-loop deployment, the model's imperfect predictions cause small heading errors. These errors push the boat into states the expert never visited. The model, having never seen these states in training, makes poor predictions, causing larger errors, which lead to even more unfamiliar states. This is the compounding error problem.

### Evidence from Training Runs

Across 9 training runs (26-1 through 26-9), we consistently observed:

| Observation | Implication |
|-------------|-------------|
| Validation loss kept improving (best: 0.000964) | Model fits training distribution well |
| CL performance did NOT correlate with val loss | Closed-loop failure is distribution shift, not underfitting |
| Best val loss run often had worst CL performance | Better fit to expert states doesn't help with model-visited states |
| CL diagnostics showed oscillation and divergence | Model applies assertive rudder but can't dampen/settle |

Specific failure modes observed in CL diagnostics:

**Underdamped oscillation** (compass_large test): The model applies strong initial correction (+22 deg rudder for 90 deg error) but doesn't reduce rudder as heading rate builds. It overshoots, reverses with equally strong rudder, overshoots again, creating a limit cycle.

```
time  h_err   pd_sug   model   heading   (target: 135)
 0.0  +1.000  +1.000  +0.528    45.0    assertive start
 5.0  +0.808  +1.000  +0.882    62.3    strong push
10.0  +0.176  +0.457  +0.773   123.1    pd_sug says ease off, model ignores
20.0  -0.107  -0.312  -0.608   144.7    overshot! reverses hard
30.0  +0.166  +0.621  +0.497   120.1    oscillating
```

**Divergence** (wind_awa_hold test): Initial correction overshoots heading, AWA collapses, model can't recover. The boat enters states never seen in training and the model's outputs make things progressively worse.

### What We Tried (and Why It Wasn't Enough)

| Approach | Run | Outcome |
|----------|-----|---------|
| Mode flag (feature 1) | 26-4 | Best overall: 2/3 CL pass. Helped mode specialization. |
| More wind data (45% wind scenarios) | 26-5 | Regression. Diluted compass/recovery data. |
| Wind-specific error recovery scenarios | 26-6 | Regression. Less compass recovery data hurt. |
| Doubled heading_rate signal (max 30->15) | 26-7 | Worse. Model used amplified rate as another P-term, not for damping. |
| PD suggestion feature (feat 19) | 26-8 | compass_large PASS 2.2 deg (best ever), but compass_small broke. |
| Tack balance + feature 16 mirror fix | 26-9 | Fixed wind_awa sign bias, but still oscillates. |

The key insight: **no amount of data engineering, feature design, or normalization tuning can fix distribution shift**. The model needs to see states it actually visits during closed-loop operation.

## DAgger: Dataset Aggregation

DAgger is the standard solution for distribution shift in imitation learning (Ross et al., 2011). The algorithm iterates:

```
1. Train initial policy pi_1 on expert demonstrations D
2. For iteration i = 1, 2, ..., N:
   a. Run pi_i in the environment, collecting states S_i
   b. For each state s in S_i, query the expert for action a* = expert(s)
   c. Aggregate: D = D + {(s, a*)}
   d. Train pi_{i+1} on aggregated D
```

The key difference from plain behavioral cloning: the model is trained on states it actually encounters under its own policy, not just states the expert encounters. After 2-3 iterations, the training distribution converges to the model's deployment distribution.

### How It Works for the Autopilot

1. **Train initial model** on simulated expert data (existing pipeline)
2. **Run the model** in closed-loop simulation across diverse scenarios
3. At each inference step, the **model steers** (collecting distribution-shifted states)
4. At the same step, query the **expert PD controller** for what it would do in that state
5. Write (model-visited state, expert action) pairs as additional training data
6. **Retrain** on original data + DAgger data (model resumes from checkpoint)
7. Repeat 2-6 until CL validation passes

The physics simulation, feature computation, and binary data format are identical to the existing pipeline. The only difference is who steers during data collection (the model) vs who provides the label (the expert).

## Preparing Synthetic Training Data

### Step 1: Generate Base Expert Data

Generate the initial training dataset using the standard scenario mix:

```bash
.venv/bin/python -u -m src.simulation.data_generator data/simulated --format binary 2>&1 | tee training-gen.txt
```

The default scenario mix (defined in `data_generator.py`) is:
- 3 wind scenarios: medium_upwind, downwind_vmg, light_air_reaching
- 3 compass scenarios: calm_compass, motoring, mixed_coastal
- 3 error recovery: random mode, 30-90 deg initial errors

This produces ~1.8M training sequences with mirror augmentation (port/starboard doubling).

#### Data Quality Checklist

Before training, verify:
- Wind scenarios have balanced tack (target_angle randomized +/-) via `randomize_initial_conditions()`
- Error recovery scenarios include all three modes (compass, wind_awa, wind_twa)
- Wave model is active in all scenarios (provides wave_period feature)
- Rate features are populated (roll_rate, awa_rate, rudder_velocity)

### Step 2: Initial Training

Train the base model:

```bash
.venv/bin/python -u -m src.training.train_imitation data/simulated -o models 2>&1 | tee training-base.txt
```

This trains until early stopping (typically 15-20 epochs, ~20-30 minutes on Apple Silicon MPS). The model checkpoint is saved automatically.

### Step 3: DAgger Data Collection

Run the trained model in closed-loop and collect expert-relabelled data:

```bash
.venv/bin/python -u -m src.training.train_imitation --dagger models/autopilot.onnx --dagger-output data/simulated 2>&1 | tee dagger-collect.txt
```

This runs the model through ~100 diverse scenarios:
- ~40 compass scenarios (varied target errors 5-90 deg, TWS 8-25 kts)
- ~40 wind_awa scenarios (target AWA +/-30-90 deg, balanced tack, TWS 8-25 kts)
- ~20 wind_twa scenarios (target TWA +/-90-170 deg, TWS 10-25 kts)

Each scenario runs for 120 seconds at 2 Hz inference, producing ~24,000 DAgger frames per iteration. These frames are high-value because they cover exactly the states where the model struggles.

The expert controller used for relabelling has:
- Same PD gains as training (kp=1.6, kd=1.5)
- Perfect execution: skill_level=1.0, no noise, no fatigue, no reaction delay
- Physical rate limiting preserved (4 deg/s, matching real actuator)

### Step 4: Retrain with Aggregated Data

Retrain the model on original + DAgger data:

```bash
.venv/bin/python -u -m src.training.train_imitation data/simulated -o models 2>&1 | tee training-dagger1.txt
```

The trainer automatically loads all `.bin` files in the data directory (original expert data + DAgger files). The model resumes from checkpoint, so it starts from the previous best weights rather than random initialization.

### Step 5: Iterate

Repeat steps 3-4 two or three times. Each iteration:
- Collects new DAgger data under the current (improved) model's policy
- Adds to the growing aggregated dataset
- Retrains from checkpoint

Typically 2-3 iterations suffice. Monitor CL validation results after each training run. Stop when all three CL tests pass:
- compass_small: <5 deg steady-state error
- compass_large: <10 deg steady-state error
- wind_awa_hold: <10 deg steady-state error

### Combined Workflow

For convenience, the training script supports running DAgger iterations automatically:

```bash
# Train with 3 DAgger iterations
.venv/bin/python -u -m src.training.train_imitation data/simulated -o models --dagger 3 2>&1 | tee training-full.txt
```

This performs: initial training -> DAgger collect -> retrain -> DAgger collect -> retrain -> DAgger collect -> retrain, with CL validation after each cycle.

## CL Validation and Diagnostics

After each training run, closed-loop validation automatically runs three scenarios:

| Test | Setup | Pass Criterion |
|------|-------|----------------|
| compass_small | 5 deg heading error, 120s, TWS 12 kts | <5 deg steady-state error |
| compass_large | 90 deg heading error, 60s, TWS 12 kts | <10 deg steady-state error |
| wind_awa_hold | AWA 10 deg off target, 120s, TWS 15 kts | <10 deg steady-state error |

Diagnostics print at key timesteps showing:
- `h_err_f`: heading error feature (normalized, +/-1.0 = +/-90 deg)
- `pd_sug`: PD suggestion feature (what the PD controller would do)
- `rud_n`: model's normalized rudder output
- `rud_deg`: rudder in degrees
- `AWA`, `heading`: current boat state

**What to look for in diagnostics:**

| Pattern | Meaning | Expected After DAgger |
|---------|---------|----------------------|
| Model output tracks pd_sug | Model follows expert damping | Yes |
| Model overshoots then oscillates | Underdamped, no settling | Should reduce with iterations |
| Model output opposite sign to pd_sug | Wrong correction direction | Should disappear |
| Growing oscillation amplitude | Unstable, diverging | Should converge to stable |

## Capturing Real Data for Training

When real sailing data becomes available, it can significantly improve model quality by covering conditions the simulator doesn't model (current, complex sea states, sail trim effects, real sensor noise).

### NMEA2000 Data Requirements

The model requires these signals at 2 Hz or faster:

| Signal | PGN | Required | Notes |
|--------|-----|----------|-------|
| Heading (magnetic) | 127250 | Yes | Primary control signal |
| Rate of Turn | 127251 | Yes | Heading rate for damping |
| Rudder Angle | 127245 | Yes | Training label (what the helm did) |
| AWA | 130306 | Yes | Wind angle for wind modes |
| AWS | 130306 | Yes | Wind speed for wind triangle |
| TWA | 130306 | Preferred | Computed from AWA/AWS/STW if absent |
| TWS | 130306 | Preferred | Computed if absent |
| STW | 128259 | Yes | Boat speed for wind triangle |
| SOG | 129026 | Yes | Ground speed |
| COG | 129026 | Yes | Course over ground |
| Roll (heel) | 127257 | Preferred | From IMU if available |
| Pitch | 127257 | Preferred | From IMU if available |

### Recording Setup

1. **CAN bus logger**: Record all NMEA2000 traffic using candump, Actisense NGT-1, or similar
2. **Sample rate**: 10 Hz minimum on heading/rudder/rate-of-turn, 2 Hz minimum on wind/speed
3. **Duration**: Aim for 1+ hours per session across varied conditions
4. **Metadata**: Note the autopilot mode (compass/AWA/TWA) and target values for each segment

### Conditions to Capture

For DAgger-enhanced training, real data is most valuable when it covers:

| Priority | Condition | Why |
|----------|-----------|-----|
| High | Moderate wind (10-20 kts) upwind | Most common autopilot use case |
| High | Wind shifts and gusts | Tests adaptation, not covered well in simulation |
| High | Beam reach in waves | Cross-sea motion challenges the model |
| Medium | Light air (<8 kts) | Different dynamics, rudder less effective |
| Medium | Heavy weather (>20 kts) | Safety-critical, high heel/waves |
| Medium | Motoring in calm | Baseline compass-hold behavior |
| Low | Downwind VMG | Less common for cruising autopilot |

### Data Quality Considerations

**Good data:**
- Steady-state sailing with occasional corrections (the model learns normal helming)
- Recovery from disturbances (wave knockdowns, wind shifts)
- Clean mode transitions with known targets
- Consistent sensor readings (no dropouts or spikes)

**Bad data:**
- Excessive manual override during autopilot operation (conflicting labels)
- Sensor faults (heading jumps, wind sensor failures)
- Berthing/maneuvering under engine (not relevant to open-water autopilot)
- Periods where crew is adjusting sails (rudder movements are sail-trim artifacts)

### Preparing Real Data for Training

1. Organize logs:
   ```bash
   .venv/bin/python -m src.n2k.log_organizer /path/to/raw/logs --execute
   ```

2. Analyze and generate metadata:
   ```bash
   .venv/bin/python -m src.training.log_analyzer /path/to/raw/logs
   ```

3. Verify the data loads:
   ```bash
   .venv/bin/python -c "
   from src.training.data_loader import TrainingDataLoader
   loader = TrainingDataLoader()
   X, y = loader.load_directory('/path/to/logs')
   print(f'Sequences: {len(X):,}, shape: {X.shape}')
   "
   ```

### Mixing Real and Simulated Data

For best results, combine simulated expert data, DAgger data, and real data:

```
data/training/
  simulated_*.bin          # Base expert demonstrations (1.8M sequences)
  dagger_iter1.bin         # DAgger iteration 1 (~24K sequences)
  dagger_iter2.bin         # DAgger iteration 2 (~24K sequences)
  real_sailing_*.bin       # Real logged data (converted to binary)
```

The trainer loads all `.bin` files from the data directory and splits them into train/validation per-file. Real data acts as a natural regularizer against simulator artifacts.

### Special Considerations for Real Data with DAgger

When using DAgger with real data:

1. **The expert controller defines "correct" behavior.** In simulation, the PD controller is the expert. For real data, the human helm is the expert. These may differ. The model learns a blend.

2. **Real data doesn't need DAgger relabelling.** DAgger only applies to data collected under the model's own policy. Real data collected under human helming is already expert-labelled.

3. **Feature parity matters.** Real data must produce the same 22-feature vector as simulated data. The `compute_features()` function in `data_loader.py` is the single source of truth. Ensure all required sensor signals are present.

4. **Rate features need consecutive frames.** Features 5 (roll_rate), 7 (AWA_rate), and 15 (rudder_velocity) are computed from consecutive frames. Real data must have consistent timestamps for accurate rate computation.

5. **Wave period from real data.** Feature 21 (wave_period) is estimated from accelerometer FFT in the real-time system (`feature_engineering.py`). If not available from sensors, it can be set to 0 — the model will still function but may be less accurate in heavy seas.

## Architecture Notes

### Feature Vector (22 features)

| Index | Feature | Normalization | Mirror |
|-------|---------|---------------|--------|
| 0 | heading_error | /90 deg | Negate |
| 1 | mode_flag | compass=0.0, awa=0.5, twa=1.0 | Keep |
| 2 | heading_rate | /30 deg/s | Negate |
| 3 | roll | /45 deg | Negate |
| 4 | pitch | /30 deg | Keep |
| 5 | roll_rate | /30 deg/s | Negate |
| 6 | AWA | /180 deg | Negate |
| 7 | AWA_rate | /10 deg/s | Negate |
| 8 | AWS | /60 kts | Keep |
| 9 | TWA | /180 deg | Negate |
| 10 | TWS | /60 kts | Keep |
| 11 | STW | /25 kts | Keep |
| 12 | SOG | /25 kts | Keep |
| 13 | COG_error | /45 deg | Negate |
| 14 | rudder_position | zeroed | Negate |
| 15 | rudder_velocity | /10 deg/s | Negate |
| 16 | computed_heading | /360 | Zeroed in mirror |
| 17 | VMG_up | /15 kts | Keep |
| 18 | VMG_down | /20 kts | Keep |
| 19 | PD_suggestion | (1.6*err - 1.5*rate)/25 | Negate |
| 20 | placeholder | 0.0 | Keep |
| 21 | wave_period | /15 s | Keep |

### Model Architecture

```
Input (20 timesteps x 22 features)
  -> Feature Mix: Linear(22, 256) + ReLU
  -> BatchNorm1d(20)
  -> LSTM(256, 128)
  -> Dropout(0.3)
  -> LSTM(128, 64)
  -> Dropout(0.3)
  -> Linear(64, 32) + LeakyReLU
  -> Linear(32, 1) + Tanh
Output: rudder normalized to [-1, 1] (maps to +/-25 deg)
```

Parameters: 255,337 (~1 MB). Deploys as ONNX on Raspberry Pi 4.

## Fallback: Residual on PD Controller

If DAgger does not deliver the expected CL results after 3-4 iterations, the next approach is to change the model from predicting absolute rudder to predicting a small correction on top of the PD controller output.

**Current approach:**
```
rudder = model(features) * 25.0
```

**Residual approach:**
```
rudder = (pd_suggestion + model(features) * scale) * 25.0
```

Implementation:
- Change training labels from `expert_rudder` to `expert_rudder - pd_suggestion`
- PD suggestion is already feature 19 in the feature vector
- At inference: add the model's output (residual correction) to the PD suggestion
- No architecture change needed, only label transformation and inference-time addition
- The PD controller handles the bulk of the steering (including damping), and the model learns small condition-specific adjustments

This guarantees PD-like damped behavior as a baseline, which directly addresses the oscillation problem. The model only needs to learn residuals near zero for most conditions.
