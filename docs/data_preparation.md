# Training Data Preparation Guide

This guide covers the complete workflow for preparing CAN log data for autopilot model training.

## Overview

The data preparation process consists of two main steps:

1. **Log Organization** - Organize raw log files into a date-ordered directory structure
2. **Log Analysis** - Analyze logs to detect operation modes and generate training metadata

```
Raw Logs → [Organize by Date] → [Analyze & Label] → Training-Ready Data
```

## Prerequisites

Ensure you have the project dependencies installed:

```bash
cd autopilot
uv sync
```

## Step 1: Log Organization

The log organizer parses CAN log files, extracts capture dates, and organizes them into a `YYYY/MM/` directory structure.

### Supported Log Formats

| Format | Description | Date Source |
|--------|-------------|-------------|
| **candump** | Standard candump with Unix timestamps | Timestamp in file |
| **analyzed CSV** | Human-readable CSV with datetime | Datetime string |
| **hardware_ts** | Hardware timestamp with packed frames | Filename or GPS PGN |
| **slcan** | slcan candump format | Unix timestamp |
| **raymarine** | Raymarine MFD native format | GPS PGN messages |

### Usage

#### Dry Run (Preview)

First, preview what changes will be made without moving any files:

```bash
uv run python -m src.n2k.log_organizer /path/to/raw/logs
```

Example output:
```
DRY RUN - No files will be moved

Source File                                   Format          Date         Destination
------------------------------------------------------------------------------------------------------------------------
candump-2018-05-07-15-18.log                  candump         2018-05-12   2018/05/candump-2018-05-07-15-18.log
N2K_000001.log                                raymarine       2023-05-14   2023/05/N2K_000001.log
capture20240411.txt                           hardware_ts     2024-04-11   2024/04/capture20240411.txt

Summary:
  Total files: 3
  By format:
    candump: 1
    raymarine: 1
    hardware_ts: 1

To execute, run with --execute flag
```

#### Execute Organization

Once satisfied with the preview, execute the organization:

```bash
uv run python -m src.n2k.log_organizer /path/to/raw/logs --execute
```

#### Custom Destination

To organize files into a different directory:

```bash
uv run python -m src.n2k.log_organizer /path/to/raw/logs --dest-dir /path/to/organized --execute
```

### Output Structure

After organization, logs will be arranged as:

```
n2klogs/raw/
├── 2017/
│   └── 11/
│       └── candump-2017-11-19_105919.log
├── 2018/
│   └── 05/
│       ├── candump-2018-05-07-15-18.log
│       └── candump-2018-05-19-1.log
├── 2024/
│   └── 04/
│       └── capture20240411.txt
└── unknown/
    └── testdump.txt  # Files without determinable dates
```

## Step 2: Log Analysis

The log analyzer examines organized logs to detect:
- **Operation mode**: anchor, motoring, sailing, or unknown
- **Steering mode**: heading, AWA, TWA, or none
- **Feature coverage**: which sensor data is available

It generates `.meta.json` files that the training data loader uses to set correct labels.

### Usage

#### Analyze a Single File

```bash
uv run python -m src.training.log_analyzer n2klogs/raw/2018/05/candump-2018-05-07-15-18.log
```

#### Analyze a Directory

```bash
uv run python -m src.training.log_analyzer n2klogs/raw/ --recursive
```

#### Dry Run (Preview Only)

To see analysis results without creating metadata files:

```bash
uv run python -m src.training.log_analyzer n2klogs/raw/ --dry-run
```

#### Verbose Output

For detailed logging during analysis:

```bash
uv run python -m src.training.log_analyzer n2klogs/raw/ --verbose
```

### Understanding the Output

#### Single File Analysis

```
======================================================================
FILE ANALYSIS: candump-2018-05-19-1.log
======================================================================

--- Overview ---
Total duration:  0.0823 hours (296.2 sec)
Total frames:    2,797
Segments found:  3

--- Usable Data ---
Usable duration: 0.1317 hours (474.0 sec)
Usable frames:   3,375
Usable segments: 3/3
Training records estimate: 3,355

--- Feature Coverage ---
Feature           Coverage       Status
--------------- ---------- ------------
heading             100.0%           OK
yaw_rate            100.0%           OK
roll                100.0%           OK
stw                  99.7%           OK
rudder_angle        100.0%           OK

--- Segments ---

[1] MOTORING / HEADING [USABLE]
    Duration:    0.0336 hours (121.0 sec)
    Frames:      1,144
    Target:      154.7° (confidence: 78%)

[2] SAILING / HEADING [USABLE]
    Duration:    0.0808 hours (291.0 sec)
    Frames:      2,210
    Target:      153.7° (confidence: 98%)

--- Summary ---
Usable for training: YES
======================================================================
```

#### Directory Summary

```
======================================================================
DIRECTORY ANALYSIS SUMMARY
======================================================================
Files analyzed: 8

--- Data Volume ---
Total log duration:    0.70 hours
Usable for training:   0.13 hours (18.9%)
Total frames:          23,436
Usable frames:         3,375
Training records est:  3,215

--- Feature Coverage (across all files) ---
Feature           Coverage       Status
--------------- ---------- ------------
heading              87.5%           OK
stw                  11.9%      MISSING <--
======================================================================
```

### Metadata Files

For each analyzed log file, a `.meta.json` file is created:

```
candump-2018-05-19-1.log
candump-2018-05-19-1.meta.json  ← Generated
```

Example metadata content:

```json
{
  "source_file": "candump-2018-05-19-1.log",
  "analyzed_at": "2026-01-23T12:00:00Z",
  "total_duration_hours": 0.0823,
  "usable_duration_hours": 0.1317,
  "usable_frame_count": 3375,
  "segments": [
    {
      "start_time": 1526742000.0,
      "end_time": 1526742121.0,
      "operation_mode": "motoring",
      "steering_mode": "heading",
      "target_value": 154.7,
      "confidence": 0.78,
      "frame_count": 1144,
      "usable_for_training": true
    }
  ],
  "summary": {
    "usable_for_training": true,
    "training_records_estimate": 3355
  }
}
```

### Interactive Review Mode

For manual review and adjustment of detected segments:

```bash
uv run python -m src.training.log_analyzer candump.log --interactive
```

Commands:
- `n` / `next` - Go to next segment
- `p` / `prev` - Go to previous segment
- `e` / `edit` - Edit current segment
- `d` / `delete` - Delete current segment
- `s` / `save` - Save changes
- `q` / `quit` - Exit

## Operation Mode Detection

The analyzer uses these heuristics:

| Mode | Detection Criteria |
|------|-------------------|
| **anchor** | SOG < 0.5 kn, position within 50m radius |
| **motoring** | Engine RPM > 100, OR TWS < 6 kn, OR AWA < 20°, OR flat (no heel) |
| **sailing** | Heel angle > 5°, OR good wind with valid sailing angle |

## Steering Mode Detection

| Mode | Detection Criteria |
|------|-------------------|
| **heading** | Heading stable (σ < 5°) over analysis window |
| **awa** | AWA stable (σ < 5°) while heading varies, typically TWA < 130° |
| **twa** | TWA stable (σ < 8°) while heading/AWA vary, typically TWA ≥ 130° |

## Required Features for Training

The following features must have >50% coverage for usable training data:

| Feature | Description | PGN Source |
|---------|-------------|------------|
| heading | Vessel heading | 127250 |
| yaw_rate | Rate of turn | 127251 |
| roll | Heel angle | 127257 |
| pitch | Pitch angle | 127257 |
| awa | Apparent wind angle | 130306 |
| aws | Apparent wind speed | 130306 |
| stw | Speed through water | 128259 |
| sog | Speed over ground | 129026 |
| cog | Course over ground | 129026 |
| rudder_angle | Rudder position | 127245 |

## Complete Workflow Example

```bash
# 1. Organize logs by date
uv run python -m src.n2k.log_organizer n2klogs/raw/ --execute

# 2. Analyze logs and generate metadata (dry run first)
uv run python -m src.training.log_analyzer n2klogs/raw/ --dry-run

# 3. Review output, then execute
uv run python -m src.training.log_analyzer n2klogs/raw/

# 4. Check summary
uv run python -m src.training.log_analyzer n2klogs/raw/ --dry-run 2>&1 | head -40

# 5. Training data is now ready - metadata files alongside logs
ls n2klogs/raw/2018/05/*.meta.json
```

## Troubleshooting

### "No frames parsed"

The log format may not be recognized. Check:
- File is not empty
- File contains valid CAN data
- Format matches one of the supported types

### Low Feature Coverage

If required features show "MISSING":
- Check that the boat's sensors were transmitting on NMEA2000
- Some features (like STW) require specific instruments
- Consider whether the missing data is critical for your use case

### Segments Marked as "skip"

Segments are skipped when:
- Operation mode is "anchor" or "unknown"
- Steering mode is "none"
- Critical features (heading, awa, aws, stw) have low coverage

## Next Steps

After data preparation:

1. Review the generated metadata files
2. Optionally use `--interactive` mode to refine segment labels
3. Proceed to model training using the prepared data

See `src/training/train_imitation.py` for the training workflow.
