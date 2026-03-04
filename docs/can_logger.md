# CAN Bus Raw Frame Datalogger

Records raw CAN frames from the NMEA2000 bus in a compact binary format for debugging, replay, and capturing proprietary PGNs that aren't yet decoded.

## Quick Start

```bash
# Run with defaults (can0, logs to logs/can/, no web UI)
uv run python -m src.n2k.can_logger

# Run with web UI
uv run python -m src.n2k.can_logger --web --web-port 8082

# Run with config file
uv run python -m src.n2k.can_logger --config config/can_logger.json --web

# All CLI options
uv run python -m src.n2k.can_logger \
    --config config/can_logger.json \
    --channel can0 \
    --log-dir /mnt/usbdrive/logs/can \
    --max-size 20 \
    --web --web-port 8082 --web-host 0.0.0.0 \
    --verbose
```

CLI flags override values in the config file.

## Web UI

When started with `--web`, a dashboard is available at `http://<pi-ip>:8082` with four tabs:

- **Dashboard** — message rate, frames logged/filtered, current file, disk usage bar, GPS sync status, PGN breakdown
- **Config** — edit include/exclude filters and rate limits, save to reload without restart
- **Log Files** — list files with size/frame count, download links
- **Live Monitor** — SSE-connected scrolling table of recent CAN frames with PGN names and hex data

## Configuration

Create a JSON config file (e.g. `config/can_logger.json`):

```json
{
    "channel": "can0",
    "log_dir": "/mnt/usbdrive/logs/can",
    "max_file_size_mb": 20,
    "max_disk_mb": 500,
    "include": [
        {"pgn": 130306},
        {"pgn": 127250, "source": 204}
    ],
    "exclude": [
        {"pgn": 127488},
        {"pgn": 127489}
    ],
    "rate_limits": [
        {"pgn": 130306, "max_hz": 2.0},
        {"pgn": 129029, "source": 42, "max_hz": 1.0}
    ],
    "gps_time_sync": true,
    "web_enabled": true,
    "web_port": 8082,
    "web_host": "0.0.0.0"
}
```

### Filter Logic

1. If `include` is non-empty, a frame must match an include entry (PGN, or PGN+source) to pass
2. If a frame matches an `exclude` entry, it is dropped (exclude overrides include)
3. Rate limits apply on top, keyed by `(pgn, source)` with PGN-only fallback

An empty `include` list means all PGNs pass (no whitelist). An empty `exclude` list means nothing is explicitly dropped.

### Hot Reload

The config file is checked every 2 seconds. Changes to filters and rate limits take effect without restarting the logger. The config editor in the web UI writes directly to the config file.

### Auto-Prune

When total log directory size exceeds `max_disk_mb`, the oldest `can_*.bin` files are deleted until the directory is under the limit. This is checked after each file rotation.

## Binary File Format

Designed for fast sequential writes and mmap-friendly random access.

### File Header (32 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | magic | `b'CANL'` |
| 4 | 4 | version | uint32 LE, currently `1` |
| 8 | 4 | flags | uint32 LE (bit 0 = extended IDs) |
| 12 | 4 | reserved | 0 |
| 16 | 8 | start_timestamp | float64 LE, Unix epoch of first frame |
| 24 | 8 | reserved | 0 |

### Frame Record (24 bytes, fixed-size)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | timestamp_offset | uint32 LE, 0.1ms units since start_timestamp |
| 4 | 4 | can_id | uint32 LE, full 29-bit CAN ID |
| 8 | 1 | dlc | uint8, data length code |
| 9 | 1 | flags | uint8 (bit 0=error, bit 1=remote) |
| 10 | 1 | source | uint8, extracted source address |
| 11 | 1 | priority | uint8 |
| 12 | 8 | data | CAN payload, zero-padded to 8 bytes |
| 20 | 4 | pgn | uint32 LE, extracted PGN |

To reconstruct absolute time: `abs_time = header.start_timestamp + (frame.timestamp_offset * 0.0001)`

### Storage Estimates

At ~100 msg/s typical NMEA2000 bus: ~2.4 KB/s, ~8.3 MB/hr. A 20 MB file holds ~2.4 hours. With `max_disk_mb=500`, that's ~60 hours of continuous logging.

### File Rotation

A new file is created when any of these conditions are met:

- File size reaches `max_file_size_mb`
- Date changes (midnight UTC)
- Timestamp offset would overflow uint32 (~4.97 days at 0.1ms resolution)

### Reading Log Files

Fixed-size records allow direct seeking by frame index:

```python
import struct

HEADER = struct.Struct('<4sIII d 8x')  # 32 bytes
FRAME = struct.Struct('<II BB BB 8s I')  # 24 bytes

with open('can_20260302_120000_000.bin', 'rb') as f:
    magic, version, flags, _, start_ts = HEADER.unpack(f.read(32))
    assert magic == b'CANL'

    while True:
        data = f.read(24)
        if len(data) < 24:
            break  # EOF or partial record (safe to ignore)
        ts_offset, can_id, dlc, flags, source, priority, payload, pgn = FRAME.unpack(data)
        abs_time = start_ts + ts_offset * 0.0001
        print(f"{abs_time:.4f} PGN {pgn} src={source} data={payload[:dlc].hex()}")
```

## GPS Time Sync

When `gps_time_sync` is enabled and the system year is < 2025 (indicating no RTC battery), the logger extracts UTC from PGN 126992 (System Time) or PGN 129033 (Time & Date) and sets the system clock via `timedatectl` or `date`. This runs once at startup regardless of include/exclude filters.

## Filesystem Safety

The logger is designed for unclean shutdown (power cut):

- **Periodic fsync** (every 5s) commits buffered writes from OS page cache to disk. At most ~5 seconds of frames are lost on power cut.
- **Fixed-size records** mean a truncated file is detectable: if `(filesize - 32) % 24 != 0`, the trailing partial record is incomplete and should be ignored. Every complete 24-byte record before that is valid.
- **fsync on rotation/close** ensures completed files are fully committed before starting a new one.
- **ext4 journaling** (default on Raspberry Pi OS) keeps filesystem metadata consistent through power cuts — no fsck needed on boot.

## Raspberry Pi Deployment

See [Pi Setup](pi_setup.md) for base OS setup. The CAN logger runs as a systemd service that starts on boot.

### Prerequisites

- USB-CAN adapter (CandleLite or similar gs_usb device) connected to NMEA2000 bus
- USB drive mounted at `/mnt/usbdrive` for log storage
- CAN interface configured (see setup files below)

### Install

```bash
# Copy service and network config files (requires disabling overlay FS temporarily)
sudo cp deploy/can-logger.service /etc/systemd/system/
sudo cp deploy/can0.network /etc/systemd/network/
sudo cp deploy/80-can.rules /etc/udev/rules.d/
sudo cp deploy/can0-up.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/can0-up.sh

# Create config directory on USB drive
mkdir -p /mnt/usbdrive/config
cp deploy/can_logger.json /mnt/usbdrive/config/

# Enable services
sudo systemctl daemon-reload
sudo systemctl enable systemd-networkd
sudo systemctl enable can-logger

# Reboot to verify auto-start
sudo reboot
```

### Management

```bash
# Service status
sudo systemctl status can-logger
journalctl -u can-logger -f

# Restart after config change (or let hot-reload handle it)
sudo systemctl restart can-logger

# Stop
sudo systemctl stop can-logger
```

### Deploy Directory

The `deploy/` directory contains all Pi deployment files:

| File | Destination | Purpose |
|------|-------------|---------|
| `can-logger.service` | `/etc/systemd/system/` | Systemd service unit |
| `80-can.rules` | `/etc/udev/rules.d/` | udev rule to bring up CAN on adapter plug |
| `can0-up.sh` | `/usr/local/bin/` | Script to configure CAN bitrate and bring up interface |
| `can0.network` | `/etc/systemd/network/` | systemd-networkd CAN link config |
| `can_logger.json` | `/mnt/usbdrive/config/` | Default logger config |
| `mnt-usbdrive.mount` | `/etc/systemd/system/` | USB drive mount unit |
