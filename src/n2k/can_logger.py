"""
CAN Bus Raw Frame Datalogger
=============================

Records raw CAN frames from the NMEA2000 bus in a compact binary format.
Supports filtering, rate limiting, GPS time sync, auto-prune, and a web UI.

Binary format: 32-byte file header + 24-byte fixed-size frame records.
At ~100 msg/s → ~2.4 KB/s → ~8.3 MB/hr.

Usage:
    uv run python -m src.n2k.can_logger --channel can0 --log-dir logs/can --web
"""

import argparse
import collections
import json
import logging
import os
import signal
import struct
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import can
except ImportError:
    can = None

logger = logging.getLogger(__name__)

# --- Binary format constants ---

FILE_MAGIC = b'CANL'
FILE_VERSION = 1
HEADER_SIZE = 32
FRAME_SIZE = 24
HEADER_STRUCT = struct.Struct('<4sIII d 8x')  # magic, version, flags, reserved, start_ts, pad
FRAME_STRUCT = struct.Struct('<II BB BB 8s I')  # ts_offset, can_id, dlc, flags, source, priority, data, pgn

# Time offset is uint32 in units of 0.1ms (100 μs).
# Overflow at 2^32 × 0.1ms ≈ 4.97 days — size rotation will always trigger first.
TIMESTAMP_UNIT_S = 0.0001  # 0.1 ms
MAX_TIMESTAMP_OFFSET = 0xFFFFFFFF

# --- PGN name lookup (from nmea2000_interface.py PGN enum + common extras) ---

PGN_NAMES: Dict[int, str] = {
    127237: "Heading/Track Control",
    127245: "Rudder",
    127250: "Vessel Heading",
    127251: "Rate of Turn",
    127257: "Attitude",
    127488: "Engine Rapid",
    127489: "Engine Dynamic",
    128259: "Speed",
    129025: "Position Rapid",
    129026: "COG/SOG",
    129029: "GNSS Position",
    129033: "Time & Date",
    129539: "GNSS DOPs",
    130306: "Wind Data",
    130310: "Env Parameters",
    130311: "Env Parameters 2",
    126992: "System Time",
    126996: "Product Info",
    59392: "ISO Acknowledgement",
    59904: "ISO Request",
    60928: "ISO Address Claim",
    65311: "Seatalk Alarm",
    65345: "Seatalk Wind Datum",
    65359: "Seatalk Pilot Heading",
    65360: "Seatalk Pilot Locked Heading",
    65362: "Seatalk Pilot Speed",
    65379: "Seatalk Pilot Mode",
    65384: "Seatalk Pilot Keypad Heartbeat",
    126208: "NMEA Command/Request/Ack",
    126464: "PGN List",
    126720: "Raymarine Proprietary",
    126993: "Heartbeat",
    127258: "Magnetic Variation",
    127506: "DC Detailed Status",
    127508: "Battery Status",
    127513: "Battery Configuration",
    128267: "Water Depth",
    128275: "Distance Log",
    129038: "AIS Class A Position",
    129039: "AIS Class B Position",
    129040: "AIS Class B Extended",
    129041: "AIS AtoN Report",
    129044: "Datum",
    129283: "Cross Track Error",
    129540: "GNSS Satellites in View",
    129793: "AIS UTC and Date",
    129794: "AIS Class A Static",
    129797: "AIS Binary Broadcast",
    129809: "AIS Class B Static A",
    129810: "AIS Class B Static B",
    130312: "Temperature",
    130313: "Humidity",
    130314: "Actual Pressure",
    130316: "Temperature Extended",
    130822: "Raymarine Proprietary 2",
    130846: "Raymarine Proprietary 4",
    130916: "Raymarine Proprietary 3",
}

# PGNs used for GPS time sync
PGN_SYSTEM_TIME = 126992
PGN_TIME_DATE = 129033


def extract_pgn_source_priority(can_id: int) -> Tuple[int, int, int]:
    """Extract PGN, source address, and priority from a 29-bit CAN ID.

    NMEA2000 uses ISO 11783 addressing:
    - Bits 28-26: priority (3 bits)
    - Bit 25: reserved
    - Bit 24: data page
    - Bits 23-16: PF (PDU format)
    - Bits 15-8: PS (PDU specific) — destination (PDU1) or group ext (PDU2)
    - Bits 7-0: source address
    """
    source = can_id & 0xFF
    priority = (can_id >> 26) & 0x07
    pdu_format = (can_id >> 16) & 0xFF
    pdu_specific = (can_id >> 8) & 0xFF

    if pdu_format < 240:
        pgn = pdu_format << 8  # PDU1: PS is destination, not part of PGN
    else:
        pgn = (pdu_format << 8) | pdu_specific  # PDU2: PS is group extension

    # Data page bit
    if (can_id >> 24) & 0x01:
        pgn |= 0x10000

    return pgn, source, priority


# --- Configuration ---


@dataclass
class CANLoggerConfig:
    """Configuration for the CAN frame logger."""
    channel: str = "can0"
    log_dir: str = "logs/can"
    max_file_size_mb: float = 20.0
    max_disk_mb: float = 500.0
    include: List[Dict[str, int]] = field(default_factory=list)
    exclude: List[Dict[str, int]] = field(default_factory=list)
    rate_limits: List[Dict[str, Any]] = field(default_factory=list)
    gps_time_sync: bool = True
    web_enabled: bool = False
    web_port: int = 8082
    web_host: str = "0.0.0.0"

    @classmethod
    def from_json(cls, path: str) -> "CANLoggerConfig":
        """Load config from JSON file, ignoring unknown keys."""
        with open(path) as f:
            data = json.load(f)
        known = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in data.items() if k in known}
        return cls(**filtered)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dict for JSON output."""
        return {
            "channel": self.channel,
            "log_dir": self.log_dir,
            "max_file_size_mb": self.max_file_size_mb,
            "max_disk_mb": self.max_disk_mb,
            "include": self.include,
            "exclude": self.exclude,
            "rate_limits": self.rate_limits,
            "gps_time_sync": self.gps_time_sync,
            "web_enabled": self.web_enabled,
            "web_port": self.web_port,
            "web_host": self.web_host,
        }


# --- Frame filter ---


class FrameFilter:
    """Filters CAN frames by PGN include/exclude lists and rate limits."""

    def __init__(self, config: CANLoggerConfig):
        self._last_sent: Dict[Tuple[int, int], float] = {}
        self._stats = {"passed": 0, "excluded": 0, "rate_limited": 0, "no_include_match": 0}
        self.update(config)

    def update(self, config: CANLoggerConfig) -> None:
        """Rebuild filter tables from config, preserving rate-limit timestamps."""
        # Include sets
        self._include_pgns: set = set()
        self._include_pairs: set = set()
        for entry in config.include:
            pgn = entry.get("pgn")
            source = entry.get("source")
            if pgn is not None and source is not None:
                self._include_pairs.add((pgn, source))
            elif pgn is not None:
                self._include_pgns.add(pgn)

        self._has_include = bool(self._include_pgns or self._include_pairs)

        # Exclude sets
        self._exclude_pgns: set = set()
        self._exclude_pairs: set = set()
        for entry in config.exclude:
            pgn = entry.get("pgn")
            source = entry.get("source")
            if pgn is not None and source is not None:
                self._exclude_pairs.add((pgn, source))
            elif pgn is not None:
                self._exclude_pgns.add(pgn)

        # Rate limits: pair-specific takes priority, then PGN-only fallback
        self._rate_limit_pair: Dict[Tuple[int, int], float] = {}
        self._rate_limit_pgn: Dict[int, float] = {}
        for entry in config.rate_limits:
            pgn = entry.get("pgn")
            source = entry.get("source")
            max_hz = entry.get("max_hz", 0)
            if max_hz <= 0:
                continue
            interval = 1.0 / max_hz
            if pgn is not None and source is not None:
                self._rate_limit_pair[(pgn, source)] = interval
            elif pgn is not None:
                self._rate_limit_pgn[pgn] = interval

    @property
    def stats(self) -> Dict[str, int]:
        return dict(self._stats)

    def should_log(self, pgn: int, source: int, timestamp: float) -> bool:
        """Check if a frame should be logged.

        Order: include check → exclude check → rate limit check.
        """
        # Include filter
        if self._has_include:
            if pgn not in self._include_pgns and (pgn, source) not in self._include_pairs:
                self._stats["no_include_match"] += 1
                return False

        # Exclude filter (overrides include)
        if pgn in self._exclude_pgns or (pgn, source) in self._exclude_pairs:
            self._stats["excluded"] += 1
            return False

        # Rate limiting
        key = (pgn, source)
        interval = self._rate_limit_pair.get(key)
        if interval is None:
            interval = self._rate_limit_pgn.get(pgn)

        if interval is not None:
            last = self._last_sent.get(key, 0.0)
            if timestamp - last < interval:
                self._stats["rate_limited"] += 1
                return False
            self._last_sent[key] = timestamp

        self._stats["passed"] += 1
        return True


# --- Binary log writer ---


class BinaryLogWriter:
    """Writes CAN frames to binary log files with rotation and pruning."""

    def __init__(self, log_dir: str, max_file_size_bytes: int, fsync_interval: float = 5.0):
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._max_file_size = max_file_size_bytes
        self._file = None
        self._start_timestamp: float = 0.0
        self._current_date: str = ""
        self._frame_count: int = 0
        self._file_frame_count: int = 0
        self._total_files: int = 0
        self._fsync_interval = fsync_interval
        self._last_fsync: float = 0.0
        self._total_bytes: int = 0
        self._current_file_path: Optional[Path] = None
        self._lock = threading.Lock()

    def _open_new_file(self, timestamp: float) -> None:
        """Open a new log file and write the header."""
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()

        self._start_timestamp = timestamp
        now = datetime.fromtimestamp(timestamp)
        self._current_date = now.strftime("%Y%m%d")
        filename = f"can_{now.strftime('%Y%m%d_%H%M%S')}_{now.strftime('%f')[:3]}.bin"
        self._current_file_path = self._log_dir / filename
        self._file = open(self._current_file_path, "wb")

        # Write header
        header = HEADER_STRUCT.pack(
            FILE_MAGIC,
            FILE_VERSION,
            1,  # flags: bit 0 = extended IDs
            0,  # reserved
            timestamp,
        )
        self._file.write(header)
        self._file_frame_count = 0
        self._total_files += 1
        self._total_bytes += HEADER_SIZE
        logger.info("Opened log file: %s", self._current_file_path.name)

    def _needs_rotation(self, timestamp: float, timestamp_us: int) -> bool:
        """Check if we need to rotate to a new file."""
        if self._file is None:
            return True

        # Size limit
        current_size = HEADER_SIZE + self._file_frame_count * FRAME_SIZE
        if current_size >= self._max_file_size:
            return True

        # Date change
        now_date = datetime.fromtimestamp(timestamp).strftime("%Y%m%d")
        if now_date != self._current_date:
            return True

        # Timestamp offset overflow (0.1ms units, ~5 days)
        if timestamp_us > MAX_TIMESTAMP_OFFSET:
            return True

        return False

    def write_frame(self, timestamp: float, can_id: int, dlc: int,
                    data: bytes, pgn: int, source: int, priority: int) -> None:
        """Write a single frame record. Handles rotation."""
        elapsed = int((timestamp - self._start_timestamp) / TIMESTAMP_UNIT_S)

        with self._lock:
            if self._needs_rotation(timestamp, elapsed):
                self._open_new_file(timestamp)
                elapsed = 0

            flags = 0
            padded_data = (data + b'\x00' * 8)[:8]

            record = FRAME_STRUCT.pack(
                elapsed,
                can_id,
                dlc,
                flags,
                source,
                priority,
                padded_data,
                pgn,
            )
            self._file.write(record)
            self._file.flush()

            # Periodic fsync to survive hard power-off. flush() only pushes
            # to the OS page cache; fsync() commits to storage. We don't
            # fsync per-frame to avoid excessive SD card wear.
            now = time.time()
            if now - self._last_fsync >= self._fsync_interval:
                os.fsync(self._file.fileno())
                self._last_fsync = now

            self._file_frame_count += 1
            self._frame_count += 1
            self._total_bytes += FRAME_SIZE

    def close(self) -> None:
        """Close the current file, fsyncing first."""
        with self._lock:
            if self._file is not None:
                self._file.flush()
                os.fsync(self._file.fileno())
                self._file.close()
                self._file = None

    def list_files(self) -> List[Dict[str, Any]]:
        """List log files with metadata."""
        files = []
        for p in sorted(self._log_dir.glob("can_*.bin")):
            size = p.stat().st_size
            frames = max(0, (size - HEADER_SIZE)) // FRAME_SIZE if size >= HEADER_SIZE else 0
            files.append({
                "name": p.name,
                "size": size,
                "frames": frames,
                "modified": p.stat().st_mtime,
            })
        return files

    def prune(self, max_disk_bytes: int) -> int:
        """Delete oldest log files until total size is under limit. Returns files deleted."""
        files = sorted(self._log_dir.glob("can_*.bin"), key=lambda p: p.stat().st_mtime)
        total = sum(p.stat().st_size for p in files)
        deleted = 0

        while total > max_disk_bytes and files:
            oldest = files.pop(0)
            # Don't delete the file we're currently writing to
            if self._current_file_path and oldest.resolve() == self._current_file_path.resolve():
                continue
            size = oldest.stat().st_size
            oldest.unlink()
            total -= size
            deleted += 1
            logger.info("Pruned old log: %s (%d bytes)", oldest.name, size)

        return deleted

    @property
    def stats(self) -> Dict[str, Any]:
        current_size = 0
        current_name = None
        if self._current_file_path and self._current_file_path.exists():
            current_size = self._current_file_path.stat().st_size
            current_name = self._current_file_path.name
        return {
            "total_frames": self._frame_count,
            "total_files": self._total_files,
            "total_bytes": self._total_bytes,
            "current_file": current_name,
            "current_file_size": current_size,
            "current_file_frames": self._file_frame_count,
        }


# --- GPS time sync ---


class GPSTimeSync:
    """Synchronizes system time from GPS PGNs on systems without RTC."""

    def __init__(self, enabled: bool = True):
        self._enabled = enabled
        self._synced = False
        self._sync_time: Optional[float] = None

    def needs_sync(self) -> bool:
        """True if time sync is needed (year < 2025 suggests no RTC)."""
        if not self._enabled or self._synced:
            return False
        return datetime.now().year < 2025

    def try_sync_from_pgn(self, pgn: int, data: bytes) -> bool:
        """Try to extract UTC from PGN 126992 or 129033 and set system time.

        PGN 126992 (System Time): bytes 2-3 = days since 1970-01-01 (uint16 LE),
            bytes 4-7 = time of day (uint32 LE, units = 0.0001s)
        PGN 129033 (Time & Date): bytes 0-1 = days, bytes 2-5 = time of day
        """
        if not self._enabled or self._synced:
            return False

        try:
            if pgn == PGN_SYSTEM_TIME and len(data) >= 8:
                days = struct.unpack_from('<H', data, 2)[0]
                tod = struct.unpack_from('<I', data, 4)[0]
            elif pgn == PGN_TIME_DATE and len(data) >= 6:
                days = struct.unpack_from('<H', data, 0)[0]
                tod = struct.unpack_from('<I', data, 2)[0]
            else:
                return False

            if days == 0xFFFF or tod == 0xFFFFFFFF:
                return False

            seconds_in_day = tod * 0.0001
            if seconds_in_day >= 86400:
                return False

            epoch_seconds = days * 86400 + seconds_in_day
            dt = datetime.fromtimestamp(epoch_seconds, tz=timezone.utc)

            if dt.year < 2020 or dt.year > 2100:
                return False

            self._set_system_time(dt)
            self._synced = True
            self._sync_time = time.time()
            logger.info("GPS time sync: %s", dt.isoformat())
            return True

        except (struct.error, ValueError, OSError) as e:
            logger.warning("GPS time sync failed: %s", e)
            return False

    def _set_system_time(self, dt: datetime) -> None:
        """Set the system clock. Requires root on Linux."""
        time_str = dt.strftime("%Y-%m-%d %H:%M:%S")
        try:
            subprocess.run(
                ["sudo", "timedatectl", "set-time", time_str],
                capture_output=True, timeout=5, check=True,
            )
        except (FileNotFoundError, subprocess.CalledProcessError):
            subprocess.run(
                ["sudo", "date", "-u", "-s", time_str],
                capture_output=True, timeout=5, check=True,
            )

    @property
    def status(self) -> Dict[str, Any]:
        return {
            "enabled": self._enabled,
            "synced": self._synced,
            "sync_time": self._sync_time,
        }


# --- Main logger ---


class CANLogger:
    """CAN bus raw frame logger with filtering, rate limiting, and auto-prune."""

    def __init__(self, config: CANLoggerConfig, config_path: Optional[str] = None):
        self._config = config
        self._config_path = config_path
        self._config_mtime: float = 0.0

        self._filter = FrameFilter(config)
        self._writer = BinaryLogWriter(
            config.log_dir,
            int(config.max_file_size_mb * 1024 * 1024),
        )
        self._gps_sync = GPSTimeSync(config.gps_time_sync)

        self._bus = None
        self._running = False
        self._reader_thread: Optional[threading.Thread] = None
        self._start_time: float = 0.0

        # Ring buffer for recent frames (for web UI / SSE)
        self._recent_frames: collections.deque = collections.deque(maxlen=100)
        self._recent_lock = threading.Lock()

        # Per-PGN counters
        self._pgn_counts: Dict[int, int] = {}
        self._pgn_counts_lock = threading.Lock()

        # Message rate tracking
        self._msg_count = 0
        self._msg_count_start = 0.0

        if config_path:
            try:
                self._config_mtime = os.path.getmtime(config_path)
            except OSError:
                pass

    def start(self) -> None:
        """Open CAN bus and start reader thread."""
        if can is None:
            raise RuntimeError("python-can not installed")

        logger.info(
            "Starting CAN logger on %s, logging to %s",
            self._config.channel, self._config.log_dir,
        )

        self._bus = can.Bus(channel=self._config.channel, interface="socketcan")
        self._running = True
        self._start_time = time.time()
        self._msg_count_start = self._start_time

        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="can-reader",
        )
        self._reader_thread.start()
        logger.info("CAN reader thread started")

    def stop(self) -> None:
        """Shut down gracefully."""
        self._running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=3)
        self._writer.close()
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None
        logger.info(
            "CAN logger stopped. Total frames: %d", self._writer.stats["total_frames"],
        )

    def _reader_loop(self) -> None:
        """Background reader thread."""
        last_config_check = time.time()

        while self._running:
            try:
                msg = self._bus.recv(timeout=0.1)
            except Exception as e:
                if self._running:
                    logger.error("CAN recv error: %s", e)
                continue

            if msg is not None:
                self._process_frame(msg)

            # Periodic config hot-reload check
            now = time.time()
            if now - last_config_check >= 2.0:
                last_config_check = now
                self._check_config_reload()

    def _process_frame(self, msg) -> None:
        """Process a single CAN message."""
        can_id = msg.arbitration_id
        timestamp = msg.timestamp if msg.timestamp else time.time()
        data = bytes(msg.data)
        dlc = msg.dlc

        pgn, source, priority = extract_pgn_source_priority(can_id)

        self._msg_count += 1

        # GPS time sync — always check regardless of filters
        if self._gps_sync.needs_sync():
            self._gps_sync.try_sync_from_pgn(pgn, data)

        # Apply filter
        if not self._filter.should_log(pgn, source, timestamp):
            return

        # Write to binary log
        self._writer.write_frame(timestamp, can_id, dlc, data, pgn, source, priority)

        # Update PGN counts
        with self._pgn_counts_lock:
            self._pgn_counts[pgn] = self._pgn_counts.get(pgn, 0) + 1

        # Add to recent frames ring buffer
        frame_info = {
            "timestamp": timestamp,
            "pgn": pgn,
            "pgn_name": PGN_NAMES.get(pgn, f"PGN {pgn}"),
            "source": source,
            "priority": priority,
            "dlc": dlc,
            "data": data.hex(),
        }
        with self._recent_lock:
            self._recent_frames.append(frame_info)

    def _check_config_reload(self) -> None:
        """Hot-reload config if file has changed."""
        if not self._config_path:
            return
        try:
            mtime = os.path.getmtime(self._config_path)
            if mtime > self._config_mtime:
                self._config_mtime = mtime
                new_config = CANLoggerConfig.from_json(self._config_path)
                self._config = new_config
                self._filter.update(new_config)
                logger.info("Config reloaded from %s", self._config_path)
        except Exception as e:
            logger.warning("Config reload failed: %s", e)

    def get_recent_frames(self, limit: int = 20) -> List[Dict[str, Any]]:
        """Get last N frames from ring buffer."""
        with self._recent_lock:
            frames = list(self._recent_frames)
        return frames[-limit:]

    @property
    def config(self) -> CANLoggerConfig:
        return self._config

    @property
    def stats(self) -> Dict[str, Any]:
        """Aggregate stats for web UI."""
        now = time.time()
        elapsed = now - self._msg_count_start if self._msg_count_start else 0
        msg_rate = self._msg_count / elapsed if elapsed > 1 else 0.0

        # Disk usage
        log_dir = Path(self._config.log_dir)
        disk_used = 0
        if log_dir.exists():
            disk_used = sum(f.stat().st_size for f in log_dir.glob("can_*.bin"))

        with self._pgn_counts_lock:
            pgn_counts = dict(self._pgn_counts)

        return {
            "running": self._running,
            "uptime": now - self._start_time if self._start_time else 0,
            "msg_rate": round(msg_rate, 1),
            "total_received": self._msg_count,
            "filter": self._filter.stats,
            "writer": self._writer.stats,
            "disk_used_bytes": disk_used,
            "disk_limit_bytes": int(self._config.max_disk_mb * 1024 * 1024),
            "gps_sync": self._gps_sync.status,
            "pgn_counts": pgn_counts,
        }

    def trigger_prune(self) -> None:
        """Run disk pruning (called after file rotation)."""
        max_bytes = int(self._config.max_disk_mb * 1024 * 1024)
        self._writer.prune(max_bytes)


# --- CLI entry point ---


def main():
    parser = argparse.ArgumentParser(description="CAN Bus Raw Frame Datalogger")
    parser.add_argument("--config", type=str, help="Path to JSON config file")
    parser.add_argument("--channel", type=str, help="CAN interface (default: can0)")
    parser.add_argument("--log-dir", type=str, help="Log directory")
    parser.add_argument("--max-size", type=float, help="Max file size in MB")
    parser.add_argument("--web", action="store_true", help="Enable web UI")
    parser.add_argument("--web-port", type=int, help="Web UI port (default: 8082)")
    parser.add_argument("--web-host", type=str, help="Web UI host (default: 0.0.0.0)")
    parser.add_argument("--verbose", action="store_true", help="Debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    # Load config
    config_path = args.config
    if config_path and os.path.exists(config_path):
        config = CANLoggerConfig.from_json(config_path)
    else:
        config = CANLoggerConfig()

    # CLI overrides
    if args.channel:
        config.channel = args.channel
    if args.log_dir:
        config.log_dir = args.log_dir
    if args.max_size:
        config.max_file_size_mb = args.max_size
    if args.web:
        config.web_enabled = True
    if args.web_port:
        config.web_port = args.web_port
    if args.web_host:
        config.web_host = args.web_host

    logger_instance = CANLogger(config, config_path=config_path)

    # Signal handlers
    def shutdown(signum, frame):
        logger.info("Received signal %d, shutting down...", signum)
        logger_instance.stop()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Start logger
    logger_instance.start()

    # Start web UI if enabled
    if config.web_enabled:
        from src.n2k.can_logger_web import create_app
        app = create_app(logger_instance, config_path)
        web_thread = threading.Thread(
            target=lambda: app.run(
                host=config.web_host,
                port=config.web_port,
                debug=False,
                use_reloader=False,
            ),
            daemon=True,
            name="can-web",
        )
        web_thread.start()
        logger.info("Web UI at http://%s:%d", config.web_host, config.web_port)

    # Main thread waits
    try:
        while logger_instance._running:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        logger_instance.stop()


if __name__ == "__main__":
    main()
