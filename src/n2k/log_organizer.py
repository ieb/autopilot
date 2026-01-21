"""CAN log file organizer for NMEA2000 logs.

Parses various CAN log formats, extracts capture dates from timestamps or GPS PGN
messages, and organizes files into YYYY/MM directory structure.
"""

import argparse
import re
import shutil
from datetime import date, datetime, timedelta
from enum import Enum
from pathlib import Path
from typing import Optional, Tuple


class LogFormat(Enum):
    """Supported CAN log file formats."""

    CANDUMP = "candump"  # Standard candump with Unix timestamp
    ANALYZED_CSV = "analyzed"  # Human-readable CSV format
    HARDWARE_TS = "hardware_ts"  # Hardware timestamp with packed frames
    SLCAN = "slcan"  # slcan candump format
    RAYMARINE = "raymarine"  # Raymarine MFD native format
    UNKNOWN = "unknown"


# Regex patterns for format detection
# Standard candump: (timestamp) can0 CANID#DATA
CANDUMP_PATTERN = re.compile(r"^\((\d+\.\d+)\)\s+can\d+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)")
# Candump variant with spaces and brackets: (timestamp)  can0  CANID   [DLC]  XX XX XX...
CANDUMP_VARIANT_PATTERN = re.compile(
    r"^\s*\((\d+\.\d+)\)\s+can\d+\s+([0-9A-Fa-f]+)\s+\[\d+\]\s+((?:[0-9A-Fa-f]{2}\s*)+)"
)
SLCAN_PATTERN = re.compile(r"^\((\d+\.\d+)\)\s+slcan\d+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)")
ANALYZED_CSV_PATTERN = re.compile(
    r"^(\d{4}-\d{2}-\d{2}-\d{2}:\d{2}:\d{2}\.\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(.*)"
)
# JSON format with timestamp field (from analyzer output)
# Example: {"timestamp":"2023-05-15-06:16:45.483","prio":7,...}
JSON_TIMESTAMP_PATTERN = re.compile(
    r'^\s*\{"timestamp"\s*:\s*"(\d{4}-\d{2}-\d{2}-\d{2}:\d{2}:\d{2}\.\d+)"'
)
HARDWARE_TS_PATTERN = re.compile(r"^\([\d.]+\)\s+ffffffff([0-9a-fA-F]+)")
# Raymarine MFD native format: Rx|Tx TIMESTAMP CAN_ID[4 bytes] DATA[8 bytes]
# Example: Rx 1799366 0d ed cc 00 a1 ff ff ff ff 04 01 3b
RAYMARINE_PATTERN = re.compile(
    r"^(Rx|Tx)\s+(\d+)\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})"
)

# Maximum plausible date for validation (reject dates far in the future)
MAX_VALID_DATE = date(2030, 12, 31)
MIN_VALID_DATE = date(2010, 1, 1)

# PGNs containing date information
PGN_SYSTEM_TIME = 126992
PGN_GNSS_POSITION = 129029
PGN_TIME_DATE = 129033
DATE_PGNS = {PGN_SYSTEM_TIME, PGN_GNSS_POSITION, PGN_TIME_DATE}

# Filename date patterns
FILENAME_DATE_PATTERNS = [
    re.compile(r"(\d{4})-(\d{2})-(\d{2})"),  # YYYY-MM-DD
    re.compile(r"(\d{4})(\d{2})(\d{2})"),  # YYYYMMDD
]


class CANLogOrganizer:
    """Organizes CAN log files into YYYY/MM directory structure."""

    def detect_format(self, filepath: Path) -> LogFormat:
        """Detect the format of a CAN log file by examining its contents."""
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                # Read first few non-empty lines
                lines_checked = 0
                for line in f:
                    line = line.strip()
                    if not line:
                        continue

                    # Skip INFO/ERROR/DEBUG log lines from analyzer output
                    if line.startswith(("INFO ", "ERROR ", "DEBUG ", "WARN ")):
                        continue

                    # Check for analyzed CSV format first (most specific)
                    if ANALYZED_CSV_PATTERN.match(line):
                        return LogFormat.ANALYZED_CSV

                    # Check for JSON format with timestamp
                    if JSON_TIMESTAMP_PATTERN.match(line):
                        return LogFormat.ANALYZED_CSV  # Same date extraction logic

                    # Check for slcan format
                    if SLCAN_PATTERN.match(line):
                        return LogFormat.SLCAN

                    # Check for standard candump format (or variant with spaces)
                    if CANDUMP_PATTERN.match(line) or CANDUMP_VARIANT_PATTERN.match(line):
                        return LogFormat.CANDUMP

                    # Check for hardware timestamp format
                    if HARDWARE_TS_PATTERN.match(line):
                        return LogFormat.HARDWARE_TS

                    # Check for Raymarine MFD native format
                    if RAYMARINE_PATTERN.match(line):
                        return LogFormat.RAYMARINE

                    lines_checked += 1
                    if lines_checked >= 10:
                        break

        except Exception:
            pass

        return LogFormat.UNKNOWN

    def extract_date(self, filepath: Path) -> Optional[date]:
        """Extract the capture date from a CAN log file."""
        log_format = self.detect_format(filepath)

        if log_format == LogFormat.CANDUMP:
            return self._extract_date_candump(filepath)
        elif log_format == LogFormat.SLCAN:
            return self._extract_date_slcan(filepath)
        elif log_format == LogFormat.ANALYZED_CSV:
            return self._extract_date_analyzed_csv(filepath)
        elif log_format == LogFormat.HARDWARE_TS:
            return self._extract_date_hardware_ts(filepath)
        elif log_format == LogFormat.RAYMARINE:
            return self._extract_date_raymarine(filepath)
        else:
            # Try to extract from filename
            return self._extract_date_from_filename(filepath)

    def _extract_date_candump(self, filepath: Path) -> Optional[date]:
        """Extract date from standard candump format with Unix timestamp."""
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    line = line.strip()
                    # Try standard format first
                    match = CANDUMP_PATTERN.match(line)
                    if not match:
                        # Try variant format with spaces and brackets
                        match = CANDUMP_VARIANT_PATTERN.match(line)
                    if match:
                        timestamp = float(match.group(1))
                        extracted = datetime.fromtimestamp(timestamp).date()
                        if self._is_valid_date(extracted):
                            return extracted
        except Exception:
            pass
        return self._extract_date_from_filename(filepath)

    def _is_valid_date(self, d: date) -> bool:
        """Check if date is within a plausible range."""
        return MIN_VALID_DATE <= d <= MAX_VALID_DATE

    def _extract_date_slcan(self, filepath: Path) -> Optional[date]:
        """Extract date from slcan candump format with Unix timestamp."""
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    match = SLCAN_PATTERN.match(line.strip())
                    if match:
                        timestamp = float(match.group(1))
                        # Check if timestamp looks like a valid Unix epoch (after year 2000)
                        # Small values like 1953.0 are relative timestamps, not epoch
                        if timestamp > 946684800:  # Jan 1, 2000 in Unix time
                            extracted = datetime.fromtimestamp(timestamp).date()
                            if self._is_valid_date(extracted):
                                return extracted
                        else:
                            # Relative timestamp - need to parse PGN for date
                            break
        except Exception:
            pass

        # Try to extract date from PGN messages in slcan format
        extracted_date = self._extract_date_from_slcan_pgn(filepath)
        if extracted_date and self._is_valid_date(extracted_date):
            return extracted_date

        return self._extract_date_from_filename(filepath)

    def _extract_date_from_slcan_pgn(self, filepath: Path) -> Optional[date]:
        """Extract date from PGN messages in slcan format files with relative timestamps."""
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    match = SLCAN_PATTERN.match(line.strip())
                    if not match:
                        continue

                    can_id_hex = match.group(2)
                    data_hex = match.group(3)

                    try:
                        can_id = int(can_id_hex, 16)
                        data = bytes.fromhex(data_hex)
                    except ValueError:
                        continue

                    pgn = self._extract_pgn_from_can_id(can_id)

                    # Check for PGN 126992 (System Time)
                    if pgn == PGN_SYSTEM_TIME:
                        extracted_date = self._decode_pgn_126992_date(data)
                        if extracted_date and self._is_valid_date(extracted_date):
                            return extracted_date

        except Exception:
            pass

        return None

    def _extract_date_analyzed_csv(self, filepath: Path) -> Optional[date]:
        """Extract date from analyzed CSV or JSON format with datetime string."""
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    line = line.strip()
                    # Skip INFO/ERROR/DEBUG log lines
                    if line.startswith(("INFO ", "ERROR ", "DEBUG ", "WARN ")):
                        continue
                    # Try CSV format first
                    match = ANALYZED_CSV_PATTERN.match(line)
                    if match:
                        datetime_str = match.group(1)
                        # Parse: 2018-05-12-13:51:08.613
                        dt = datetime.strptime(datetime_str[:19], "%Y-%m-%d-%H:%M:%S")
                        return dt.date()
                    # Try JSON format
                    match = JSON_TIMESTAMP_PATTERN.match(line)
                    if match:
                        datetime_str = match.group(1)
                        # Parse: 2023-05-15-06:16:45.483
                        dt = datetime.strptime(datetime_str[:19], "%Y-%m-%d-%H:%M:%S")
                        return dt.date()
        except Exception:
            pass
        return self._extract_date_from_filename(filepath)

    def _extract_date_hardware_ts(self, filepath: Path) -> Optional[date]:
        """Extract date from hardware timestamp format.

        For hardware timestamp format, filename dates are often more accurate
        for when the capture was made (vs. GPS data which may be from replayed
        logs or devices with stale clocks). So we prefer filename dates when
        available.
        """
        # First try filename - these often indicate when capture was made
        filename_date = self._extract_date_from_filename(filepath)
        if filename_date and self._is_valid_date(filename_date):
            return filename_date

        # Fall back to PGN messages
        extracted_date = self._extract_date_from_pgn_messages(filepath)
        if extracted_date and self._is_valid_date(extracted_date):
            return extracted_date

        return None

    def _extract_date_from_pgn_messages(self, filepath: Path) -> Optional[date]:
        """Parse hardware timestamp format and extract date from GPS PGN messages."""
        # For fast-packet PGNs, we need to reassemble frames
        fast_packet_buffers: dict[Tuple[int, int], list[bytes]] = {}  # (pgn, seq_id) -> frames

        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    result = self._parse_hardware_ts_line(line.strip())
                    if result is None:
                        continue

                    can_id, data = result
                    pgn = self._extract_pgn_from_can_id(can_id)

                    if pgn not in DATE_PGNS:
                        continue

                    # Handle single-frame PGN 126992 (System Time)
                    if pgn == PGN_SYSTEM_TIME:
                        extracted_date = self._decode_pgn_126992_date(data)
                        if extracted_date:
                            return extracted_date

                    # Handle fast-packet PGNs (129029, 129033)
                    elif pgn in (PGN_GNSS_POSITION, PGN_TIME_DATE):
                        # First byte contains sequence counter and frame index
                        if len(data) < 2:
                            continue

                        frame_info = data[0]
                        seq_id = (frame_info >> 5) & 0x07
                        frame_idx = frame_info & 0x1F

                        key = (pgn, seq_id)

                        if frame_idx == 0:
                            # First frame: second byte is total length
                            fast_packet_buffers[key] = [data[2:]]  # Skip counter and length
                        elif key in fast_packet_buffers:
                            # Continuation frame
                            fast_packet_buffers[key].append(data[1:])

                            # Try to decode after we have enough data
                            assembled = b"".join(fast_packet_buffers[key])
                            if pgn == PGN_GNSS_POSITION and len(assembled) >= 4:
                                extracted_date = self._decode_pgn_129029_date(assembled)
                                if extracted_date:
                                    return extracted_date
                            elif pgn == PGN_TIME_DATE and len(assembled) >= 4:
                                extracted_date = self._decode_pgn_129033_date(assembled)
                                if extracted_date:
                                    return extracted_date

        except Exception:
            pass

        return None

    def _parse_hardware_ts_line(self, line: str) -> Optional[Tuple[int, bytes]]:
        """Parse a hardware timestamp format line.

        Format: (timestamp) ffffffff<can_id_le:4><dlc:1><pad:3><data:8><hw_ts:4>
        """
        match = HARDWARE_TS_PATTERN.match(line)
        if not match:
            return None

        hex_data = match.group(1)
        if len(hex_data) < 32:
            return None

        try:
            # CAN ID is first 4 bytes, little-endian
            can_id = int.from_bytes(bytes.fromhex(hex_data[0:8]), "little")
            # DLC is next byte
            dlc = int(hex_data[8:10], 16)
            # Data starts after DLC + 3 padding bytes (8 bytes total offset from start)
            data = bytes.fromhex(hex_data[16 : 16 + min(dlc, 8) * 2])
            return can_id, data
        except (ValueError, IndexError):
            return None

    def _extract_pgn_from_can_id(self, can_id: int) -> int:
        """Extract PGN from 29-bit CAN ID.

        CAN ID format (29 bits):
        - Priority: bits 26-28 (3 bits)
        - Reserved: bit 25 (1 bit)
        - Data Page: bit 24 (1 bit)
        - PDU Format (PF): bits 16-23 (8 bits)
        - PDU Specific (PS): bits 8-15 (8 bits)
        - Source Address: bits 0-7 (8 bits)

        For PDU2 (PF >= 240): PGN = DP << 16 | PF << 8 | PS
        For PDU1 (PF < 240): PGN = DP << 16 | PF << 8 (PS is destination)
        """
        dp = (can_id >> 24) & 0x01
        pf = (can_id >> 16) & 0xFF
        ps = (can_id >> 8) & 0xFF

        if pf >= 240:
            # PDU2 format: PS is part of PGN
            return (dp << 16) | (pf << 8) | ps
        else:
            # PDU1 format: PS is destination address
            return (dp << 16) | (pf << 8)

    def _decode_pgn_126992_date(self, data: bytes) -> Optional[date]:
        """Decode date from PGN 126992 (System Time).

        Field layout:
        - Byte 0: SID
        - Byte 1: Time Source
        - Bytes 2-3: Date (days since 1970-01-01, uint16 LE)
        - Bytes 4-7: Time (seconds since midnight * 10000, uint32 LE)
        """
        if len(data) < 4:
            return None

        try:
            days = int.from_bytes(data[2:4], "little")
            if days == 0xFFFF or days == 0:
                return None
            return date(1970, 1, 1) + timedelta(days=days)
        except (ValueError, OverflowError):
            return None

    def _decode_pgn_129029_date(self, data: bytes) -> Optional[date]:
        """Decode date from PGN 129029 (GNSS Position Data).

        Field layout (after fast-packet reassembly):
        - Byte 0: SID
        - Bytes 1-2: Date (days since 1970-01-01, uint16 LE)
        - Bytes 3-6: Time (seconds since midnight * 10000, uint32 LE)
        - ... more fields follow
        """
        if len(data) < 3:
            return None

        try:
            days = int.from_bytes(data[1:3], "little")
            if days == 0xFFFF or days == 0:
                return None
            return date(1970, 1, 1) + timedelta(days=days)
        except (ValueError, OverflowError):
            return None

    def _decode_pgn_129033_date(self, data: bytes) -> Optional[date]:
        """Decode date from PGN 129033 (Time & Date).

        Field layout:
        - Bytes 0-1: Date (days since 1970-01-01, uint16 LE)
        - Bytes 2-5: Time (seconds since midnight * 10000, uint32 LE)
        - Bytes 6-7: Local Offset (minutes, int16 LE)
        """
        if len(data) < 2:
            return None

        try:
            days = int.from_bytes(data[0:2], "little")
            if days == 0xFFFF or days == 0:
                return None
            return date(1970, 1, 1) + timedelta(days=days)
        except (ValueError, OverflowError):
            return None

    def _extract_date_from_filename(self, filepath: Path) -> Optional[date]:
        """Extract date from filename using common patterns."""
        filename = filepath.name

        for pattern in FILENAME_DATE_PATTERNS:
            match = pattern.search(filename)
            if match:
                try:
                    year = int(match.group(1))
                    month = int(match.group(2))
                    day = int(match.group(3))
                    return date(year, month, day)
                except ValueError:
                    continue

        return None

    def _extract_date_raymarine(self, filepath: Path) -> Optional[date]:
        """Extract date from Raymarine MFD native format.

        Raymarine logs use relative timestamps (milliseconds since boot),
        so we must parse GPS PGN messages to get the actual date.
        """
        try:
            with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    result = self._parse_raymarine_line(line.strip())
                    if result is None:
                        continue

                    can_id, data = result
                    pgn = self._extract_pgn_from_can_id(can_id)

                    # Check for PGN 126992 (System Time) - single frame, contains date
                    if pgn == PGN_SYSTEM_TIME:
                        extracted_date = self._decode_pgn_126992_date(data)
                        if extracted_date and self._is_valid_date(extracted_date):
                            return extracted_date

        except Exception:
            pass

        # Fall back to filename if no valid GPS date found
        return self._extract_date_from_filename(filepath)

    def _parse_raymarine_line(self, line: str) -> Optional[Tuple[int, bytes]]:
        """Parse a Raymarine MFD native format line.

        Format: Rx|Tx TIMESTAMP_MS CAN_ID[4 bytes BE] DATA[8 bytes]
        Example: Rx 1799366 0d ed cc 00 a1 ff ff ff ff 04 01 3b

        Returns:
            Tuple of (can_id, data) or None if line doesn't match
        """
        match = RAYMARINE_PATTERN.match(line)
        if not match:
            return None

        try:
            # CAN ID from groups 3-6 (big-endian: concatenate as hex string)
            can_id_hex = match.group(3) + match.group(4) + match.group(5) + match.group(6)
            can_id = int(can_id_hex, 16)

            # Data from groups 7-14
            data_hex = "".join(match.group(i) for i in range(7, 15))
            data = bytes.fromhex(data_hex)

            return can_id, data
        except (ValueError, IndexError):
            return None

    def organize_files(
        self, src_dir: Path, dest_dir: Optional[Path] = None, dry_run: bool = True
    ) -> list[Tuple[Path, Path, Optional[date], LogFormat]]:
        """Organize CAN log files into YYYY/MM directory structure.

        Args:
            src_dir: Source directory containing log files
            dest_dir: Destination base directory (defaults to src_dir)
            dry_run: If True, don't actually move files

        Returns:
            List of (source_path, dest_path, extracted_date, format) tuples
        """
        if dest_dir is None:
            dest_dir = src_dir

        results: list[Tuple[Path, Path, Optional[date], LogFormat]] = []

        # Get all files in source directory
        if not src_dir.is_dir():
            print(f"Error: {src_dir} is not a directory")
            return results

        for filepath in sorted(src_dir.iterdir()):
            if not filepath.is_file():
                continue

            # Skip hidden files
            if filepath.name.startswith("."):
                continue

            log_format = self.detect_format(filepath)
            extracted_date = self.extract_date(filepath)

            if extracted_date:
                # Create destination path
                year_month_dir = dest_dir / str(extracted_date.year) / f"{extracted_date.month:02d}"
                dest_path = year_month_dir / filepath.name
            else:
                # Can't determine date - put in 'unknown' folder
                dest_path = dest_dir / "unknown" / filepath.name

            results.append((filepath, dest_path, extracted_date, log_format))

            if not dry_run:
                dest_path.parent.mkdir(parents=True, exist_ok=True)
                if filepath != dest_path:
                    shutil.move(str(filepath), str(dest_path))

        return results


def main() -> None:
    """CLI entry point for CAN log organizer."""
    parser = argparse.ArgumentParser(
        description="Organize CAN log files into YYYY/MM directory structure"
    )
    parser.add_argument("src_dir", type=Path, help="Source directory containing log files")
    parser.add_argument(
        "--dest-dir",
        type=Path,
        default=None,
        help="Destination directory (defaults to source directory)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        default=True,
        help="Show what would be done without making changes (default: True)",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually move the files (opposite of --dry-run)",
    )

    args = parser.parse_args()

    dry_run = not args.execute

    organizer = CANLogOrganizer()
    results = organizer.organize_files(args.src_dir, args.dest_dir, dry_run=dry_run)

    if dry_run:
        print("DRY RUN - No files will be moved\n")

    # Print results grouped by format
    format_counts: dict[LogFormat, int] = {}
    date_counts: dict[Optional[date], int] = {}

    print(f"{'Source File':<45} {'Format':<15} {'Date':<12} {'Destination'}")
    print("-" * 120)

    for src, dest, extracted_date, log_format in results:
        format_counts[log_format] = format_counts.get(log_format, 0) + 1
        date_counts[extracted_date] = date_counts.get(extracted_date, 0) + 1

        date_str = str(extracted_date) if extracted_date else "UNKNOWN"
        dest_rel = dest.relative_to(args.dest_dir or args.src_dir) if dest else "N/A"
        print(f"{src.name:<45} {log_format.value:<15} {date_str:<12} {dest_rel}")

    print("\n" + "=" * 60)
    print("Summary:")
    print(f"  Total files: {len(results)}")
    print("\n  By format:")
    for fmt, count in sorted(format_counts.items(), key=lambda x: x[0].value):
        print(f"    {fmt.value}: {count}")
    print("\n  By date:")
    for dt, count in sorted(date_counts.items(), key=lambda x: (x[0] is None, x[0])):
        date_str = str(dt) if dt else "UNKNOWN"
        print(f"    {date_str}: {count}")

    if dry_run:
        print("\nTo execute, run with --execute flag")


if __name__ == "__main__":
    main()
