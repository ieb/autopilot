# Raymarine MFD Log Format Support

> **Status**: Implemented in [`src/n2k/log_organizer.py`](../src/n2k/log_organizer.py)

## Overview

Add support for Raymarine MFD native log format to the CAN log organizer, enabling date extraction from GPS PGN messages and file organization into YYYY/MM structure.

## Format Analysis

The 7 files in [`n2klogs/ray/`](../n2klogs/ray/) include:

### Raymarine Native Format (4 files)
- **Files**: `N2K_000001.log`, `N2K_000001.1.log`, `N2K_000001.2.log`, `N2K_000001.4.log`
- **Pattern**: `Rx|Tx TIMESTAMP_MS CAN_ID[4 bytes] DATA[8 bytes]`
- **Example**: `Rx 1799366 0d ed cc 00 a1 ff ff ff ff 04 01 3b`
- **Date extraction**: Parse GPS PGN messages (126992, 129029)

### Already-Converted Formats (3 files)
- **`N2K_000001.converted.log`**: Mixed analyzer output with JSON lines containing timestamps like `2023-05-14-18:43:03.430`
- **`N2K_000001.converted.json.log`**: Pure JSON format with same timestamp structure
- **`N2K_000001.4.log.candump`**: slcan format with relative timestamps

## Raymarine Native Format Details

```
Rx 1799400 09 f8 01 67 86 4f f8 1e aa bf c2 00
│  │       └─────────┘ └───────────────────────┘
│  │       CAN ID      Data (8 bytes)
│  └─ Timestamp (milliseconds, relative)
└─ Direction (Rx=received, Tx=transmitted)
```

### CAN ID Decoding
The 4 CAN ID bytes are big-endian, forming a 29-bit extended CAN ID:

```
0x09F80167 → PGN 129025 (Position Rapid Update)
0x0DF01067 → PGN 126992 (System Time) ← Contains date!
```

### PGN Extraction from CAN ID
```python
can_id = 0x0DF01067  # from bytes "0d f0 10 67"
dp = (can_id >> 24) & 0x01     # Data Page = 1
pf = (can_id >> 16) & 0xFF     # PDU Format = 240
ps = (can_id >> 8) & 0xFF      # PDU Specific = 16
# PF >= 240 → PDU2: PGN = (dp << 16) | (pf << 8) | ps = 126992
```

### GPS PGN Data Found
Confirmed PGN 126992 (System Time) messages in logs:
```
Rx 1799900 0d f0 10 67 2e f0 23 4c b0 9b 9e 10
                       │  │  └────┘
                       │  │  Date = 0x4C23 = 19491 days = 2023-05-14
                       │  └─ Time Source (GPS)
                       └─ SID
```

## Implementation

### Changes to [`src/n2k/log_organizer.py`](../src/n2k/log_organizer.py)

**1. New LogFormat enum value:**
```python
class LogFormat(Enum):
    # ... existing ...
    RAYMARINE = "raymarine"  # Raymarine MFD native format
```

**2. Detection pattern:**
```python
RAYMARINE_PATTERN = re.compile(
    r"^(Rx|Tx)\s+(\d+)\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+"
    r"([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})"
)
```

**3. Line parsing helper:**
```python
def _parse_raymarine_line(self, line: str) -> Optional[Tuple[int, bytes]]:
    """Parse Raymarine format line, return (can_id, data)."""
    match = RAYMARINE_PATTERN.match(line)
    if match:
        # CAN ID from bytes 3-6 (big-endian)
        can_id = int(match.group(3) + match.group(4) + 
                     match.group(5) + match.group(6), 16)
        # Data from bytes 7-14
        data = bytes.fromhex(''.join(match.group(i) for i in range(7, 15)))
        return can_id, data
```

**4. Date extraction method:**
```python
def _extract_date_raymarine(self, filepath: Path) -> Optional[date]:
    """Extract date from Raymarine MFD native format."""
    # Parse lines, extract CAN ID and data
    # Look for PGN 126992 (System Time)
    # Decode date from PGN data fields (days since 1970-01-01)
```

### Additional Improvements

- Added `JSON_TIMESTAMP_PATTERN` for JSON analyzer output with timestamps
- Updated format detection to skip INFO/ERROR/DEBUG log lines
- Added `_extract_date_from_slcan_pgn()` for slcan files with relative timestamps

## Test Results

All 7 files successfully processed:

| File | Format | Extracted Date | Destination |
|------|--------|---------------|-------------|
| N2K_000001.log | raymarine | 2023-05-14 | 2023/05/ |
| N2K_000001.1.log | raymarine | 2023-05-14 | 2023/05/ |
| N2K_000001.2.log | raymarine | 2023-05-14 | 2023/05/ |
| N2K_000001.4.log | raymarine | 2023-05-14 | 2023/05/ |
| N2K_000001.converted.log | analyzed | 2023-05-14 | 2023/05/ |
| N2K_000001.converted.json.log | analyzed | 2023-05-15 | 2023/05/ |
| N2K_000001.4.log.candump | slcan | 2023-05-14 | 2023/05/ |
