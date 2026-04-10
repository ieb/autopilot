#!/usr/bin/env python3
"""
Convert CANL binary log files to candump format for use with canplayer.

Usage:
    python scripts/canl_to_candump.py n2klogs/logged/*.bin > output.log
    canplayer -I output.log can0=can0

    # Or pipe directly:
    python scripts/canl_to_candump.py n2klogs/logged/*.bin | canplayer can0=can0

    # Specify output file:
    python scripts/canl_to_candump.py -o output.log n2klogs/logged/*.bin

    # Use a different interface name:
    python scripts/canl_to_candump.py -i vcan0 n2klogs/logged/*.bin

Output format (candump/canplayer compatible):
    (1710416526.105096) can0 19F51100#3B9FF081900003

CAN ID is the full 29-bit extended ID reconstructed from PGN, source, and priority.
"""

import argparse
import struct
import sys
from pathlib import Path

HEADER_STRUCT = struct.Struct('<4sIII d 8x')  # 32 bytes
FRAME_STRUCT = struct.Struct('<II BB BB 8s I')  # 24 bytes
TIMESTAMP_UNIT_S = 0.0001


def canl_to_candump(bin_path, interface, outfile):
    """Convert one CANL .bin file to candump lines."""
    count = 0
    with open(bin_path, 'rb') as f:
        header = f.read(32)
        if len(header) < 32:
            print(f"# Skipping {bin_path}: too short", file=sys.stderr)
            return 0
        magic, version, flags, _, start_ts = HEADER_STRUCT.unpack(header)
        if magic != b'CANL':
            print(f"# Skipping {bin_path}: bad magic {magic!r}", file=sys.stderr)
            return 0

        while True:
            raw = f.read(24)
            if len(raw) < 24:
                break
            ts_offset, can_id, dlc, fflags, source, priority, data, pgn = FRAME_STRUCT.unpack(raw)

            timestamp = start_ts + ts_offset * TIMESTAMP_UNIT_S
            dlc = min(dlc, 8)
            data_hex = data[:dlc].hex().upper()

            # Use the stored can_id directly — it's the full 29-bit extended ID
            # as captured from the bus
            outfile.write(f"({timestamp:.6f}) {interface} {can_id:08X}#{data_hex}\n")
            count += 1

    return count


def main():
    parser = argparse.ArgumentParser(
        description="Convert CANL binary logs to candump format for canplayer")
    parser.add_argument("files", nargs="+", type=Path,
                        help="CANL .bin files to convert")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="Output file (default: stdout)")
    parser.add_argument("-i", "--interface", default="can0",
                        help="CAN interface name in output (default: can0)")
    args = parser.parse_args()

    if args.output:
        outfile = open(args.output, 'w')
    else:
        outfile = sys.stdout

    total = 0
    for path in sorted(args.files):
        if not path.exists():
            print(f"# File not found: {path}", file=sys.stderr)
            continue
        n = canl_to_candump(path, args.interface, outfile)
        total += n
        print(f"# {path.name}: {n:,} frames", file=sys.stderr)

    print(f"# Total: {total:,} frames", file=sys.stderr)

    if args.output:
        outfile.close()


if __name__ == "__main__":
    main()
