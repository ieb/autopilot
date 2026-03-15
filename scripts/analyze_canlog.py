#!/usr/bin/env python3
"""
Analyze CAN log binary files from src.n2k.can_logger.

Identifies all devices on the bus, their PGN traffic, and reverse-engineers
Raymarine proprietary messages (especially PGN 126720).
"""

import struct
import sys
from collections import defaultdict, Counter
from pathlib import Path

# Binary format from can_logger.py
HEADER_STRUCT = struct.Struct('<4sIII d 8x')  # 32 bytes
FRAME_STRUCT = struct.Struct('<II BB BB 8s I')  # 24 bytes

# Known PGN names
PGN_NAMES = {
    59904: "ISO Request",
    60928: "ISO Address Claim",
    126208: "NMEA Command/Request Group",
    126464: "PGN List (Transmit/Receive)",
    126720: "Raymarine Proprietary",
    126992: "System Time",
    126993: "Heartbeat",
    126996: "Product Information",
    127237: "Heading/Track Control",
    127245: "Rudder",
    127250: "Vessel Heading",
    127251: "Rate of Turn",
    127257: "Attitude",
    127258: "Magnetic Variation",
    127506: "DC Detailed Status",
    127508: "Battery Status",
    127513: "Battery Config",
    128259: "Speed (Water)",
    128267: "Water Depth",
    128275: "Distance Log",
    129025: "Position Rapid",
    129026: "COG/SOG Rapid",
    129029: "GNSS Position",
    129038: "AIS Class A Position",
    129039: "AIS Class B Position",
    129040: "AIS Class B Extended",
    129041: "AIS Aids to Navigation",
    129044: "Datum",
    129283: "Cross Track Error",
    129283: "XTE",
    129539: "GNSS DOPs",
    129540: "GNSS Sats in View",
    129793: "AIS UTC/Date Report",
    129794: "AIS Class A Static",
    129797: "AIS Binary Message",
    129809: "AIS Class B CS Static 1",
    129810: "AIS Class B CS Static 2",
    130306: "Wind Data",
    130310: "Environmental (Temp/Humidity/Pressure)",
    130311: "Environmental (Temp/Humidity/Pressure 2)",
    130312: "Temperature",
    130313: "Humidity",
    130314: "Actual Pressure",
    130316: "Temperature Extended",
    130822: "Raymarine Proprietary 2",
    130846: "Unknown Proprietary",
    130916: "Unknown Proprietary 2",
    65311: "Seatalk Alarm",
    65345: "Seatalk Wind Datum",
    65359: "Seatalk Pilot Heading",
    65362: "Seatalk Pilot Speed",
    65379: "Seatalk Pilot Mode",
    65384: "Seatalk Pilot Keypad Heartbeat",
}

# ISO Address Claim: decode NAME field
def decode_address_claim(data: bytes) -> dict:
    """Decode 8-byte ISO Address Claim NAME field."""
    if len(data) < 8:
        return {}
    val = int.from_bytes(data[:8], 'little')
    return {
        'unique_number': val & 0x1FFFFF,
        'manufacturer_code': (val >> 21) & 0x7FF,
        'device_instance_lower': (val >> 32) & 0x07,
        'device_instance_upper': (val >> 35) & 0x1F,
        'device_function': (val >> 40) & 0xFF,
        'device_class': (val >> 48) & 0x7F,
        'system_instance': (val >> 56) & 0x0F,
        'industry_group': (val >> 60) & 0x07,
        'self_configurable': (val >> 63) & 0x01,
    }

MANUFACTURER_CODES = {
    275: "Raymarine",
    137: "Garmin",
    381: "B&G / Navico",
    743: "Unknown (743)",
    147: "Simrad / Navico",
    235: "Airmar",
    273: "Furuno",
    345: "Victron",
    419: "Maretron",
    529: "Yacht Devices",
    # Add more as found
}

DEVICE_CLASSES = {
    25: "Network Device",
    30: "Electrical Distribution",
    40: "Steering",
    60: "Navigation",
    70: "Communication",
    75: "Sensor Communication",
    80: "Instrumentation/General Systems",
    85: "External Environment",
    90: "Internal Environment",
}

DEVICE_FUNCTIONS = {
    130: "GPS",
    140: "Autopilot",
    150: "Steering Gear / Rudder",
    160: "Heading Sensor",
    170: "Wind Sensor",
    175: "Attitude Sensor",
}


# Fast-packet reassembly
class FastPacketReassembler:
    """Reassemble NMEA2000 fast-packet messages."""
    def __init__(self):
        self.sessions = {}  # (source, pgn) -> {seq, frames, total_len, data}

    def process_frame(self, source: int, pgn: int, data: bytes) -> bytes | None:
        """Process a CAN frame. Returns complete message data when reassembly is done."""
        if len(data) < 2:
            return None

        counter = data[0]
        frame_num = counter & 0x1F
        seq = (counter >> 5) & 0x07

        key = (source, pgn)

        if frame_num == 0:
            # First frame: [counter] [total_len] [up to 6 bytes data]
            total_len = data[1]
            payload = data[2:8]
            self.sessions[key] = {
                'seq': seq,
                'total_len': total_len,
                'data': bytearray(payload[:min(6, total_len)]),
                'expected_frame': 1,
            }
            if total_len <= 6:
                # Complete in one frame
                result = bytes(self.sessions[key]['data'][:total_len])
                del self.sessions[key]
                return result
            return None
        else:
            # Continuation frame: [counter] [up to 7 bytes data]
            session = self.sessions.get(key)
            if session is None:
                return None
            if seq != session['seq']:
                # Wrong sequence — drop
                del self.sessions[key]
                return None
            if frame_num != session['expected_frame']:
                # Out of order — drop
                del self.sessions[key]
                return None

            payload = data[1:8]
            session['data'].extend(payload)
            session['expected_frame'] = frame_num + 1

            if len(session['data']) >= session['total_len']:
                result = bytes(session['data'][:session['total_len']])
                del self.sessions[key]
                return result
            return None


# Fast-packet PGNs (multi-frame)
FAST_PACKET_PGNS = {
    126208, 126464, 126720, 126996, 128275,
    129029, 129038, 129039, 129040, 129041, 129044,
    129283, 129540, 129793, 129794, 129797, 129809, 129810,
    130822, 130846, 130916,
}


def read_log(path: Path):
    """Read all frames from a binary log file."""
    with open(path, 'rb') as f:
        header = f.read(32)
        if len(header) < 32:
            return
        magic, version, flags, _, start_ts = HEADER_STRUCT.unpack(header)
        if magic != b'CANL':
            print(f"Bad magic: {magic}")
            return

        print(f"File: {path.name}")
        print(f"  Version: {version}, Flags: {flags:#x}")
        import datetime
        print(f"  Start: {datetime.datetime.fromtimestamp(start_ts)}")

        frames = []
        while True:
            raw = f.read(24)
            if len(raw) < 24:
                break
            ts_offset, can_id, dlc, fflags, source, priority, data, pgn = FRAME_STRUCT.unpack(raw)
            frames.append({
                'ts': start_ts + ts_offset * 0.0001,
                'ts_offset': ts_offset,
                'can_id': can_id,
                'dlc': dlc,
                'source': source,
                'priority': priority,
                'data': data[:dlc],
                'pgn': pgn,
            })
        print(f"  Frames: {len(frames):,}")
        return frames


def analyze_devices(frames):
    """Identify all devices via ISO Address Claim (PGN 60928)."""
    print("\n" + "=" * 80)
    print("DEVICE IDENTIFICATION (from ISO Address Claim PGN 60928)")
    print("=" * 80)

    devices = {}
    for f in frames:
        if f['pgn'] == 60928:
            info = decode_address_claim(f['data'])
            addr = f['source']
            devices[addr] = info

    for addr in sorted(devices):
        info = devices[addr]
        mfr = MANUFACTURER_CODES.get(info['manufacturer_code'], f"Unknown ({info['manufacturer_code']})")
        dev_class = DEVICE_CLASSES.get(info['device_class'], f"Unknown ({info['device_class']})")
        dev_func = DEVICE_FUNCTIONS.get(info['device_function'], f"Function {info['device_function']}")
        print(f"\n  Source {addr}:")
        print(f"    Manufacturer: {mfr} ({info['manufacturer_code']})")
        print(f"    Device Class: {dev_class} ({info['device_class']})")
        print(f"    Device Function: {dev_func} ({info['device_function']})")
        print(f"    Unique Number: {info['unique_number']}")
        print(f"    Instance: {info['device_instance_lower']}.{info['device_instance_upper']}")
        print(f"    Industry Group: {info['industry_group']}")

    return devices


def analyze_product_info(frames):
    """Decode Product Information (PGN 126996) — fast-packet."""
    print("\n" + "=" * 80)
    print("PRODUCT INFORMATION (PGN 126996)")
    print("=" * 80)

    reassembler = FastPacketReassembler()
    products = {}

    for f in frames:
        if f['pgn'] == 126996:
            result = reassembler.process_frame(f['source'], f['pgn'], f['data'])
            if result and f['source'] not in products:
                products[f['source']] = result

    for addr in sorted(products):
        data = products[addr]
        if len(data) < 8:
            print(f"\n  Source {addr}: (too short: {len(data)} bytes)")
            continue

        # Product info format: NMEA2000 version(2) + product_code(2) +
        # model_id(32 bytes padded) + sw_version(32) + model_version(32) + model_serial(32) +
        # certification_level(1) + load_equivalency(1)
        n2k_ver = struct.unpack_from('<H', data, 0)[0]
        product_code = struct.unpack_from('<H', data, 2)[0]

        def read_str(offset, maxlen=32):
            end = min(offset + maxlen, len(data))
            raw = data[offset:end]
            # Null-terminated or @-padded
            s = raw.split(b'\x00')[0].split(b'\xff')[0]
            s = s.replace(b'@', b' ').strip()
            return s.decode('ascii', errors='replace')

        model_id = read_str(4, 32) if len(data) > 4 else "?"
        sw_ver = read_str(36, 32) if len(data) > 36 else "?"
        model_ver = read_str(68, 32) if len(data) > 68 else "?"
        serial = read_str(100, 32) if len(data) > 100 else "?"

        print(f"\n  Source {addr}:")
        print(f"    Product Code: {product_code}")
        print(f"    Model: {model_id}")
        print(f"    SW Version: {sw_ver}")
        print(f"    Model Version: {model_ver}")
        print(f"    Serial: {serial}")
        print(f"    N2K Version: {n2k_ver}")


def analyze_pgn_by_source(frames):
    """Build source→PGN matrix."""
    print("\n" + "=" * 80)
    print("PGN TRAFFIC BY SOURCE")
    print("=" * 80)

    # source → pgn → count
    traffic = defaultdict(Counter)
    for f in frames:
        traffic[f['source']][f['pgn']] += 1

    for source in sorted(traffic):
        print(f"\n  Source {source}:")
        for pgn, count in traffic[source].most_common():
            name = PGN_NAMES.get(pgn, f"PGN {pgn}")
            print(f"    {pgn:6d} {name:40s} {count:>8,}")


def analyze_126720(frames):
    """Deep analysis of PGN 126720 (Raymarine proprietary) messages."""
    print("\n" + "=" * 80)
    print("PGN 126720 DEEP ANALYSIS (Raymarine Proprietary)")
    print("=" * 80)

    reassembler = FastPacketReassembler()

    # Collect reassembled messages by source
    messages_by_source = defaultdict(list)
    message_types = Counter()  # (source, signature) → count

    for f in frames:
        if f['pgn'] == 126720:
            result = reassembler.process_frame(f['source'], f['pgn'], f['data'])
            if result:
                messages_by_source[f['source']].append({
                    'ts': f['ts'],
                    'data': result,
                })

    for source in sorted(messages_by_source):
        msgs = messages_by_source[source]
        print(f"\n  Source {source}: {len(msgs):,} reassembled messages")

        # Categorize by message signature (first few bytes after manufacturer header)
        signatures = Counter()
        sig_examples = {}
        sig_lengths = defaultdict(Counter)

        for m in msgs:
            data = m['data']
            # Check for Raymarine header
            if len(data) >= 2 and data[0] == 0x3B and data[1] == 0x9F:
                # Signature: bytes 2-5 (after manufacturer header)
                sig = data[2:min(6, len(data))]
                sig_hex = sig.hex()
                signatures[sig_hex] += 1
                sig_lengths[sig_hex][len(data)] += 1
                if sig_hex not in sig_examples or len(sig_examples[sig_hex]) < 5:
                    if sig_hex not in sig_examples:
                        sig_examples[sig_hex] = []
                    sig_examples[sig_hex].append(data)
            else:
                # Non-Raymarine or short
                sig = f"non-ray-{data[:4].hex()}"
                signatures[sig] += 1
                if sig not in sig_examples:
                    sig_examples[sig] = []
                if len(sig_examples[sig]) < 3:
                    sig_examples[sig].append(data)

        print(f"    Message signatures (bytes[2:6] after 3B 9F header):")
        for sig, count in signatures.most_common(30):
            lengths = sig_lengths.get(sig, {})
            len_str = ", ".join(f"{l}B:{c}" for l, c in sorted(lengths.items()))
            print(f"      sig={sig:16s}  count={count:>6,}  lengths=[{len_str}]")

            # Identify known signatures
            if sig.startswith("f0"):
                if sig.startswith("f0818621"):
                    print(f"        → KEYSTROKE command")
                else:
                    print(f"        → Seatalk1 bridge / pilot data")

            # Print examples
            examples = sig_examples.get(sig, [])
            for i, ex in enumerate(examples[:3]):
                print(f"        example[{i}]: {ex.hex(' ')}")


def analyze_126208(frames):
    """Analyze PGN 126208 (NMEA Command Group) — includes mode changes."""
    print("\n" + "=" * 80)
    print("PGN 126208 ANALYSIS (NMEA Command/Request Group)")
    print("=" * 80)

    reassembler = FastPacketReassembler()
    messages_by_source = defaultdict(list)

    for f in frames:
        if f['pgn'] == 126208:
            result = reassembler.process_frame(f['source'], f['pgn'], f['data'])
            if result:
                messages_by_source[f['source']].append({
                    'ts': f['ts'],
                    'data': result,
                })

    for source in sorted(messages_by_source):
        msgs = messages_by_source[source]
        print(f"\n  Source {source}: {len(msgs)} reassembled messages")

        # Categorize by function code (byte 0)
        func_codes = Counter()
        func_examples = defaultdict(list)

        for m in msgs:
            data = m['data']
            if len(data) < 1:
                continue
            fc = data[0]
            # For command (0x01), check target PGN in bytes 1-2
            if fc == 0x01 and len(data) >= 3:
                target_pgn_lo = data[1]
                target_pgn_hi = data[2]
                key = f"CMD→{target_pgn_lo:02x}{target_pgn_hi:02x}"
            elif fc == 0x00:
                key = "REQUEST"
            else:
                key = f"FC={fc:02x}"

            func_codes[key] += 1
            if len(func_examples[key]) < 5:
                func_examples[key].append(data)

        for key, count in func_codes.most_common():
            print(f"    {key}: {count} messages")
            for i, ex in enumerate(func_examples[key][:3]):
                print(f"      example[{i}]: {ex.hex(' ')}")

                # Decode mode change commands (target PGN 65379 = 0xFF63)
                if key == "CMD→63ff" and len(ex) >= 14:
                    mode = ex[12]
                    submode = ex[13] if len(ex) > 13 else 0
                    mode_names = {
                        (0x00, 0x00): "STANDBY",
                        (0x40, 0x00): "AUTO (Compass)",
                        (0x00, 0x01): "WIND",
                        (0x80, 0x01): "TRACK",
                        (0x81, 0x00): "NO DRIFT",
                    }
                    mode_name = mode_names.get((mode, submode), f"mode={mode:#x} sub={submode:#x}")
                    print(f"               → Mode change: {mode_name}")

                # Decode heading set (target PGN 65360 = 0xFF50)
                if key == "CMD→50ff" and len(ex) >= 14:
                    heading_raw = ex[12] | (ex[13] << 8)
                    heading_rad = heading_raw / 10000.0
                    import math
                    heading_deg = math.degrees(heading_rad)
                    print(f"               → Heading set: {heading_deg:.1f}°")


def analyze_seatalk_pgns(frames):
    """Analyze Seatalk proprietary single-frame PGNs."""
    print("\n" + "=" * 80)
    print("SEATALK PROPRIETARY PGNs (single-frame)")
    print("=" * 80)

    seatalk_pgns = {65311, 65345, 65359, 65362, 65379, 65384}

    for target_pgn in sorted(seatalk_pgns):
        pgn_frames = [f for f in frames if f['pgn'] == target_pgn]
        if not pgn_frames:
            continue

        name = PGN_NAMES.get(target_pgn, "Unknown")
        sources = Counter(f['source'] for f in pgn_frames)
        print(f"\n  PGN {target_pgn} ({name}): {len(pgn_frames):,} frames")
        print(f"    Sources: {dict(sources)}")

        # Show unique data patterns (first few)
        patterns = Counter()
        examples = {}
        for f in pgn_frames:
            d = f['data']
            # Check Raymarine header
            if len(d) >= 2 and d[0] == 0x3B and d[1] == 0x9F:
                sig = d[2:].hex()
            else:
                sig = d.hex()
            patterns[sig] += 1
            if sig not in examples:
                examples[sig] = f

        print(f"    Unique patterns: {len(patterns)}")
        for sig, count in patterns.most_common(10):
            ex = examples[sig]
            data_hex = ex['data'].hex(' ')
            print(f"      {data_hex:40s}  count={count:>6,}")

            # Decode specific PGNs
            if target_pgn == 65379 and len(ex['data']) >= 5:
                # Pilot Mode: 3B 9F 01 [mode] [mode_hi] [submode] ...
                d = ex['data']
                if len(d) >= 6:
                    mode = d[3]
                    submode = d[5]
                    mode_names = {
                        (0x00, 0x00): "STANDBY",
                        (0x40, 0x00): "AUTO",
                        (0x00, 0x01): "WIND",
                        (0x80, 0x01): "TRACK",
                    }
                    mn = mode_names.get((mode, submode), f"mode={mode:#x} sub={submode:#x}")
                    print(f"        → Pilot Mode: {mn}")

            if target_pgn == 65359 and len(ex['data']) >= 7:
                # Pilot Heading: 3B 9F [SID] [true_h_lo] [true_h_hi] [mag_h_lo] [mag_h_hi]
                d = ex['data']
                if len(d) >= 7:
                    mag_raw = d[5] | (d[6] << 8)
                    if mag_raw != 0xFFFF:
                        import math
                        heading_deg = math.degrees(mag_raw / 10000.0)
                        print(f"        → Magnetic Heading: {heading_deg:.1f}°")


def analyze_keystroke_commands(frames):
    """Specifically analyze PGN 126720 keystroke commands from p70."""
    print("\n" + "=" * 80)
    print("P70 KEYSTROKE COMMANDS (PGN 126720, signature F0 81 86 21)")
    print("=" * 80)

    reassembler = FastPacketReassembler()
    keystrokes = []

    KEY_NAMES = {
        0x07F8: "+1°",
        0x05FA: "-1°",
        0x08F7: "+10°",
        0x06F9: "-10°",
        0x22DD: "TACK STBD",
        0x21DE: "TACK PORT",
    }

    for f in frames:
        if f['pgn'] == 126720:
            result = reassembler.process_frame(f['source'], f['pgn'], f['data'])
            if result and len(result) >= 8:
                if result[0] == 0x3B and result[1] == 0x9F:
                    if result[2] == 0xF0 and result[3] == 0x81 and result[4] == 0x86 and result[5] == 0x21:
                        key_code = (result[6] << 8) | result[7]
                        key_name = KEY_NAMES.get(key_code, f"Unknown ({key_code:#06x})")
                        keystrokes.append({
                            'ts': f['ts'],
                            'source': f['source'],
                            'key_code': key_code,
                            'key_name': key_name,
                            'data': result,
                        })

    print(f"\n  Total keystroke commands: {len(keystrokes)}")

    if keystrokes:
        key_counts = Counter(k['key_name'] for k in keystrokes)
        print(f"  Key distribution:")
        for name, count in key_counts.most_common():
            print(f"    {name:20s} {count:>6}")

        sources = Counter(k['source'] for k in keystrokes)
        print(f"  Sources: {dict(sources)}")

        # Show chronological log of keystrokes
        print(f"\n  Keystroke timeline (first 50):")
        for k in keystrokes[:50]:
            t = k['ts']
            import datetime
            ts_str = datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
            print(f"    {ts_str} src={k['source']:3d} {k['key_name']:15s} data={k['data'].hex(' ')}")

        # Check for unknown key codes
        unknown = [k for k in keystrokes if k['key_name'].startswith("Unknown")]
        if unknown:
            print(f"\n  UNKNOWN key codes ({len(unknown)}):")
            unknown_codes = Counter(k['key_code'] for k in unknown)
            for code, count in unknown_codes.most_common():
                ex = next(k for k in unknown if k['key_code'] == code)
                print(f"    {code:#06x}: {count} occurrences, example: {ex['data'].hex(' ')}")


def main():
    log_dir = Path("/Users/boston/ieb/autopilot/n2klogs/logged")
    bin_files = sorted(log_dir.glob("*.bin"))

    if not bin_files:
        print("No .bin files found")
        return

    all_frames = []
    for path in bin_files:
        frames = read_log(path)
        if frames:
            all_frames.extend(frames)

    print(f"\nTotal frames across all files: {len(all_frames):,}")

    # Run all analyses
    devices = analyze_devices(all_frames)
    analyze_product_info(all_frames)
    analyze_pgn_by_source(all_frames)
    analyze_126720(all_frames)
    analyze_126208(all_frames)
    analyze_seatalk_pgns(all_frames)
    analyze_keystroke_commands(all_frames)


if __name__ == "__main__":
    main()
