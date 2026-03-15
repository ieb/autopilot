#!/usr/bin/env python3
"""
Deep reverse-engineering of EV-1 PGN 126720 messages and correlation with
p70 REQUEST patterns.
"""

import struct
import math
from collections import defaultdict, Counter
from pathlib import Path
import datetime

HEADER_STRUCT = struct.Struct('<4sIII d 8x')
FRAME_STRUCT = struct.Struct('<II BB BB 8s I')


class FastPacketReassembler:
    def __init__(self):
        self.sessions = {}

    def process_frame(self, source, pgn, data):
        if len(data) < 2:
            return None
        counter = data[0]
        frame_num = counter & 0x1F
        seq = (counter >> 5) & 0x07
        key = (source, pgn)
        if frame_num == 0:
            total_len = data[1]
            payload = data[2:8]
            self.sessions[key] = {
                'seq': seq, 'total_len': total_len,
                'data': bytearray(payload[:min(6, total_len)]),
                'expected_frame': 1,
            }
            if total_len <= 6:
                result = bytes(self.sessions[key]['data'][:total_len])
                del self.sessions[key]
                return result
            return None
        else:
            session = self.sessions.get(key)
            if session is None or seq != session['seq'] or frame_num != session['expected_frame']:
                if key in self.sessions:
                    del self.sessions[key]
                return None
            session['data'].extend(data[1:8])
            session['expected_frame'] = frame_num + 1
            if len(session['data']) >= session['total_len']:
                result = bytes(session['data'][:session['total_len']])
                del self.sessions[key]
                return result
            return None


def read_all_frames(log_dir):
    frames = []
    for path in sorted(log_dir.glob("*.bin")):
        with open(path, 'rb') as f:
            header = f.read(32)
            magic, version, flags, _, start_ts = HEADER_STRUCT.unpack(header)
            if magic != b'CANL':
                continue
            while True:
                raw = f.read(24)
                if len(raw) < 24:
                    break
                ts_offset, can_id, dlc, fflags, source, priority, data, pgn = FRAME_STRUCT.unpack(raw)
                frames.append({
                    'ts': start_ts + ts_offset * 0.0001,
                    'can_id': can_id,
                    'dlc': dlc,
                    'source': source,
                    'data': data[:dlc],
                    'pgn': pgn,
                })
    return frames


def reassemble_pgn(frames, target_pgn, target_source=None):
    """Reassemble all messages for a specific PGN/source."""
    r = FastPacketReassembler()
    messages = []
    for f in frames:
        if f['pgn'] != target_pgn:
            continue
        if target_source is not None and f['source'] != target_source:
            continue
        result = r.process_frame(f['source'], f['pgn'], f['data'])
        if result:
            messages.append({'ts': f['ts'], 'source': f['source'], 'data': result})
    return messages


def ts_str(ts):
    return datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3]


def main():
    log_dir = Path("/Users/boston/ieb/autopilot/n2klogs/logged")
    frames = read_all_frames(log_dir)
    print(f"Total frames: {len(frames):,}")

    # =========================================================================
    # 1. Reassemble ALL EV-1 PGN 126720 messages
    # =========================================================================
    ev1_msgs = reassemble_pgn(frames, 126720, target_source=205)
    print(f"\nEV-1 (Source 205) PGN 126720: {len(ev1_msgs):,} reassembled messages")

    # =========================================================================
    # 2. Categorize by sub-type byte (byte[4] after 3b 9f f0 88)
    # =========================================================================
    print("\n" + "=" * 80)
    print("EV-1 PGN 126720 SUB-TYPE CATALOGUE")
    print("=" * 80)

    by_subtype = defaultdict(list)
    for m in ev1_msgs:
        d = m['data']
        if len(d) >= 6 and d[0] == 0x3B and d[1] == 0x9F:
            # Key: bytes 2-5
            sig = d[2:6]
            by_subtype[sig.hex()].append(m)
        else:
            by_subtype['other'].append(m)

    # Print overview sorted by signature
    print(f"\n{'Signature':<16s} {'Count':>6s} {'Lengths':>12s}  Description")
    print("-" * 70)
    for sig in sorted(by_subtype):
        msgs = by_subtype[sig]
        lengths = Counter(len(m['data']) for m in msgs)
        len_str = ",".join(f"{l}" for l in sorted(lengths))
        # Compute rate
        if len(msgs) > 1:
            total_time = msgs[-1]['ts'] - msgs[0]['ts']
            rate = len(msgs) / total_time if total_time > 0 else 0
        else:
            rate = 0
        print(f"  {sig:<14s} {len(msgs):>6,}  len={len_str:<8s}  ~{rate:.1f} Hz")

    # =========================================================================
    # 3. Deep decode f088XX00 sub-types (the bulk of traffic)
    # =========================================================================
    print("\n" + "=" * 80)
    print("DETAILED ANALYSIS OF f088XX00 SUB-TYPES")
    print("=" * 80)

    # Group f088 messages by sub-type byte
    f088_types = {}
    for sig, msgs in by_subtype.items():
        if sig.startswith("f088") and len(sig) == 8:
            subtype_byte = int(sig[4:6], 16)
            f088_types[subtype_byte] = msgs

    for subtype in sorted(f088_types):
        msgs = f088_types[subtype]
        print(f"\n  --- Sub-type 0x{subtype:02X} ({len(msgs)} messages) ---")

        # Group by length
        by_len = defaultdict(list)
        for m in msgs:
            by_len[len(m['data'])].append(m)

        for length in sorted(by_len):
            examples = by_len[length]
            print(f"\n    Length {length} bytes ({len(examples)} messages):")

            # Analyze which bytes are constant vs variable
            if len(examples) >= 2:
                # Check each byte position
                byte_analysis = []
                for pos in range(length):
                    values = set(m['data'][pos] for m in examples[:100])
                    if len(values) == 1:
                        byte_analysis.append(f"C({list(values)[0]:02x})")
                    elif len(values) <= 5:
                        byte_analysis.append(f"V({len(values)})")
                    else:
                        all_vals = [m['data'][pos] for m in examples[:100]]
                        mn, mx = min(all_vals), max(all_vals)
                        byte_analysis.append(f"V({mn:02x}-{mx:02x})")

                # Print byte map
                # Header is always bytes 0-5: 3b 9f f0 88 XX 00
                print(f"    Byte map (C=constant, V=variable):")
                for i in range(0, length, 8):
                    chunk = byte_analysis[i:i+8]
                    positions = " ".join(f"[{i+j:2d}]" for j in range(len(chunk)))
                    values = " ".join(f"{v:>5s}" for v in chunk)
                    print(f"      {positions}")
                    print(f"      {values}")

            # Show first 3 examples
            for i, m in enumerate(examples[:3]):
                print(f"    ex[{i}]: {m['data'].hex(' ')}")

            # Try to decode 27-byte messages (most common length)
            if length == 27 and len(examples) >= 10:
                print(f"\n    Attempting field decode for 27-byte messages:")
                print(f"    Bytes 0-5: Header (3b 9f f0 88 {subtype:02x} 00)")
                print(f"    Byte 6: appears to be a length/format indicator")

                # Byte 6 is often 0x17 (=23, matching remaining payload length 27-4=23? or 27-6=21?)
                b6_values = Counter(m['data'][6] for m in examples)
                print(f"    Byte 6 values: {dict(b6_values.most_common(5))}")

                # Bytes 7 onwards — try interpreting as int16 LE pairs
                print(f"\n    Attempting int16 LE decode (bytes 7-26):")
                for i, m in enumerate(examples[:5]):
                    d = m['data']
                    fields = []
                    for pos in range(7, min(length, 27), 2):
                        if pos + 1 < length:
                            val = struct.unpack_from('<h', d, pos)[0]
                            fields.append(f"{val:+6d}")
                    print(f"    ex[{i}]: {' '.join(fields)}")

                # Try to identify known values by checking against concurrent sensor data
                # Compare with heading from PGN 65359 at same timestamp
                print(f"\n    Cross-reference with concurrent heading (PGN 65359):")

    # =========================================================================
    # 4. Correlate p70 REQUESTs with EV-1 126720 responses
    # =========================================================================
    print("\n" + "=" * 80)
    print("P70 REQUEST → EV-1 RESPONSE CORRELATION")
    print("=" * 80)

    # Get p70 requests (126208 FC=0x00 for PGN 126720)
    p70_requests_126208 = reassemble_pgn(frames, 126208, target_source=0)
    p70_reqs = [m for m in p70_requests_126208
                if len(m['data']) >= 4 and m['data'][0] == 0x00
                and (m['data'][1] | (m['data'][2] << 8) | (m['data'][3] << 16)) == 126720]

    print(f"\np70 requests for PGN 126720: {len(p70_reqs)}")

    # Decode the two request patterns
    print("\nRequest patterns:")
    req_patterns = Counter(m['data'].hex() for m in p70_reqs)
    for pattern, count in req_patterns.most_common():
        raw = bytes.fromhex(pattern)
        print(f"\n  [{count}x] {raw.hex(' ')}")

        # The request has parameters that specify which data the p70 wants
        # Format: FC(1) + PGN(3) + interval(4) + offset(2) + num_params(1) + params...
        if len(raw) >= 11:
            num_params = raw[10]
            print(f"    num_params={num_params}")
            pos = 11
            for p in range(num_params):
                if pos < len(raw):
                    idx = raw[pos]
                    pos += 1
                    # Value bytes — for Seatalk proprietary, values are typically 2 bytes
                    if pos + 1 < len(raw):
                        val = raw[pos] | (raw[pos+1] << 8)
                        val_hex = raw[pos:pos+2].hex(' ')
                        pos += 2
                        print(f"    param[{p}]: index={idx} ({idx:#04x})  value={val} ({val:#06x})  raw={val_hex}")

    # For each p70 request, find what EV-1 126720 messages appear within 200ms after
    print("\n\nCorrelating requests with responses (first 30):")
    for req in p70_reqs[:30]:
        req_time = req['ts']
        req_last_bytes = req['data'][-2:].hex(' ')

        # Find EV-1 126720 messages within 200ms after request
        responses = []
        for m in ev1_msgs:
            dt = m['ts'] - req_time
            if dt > 0 and dt < 0.2:
                responses.append(m)
            elif dt >= 0.2:
                break

        resp_sigs = [m['data'][2:6].hex() if len(m['data']) >= 6 else '?' for m in responses]
        resp_count = len(responses)

        print(f"  {ts_str(req_time)} req[...{req_last_bytes}] → {resp_count} EV-1 msgs: {resp_sigs[:8]}")

    # =========================================================================
    # 5. Focus on f081ae02 (the second most common non-f088 type)
    # =========================================================================
    if 'f081ae02' in by_subtype:
        print("\n" + "=" * 80)
        print("ANALYSIS OF f081ae02 (9 bytes, 1729 messages)")
        print("=" * 80)

        msgs = by_subtype['f081ae02']
        print(f"\n  Total: {len(msgs)} messages")
        if len(msgs) > 1:
            total_time = msgs[-1]['ts'] - msgs[0]['ts']
            print(f"  Rate: ~{len(msgs)/total_time:.1f} Hz")

        patterns = Counter(m['data'].hex() for m in msgs)
        print(f"  Unique patterns: {len(patterns)}")
        for pat, count in patterns.most_common(15):
            raw = bytes.fromhex(pat)
            # Bytes: 3b 9f f0 81 ae 02 XX YY ZZ
            if len(raw) >= 9:
                b6, b7, b8 = raw[6], raw[7], raw[8]
                print(f"    [{count:>4}x] {raw.hex(' ')}  bytes[6:9]={b6:02x} {b7:02x} {b8:02x}")
                # Try as status byte + int16
                val = struct.unpack_from('<h', raw, 7)[0] if len(raw) >= 9 else 0
                print(f"            byte6={b6:#04x}  int16[7:9]={val}")

    # =========================================================================
    # 6. p70's own 126720 messages (f0819000)
    # =========================================================================
    if 'f0819000' in by_subtype:
        print("\n" + "=" * 80)
        print("P70 (Source 0) PGN 126720 — f0819000")
        print("=" * 80)

        p70_own = reassemble_pgn(frames, 126720, target_source=0)
        p70_f081 = [m for m in p70_own if len(m['data']) >= 6
                    and m['data'][2:6] == bytes([0xf0, 0x81, 0x90, 0x00])]
        print(f"\n  Total: {len(p70_f081)} messages")

        patterns = Counter(m['data'].hex() for m in p70_f081)
        for pat, count in patterns.most_common(5):
            print(f"    [{count}x] {bytes.fromhex(pat).hex(' ')}")

        if len(p70_f081) > 1:
            total_time = p70_f081[-1]['ts'] - p70_f081[0]['ts']
            print(f"  Rate: ~{len(p70_f081)/total_time:.1f} Hz")

    # =========================================================================
    # 7. Concurrent heading cross-reference for field identification
    # =========================================================================
    print("\n" + "=" * 80)
    print("FIELD IDENTIFICATION: Cross-reference with known sensor values")
    print("=" * 80)

    # Get PGN 65359 (Seatalk Pilot Heading) - these contain known heading values
    heading_frames = [f for f in frames if f['pgn'] == 65359 and f['source'] == 205]
    # Get PGN 127250 (Vessel Heading)
    vessel_heading_frames = [f for f in frames if f['pgn'] == 127250 and f['source'] == 205]
    # Get PGN 127245 (Rudder)
    rudder_frames = [f for f in frames if f['pgn'] == 127245]
    # Get PGN 130306 (Wind)
    wind_frames = [f for f in frames if f['pgn'] == 130306]

    # Build time-indexed lookup for headings
    heading_by_time = {}
    for f in vessel_heading_frames:
        d = f['data']
        if len(d) >= 4:
            raw = d[1] | (d[2] << 8)  # SID + heading (UDouble*0.0001 rad)
            raw = d[2] | (d[3] << 8)  # heading is bytes 2-3 after SID
            # Actually for PGN 127250: SID(1) + Heading(2, UDouble*0.0001 rad) + ...
            heading_raw = struct.unpack_from('<H', d, 1)[0]
            if heading_raw != 0xFFFF:
                heading_deg = math.degrees(heading_raw * 0.0001)
                heading_by_time[round(f['ts'], 1)] = heading_deg

    # For the most common 27-byte f088 sub-type, try to find heading in the data
    # Use sub-type 0x30 as reference (first one)
    if 0x30 in f088_types:
        msgs_30 = [m for m in f088_types[0x30] if len(m['data']) == 27]
        print(f"\n  Using sub-type 0x30 ({len(msgs_30)} messages at 27 bytes) for field identification")
        print(f"\n  Cross-referencing with vessel heading (PGN 127250):")
        print(f"  {'Time':>12s}  {'Heading°':>8s}  {'Bytes 7-26 as int16 LE':s}")

        matched = 0
        for m in msgs_30[:20]:
            t_key = round(m['ts'], 1)
            heading = heading_by_time.get(t_key)
            if heading is None:
                # Try nearby timestamps
                for dt in [-0.1, 0.1, -0.2, 0.2]:
                    heading = heading_by_time.get(round(m['ts'] + dt, 1))
                    if heading is not None:
                        break

            d = m['data']
            fields_i16 = []
            for pos in range(7, 27, 2):
                if pos + 1 < len(d):
                    val = struct.unpack_from('<h', d, pos)[0]
                    fields_i16.append(val)

            heading_str = f"{heading:8.1f}" if heading is not None else "   N/A  "
            fields_str = " ".join(f"{v:+6d}" for v in fields_i16)
            print(f"  {ts_str(m['ts'])}  {heading_str}  {fields_str}")

            # Check if any field matches heading * 10000 / (180/pi) = heading_rad * 10000
            if heading is not None:
                heading_rad_10k = int(math.radians(heading) * 10000)
                for i, v in enumerate(fields_i16):
                    uv = v & 0xFFFF  # unsigned
                    if abs(uv - heading_rad_10k) < 10:
                        print(f"    *** Field {i} (byte {7+i*2}) matches heading! uval={uv} vs hdg_raw={heading_rad_10k}")
                matched += 1

    # =========================================================================
    # 8. Try to decode the 67-byte messages (less common but potentially richer)
    # =========================================================================
    print("\n" + "=" * 80)
    print("67-BYTE f088 MESSAGES (adaptive/calibration data?)")
    print("=" * 80)

    for subtype in sorted(f088_types):
        msgs = f088_types[subtype]
        long_msgs = [m for m in msgs if len(m['data']) == 67]
        if long_msgs:
            print(f"\n  Sub-type 0x{subtype:02X}: {len(long_msgs)} messages at 67 bytes")
            # These likely contain float32 values (calibration, gains, etc.)
            for i, m in enumerate(long_msgs[:2]):
                d = m['data']
                print(f"    ex[{i}]: {d.hex(' ')}")
                # Try decoding bytes 6+ as float32 LE
                print(f"    As float32 LE (from byte 6):")
                floats = []
                for pos in range(6, len(d) - 3, 4):
                    val = struct.unpack_from('<f', d, pos)[0]
                    if abs(val) < 1e10 and val != 0:
                        floats.append((pos, val))
                for pos, val in floats:
                    print(f"      byte[{pos:2d}]: {val:12.4f}")

    # =========================================================================
    # 9. Analyze what changes between pilot modes in 126720 traffic
    # =========================================================================
    print("\n" + "=" * 80)
    print("126720 TRAFFIC CHANGES BY PILOT MODE")
    print("=" * 80)

    # Get mode changes from p70 commands
    mode_msgs = reassemble_pgn(frames, 126208, target_source=0)
    mode_changes = []
    for m in mode_msgs:
        d = m['data']
        if len(d) >= 14 and d[0] == 0x01:
            pgn = d[1] | (d[2] << 8) | (d[3] << 16)
            if pgn == 65379:
                mode = d[12]
                submode = d[13]
                mode_map = {(0x00,0x00): "STANDBY", (0x40,0x00): "AUTO",
                            (0x00,0x01): "WIND", (0x80,0x01): "TRACK"}
                name = mode_map.get((mode, submode), f"0x{mode:02x}/0x{submode:02x}")
                mode_changes.append({'ts': m['ts'], 'mode': name})

    if mode_changes:
        print(f"\n  Mode changes detected: {len(mode_changes)}")
        for mc in mode_changes:
            print(f"    {ts_str(mc['ts'])} → {mc['mode']}")

        # For each mode period, check which f088 sub-types appear
        for i, mc in enumerate(mode_changes):
            start = mc['ts']
            end = mode_changes[i+1]['ts'] if i+1 < len(mode_changes) else start + 30.0
            period_msgs = [m for m in ev1_msgs if start <= m['ts'] < end]

            sigs = Counter()
            for m in period_msgs:
                if len(m['data']) >= 6:
                    sigs[m['data'][2:6].hex()] += 1

            print(f"\n    Mode: {mc['mode']} ({ts_str(start)} to {ts_str(end)})")
            print(f"    126720 messages in period: {len(period_msgs)}")
            # Show any new signatures or changed rates
            for sig, count in sigs.most_common(5):
                duration = end - start
                rate = count / duration if duration > 0 else 0
                print(f"      {sig}: {count} msgs ({rate:.1f} Hz)")


if __name__ == "__main__":
    main()
