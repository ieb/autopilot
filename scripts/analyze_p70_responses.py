#!/usr/bin/env python3
"""
Decode the specific PGN 126720 response messages that the p70 polls for.
Focus on 6c1a and 6c23 sub-types, and the f081ae02 pilot status heartbeat.
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
                    'source': source,
                    'data': data[:dlc],
                    'pgn': pgn,
                })
    return frames


def ts_str(ts):
    return datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3]


def main():
    log_dir = Path("/Users/boston/ieb/autopilot/n2klogs/logged")
    frames = read_all_frames(log_dir)
    print(f"Total frames: {len(frames):,}")

    # Reassemble EV-1 PGN 126720
    r = FastPacketReassembler()
    ev1_126720 = []
    for f in frames:
        if f['pgn'] == 126720 and f['source'] == 205:
            result = r.process_frame(f['source'], f['pgn'], f['data'])
            if result:
                ev1_126720.append({'ts': f['ts'], 'data': result})

    # Also reassemble p70 PGN 126720
    r2 = FastPacketReassembler()
    p70_126720 = []
    for f in frames:
        if f['pgn'] == 126720 and f['source'] == 0:
            result = r2.process_frame(f['source'], f['pgn'], f['data'])
            if result:
                p70_126720.append({'ts': f['ts'], 'data': result})

    # Also reassemble ACU (182) PGN 126720
    r3 = FastPacketReassembler()
    acu_126720 = []
    for f in frames:
        if f['pgn'] == 126720 and f['source'] == 182:
            result = r3.process_frame(f['source'], f['pgn'], f['data'])
            if result:
                acu_126720.append({'ts': f['ts'], 'data': result})

    # =========================================================================
    # 1. Decode 6c1a5086 responses (13 bytes)
    # =========================================================================
    print("\n" + "=" * 80)
    print("EV-1 RESPONSE TYPE 6c1a (requested by p70 pattern A)")
    print("=" * 80)

    resp_1a = [m for m in ev1_126720
               if len(m['data']) >= 6 and m['data'][2:4] == bytes([0x6c, 0x1a])]
    print(f"\n  Total: {len(resp_1a)} messages")

    if resp_1a:
        # Show unique patterns
        patterns = Counter(m['data'].hex() for m in resp_1a)
        print(f"  Unique patterns: {len(patterns)}")
        for pat, count in patterns.most_common(20):
            raw = bytes.fromhex(pat)
            print(f"    [{count:>3}x] {raw.hex(' ')}")

        # Analyze byte-by-byte
        print(f"\n  Byte analysis (13 bytes):")
        for pos in range(len(resp_1a[0]['data'])):
            values = [m['data'][pos] for m in resp_1a if pos < len(m['data'])]
            unique = set(values)
            if len(unique) == 1:
                print(f"    byte[{pos:2d}]: CONSTANT 0x{list(unique)[0]:02x}")
            elif len(unique) <= 10:
                cnt = Counter(values)
                print(f"    byte[{pos:2d}]: {len(unique)} values: {dict(cnt.most_common(10))}")
            else:
                mn, mx = min(values), max(values)
                print(f"    byte[{pos:2d}]: VARIABLE range 0x{mn:02x}-0x{mx:02x} ({len(unique)} unique)")

        # Try interpreting fields
        print(f"\n  Field decode attempts:")
        for m in resp_1a[:10]:
            d = m['data']
            # 3b 9f 6c 1a 50 86 XX XX XX XX XX XX XX
            # Bytes 4-5: 50 86
            # Bytes 6-12: payload
            if len(d) >= 13:
                b4_5 = f"{d[4]:02x} {d[5]:02x}"
                # Try bytes 6-7 as uint16
                u16_6 = struct.unpack_from('<H', d, 6)[0]
                # Try bytes 8-9 as uint16
                u16_8 = struct.unpack_from('<H', d, 8)[0]
                # Try bytes 10-11 as uint16
                u16_10 = struct.unpack_from('<H', d, 10)[0]
                # Try byte 12
                b12 = d[12]

                # Heading as radians*10000?
                hdg_6 = math.degrees(u16_6 * 0.0001) if u16_6 < 0xFFFE else None
                hdg_8 = math.degrees(u16_8 * 0.0001) if u16_8 < 0xFFFE else None
                hdg_10 = math.degrees(u16_10 * 0.0001) if u16_10 < 0xFFFE else None

                print(f"    {ts_str(m['ts'])} [{b4_5}] u16@6={u16_6:5d}({hdg_6:6.1f}° if hdg) u16@8={u16_8:5d}({hdg_8:6.1f}° if hdg) u16@10={u16_10:5d}({hdg_10:6.1f}° if hdg) b12={b12:02x}")

    # =========================================================================
    # 2. Decode 6c235064 responses (9 bytes)
    # =========================================================================
    print("\n" + "=" * 80)
    print("EV-1 RESPONSE TYPE 6c23 (requested by p70 pattern B)")
    print("=" * 80)

    resp_23 = [m for m in ev1_126720
               if len(m['data']) >= 6 and m['data'][2:4] == bytes([0x6c, 0x23])]
    print(f"\n  Total: {len(resp_23)} messages")

    if resp_23:
        patterns = Counter(m['data'].hex() for m in resp_23)
        print(f"  Unique patterns: {len(patterns)}")
        for pat, count in patterns.most_common(20):
            raw = bytes.fromhex(pat)
            print(f"    [{count:>3}x] {raw.hex(' ')}")

        # Byte analysis
        print(f"\n  Byte analysis (9 bytes):")
        for pos in range(len(resp_23[0]['data'])):
            values = [m['data'][pos] for m in resp_23 if pos < len(m['data'])]
            unique = set(values)
            if len(unique) == 1:
                print(f"    byte[{pos:2d}]: CONSTANT 0x{list(unique)[0]:02x}")
            elif len(unique) <= 10:
                cnt = Counter(values)
                print(f"    byte[{pos:2d}]: {len(unique)} values: {dict(cnt.most_common(10))}")
            else:
                mn, mx = min(values), max(values)
                print(f"    byte[{pos:2d}]: VARIABLE range 0x{mn:02x}-0x{mx:02x} ({len(unique)} unique)")

    # =========================================================================
    # 3. Other 6c prefix messages
    # =========================================================================
    print("\n" + "=" * 80)
    print("ALL 6c-PREFIX RESPONSES FROM EV-1")
    print("=" * 80)

    resp_6c = [m for m in ev1_126720
               if len(m['data']) >= 4 and m['data'][2] == 0x6c]
    print(f"\n  Total 6c-prefix messages: {len(resp_6c)}")

    sigs = Counter()
    sig_examples = {}
    for m in resp_6c:
        sig = m['data'][2:6].hex()
        sigs[sig] += 1
        if sig not in sig_examples:
            sig_examples[sig] = m['data']

    for sig, count in sigs.most_common():
        ex = sig_examples[sig]
        print(f"  [{count:>4}x] sig={sig}  len={len(ex)}  data={ex.hex(' ')}")

    # =========================================================================
    # 4. f081ae02 pilot status heartbeat — detailed decode
    # =========================================================================
    print("\n" + "=" * 80)
    print("f081ae02 PILOT STATUS HEARTBEAT")
    print("=" * 80)

    hb = [m for m in ev1_126720
          if len(m['data']) >= 9 and m['data'][2:6] == bytes([0xf0, 0x81, 0xae, 0x02])]
    print(f"\n  Total: {len(hb)} messages at ~1 Hz")

    # Correlate byte[6] with pilot mode
    # Get mode timeline
    r4 = FastPacketReassembler()
    mode_msgs = []
    for f in frames:
        if f['pgn'] == 126208 and f['source'] == 0:
            result = r4.process_frame(f['source'], f['pgn'], f['data'])
            if result and len(result) >= 14 and result[0] == 0x01:
                pgn = result[1] | (result[2] << 8) | (result[3] << 16)
                if pgn == 65379:
                    mode = result[12]
                    submode = result[13]
                    mode_map = {(0x00,0x00): "STANDBY", (0x40,0x00): "AUTO",
                                (0x00,0x01): "WIND", (0x80,0x01): "TRACK"}
                    name = mode_map.get((mode, submode), f"0x{mode:02x}/0x{submode:02x}")
                    mode_msgs.append({'ts': f['ts'], 'mode': name})

    # For each heartbeat, find what mode was active
    mode_idx = 0
    current_mode = "STANDBY"
    b6_by_mode = defaultdict(Counter)

    for h in hb:
        while mode_idx < len(mode_msgs) and mode_msgs[mode_idx]['ts'] <= h['ts']:
            current_mode = mode_msgs[mode_idx]['mode']
            mode_idx += 1
        b6 = h['data'][6]
        b7_8 = struct.unpack_from('<H', h['data'], 7)[0] if len(h['data']) >= 9 else 0
        b6_by_mode[current_mode][b6] += 1

    print(f"\n  Byte[6] by pilot mode:")
    for mode, counter in sorted(b6_by_mode.items()):
        values = dict(counter.most_common())
        values_hex = {f"0x{k:02x}": v for k, v in values.items()}
        print(f"    {mode:10s}: {values_hex}")

    # Also decode bytes 7-8
    print(f"\n  Bytes[7:9] analysis:")
    b78_values = Counter(struct.unpack_from('<H', m['data'], 7)[0] for m in hb if len(m['data']) >= 9)
    for val, count in b78_values.most_common(10):
        print(f"    0x{val:04x} ({val:5d}): {count} times")

    # =========================================================================
    # 5. f0819c and f08184 families (mode-dependent)
    # =========================================================================
    print("\n" + "=" * 80)
    print("MODE-DEPENDENT MESSAGE FAMILIES")
    print("=" * 80)

    for prefix_name, prefix_bytes in [("f0819c", bytes([0xf0, 0x81, 0x9c])),
                                       ("f08184", bytes([0xf0, 0x81, 0x84])),
                                       ("f0817f", bytes([0xf0, 0x81, 0x7f])),
                                       ("f0819010", bytes([0xf0, 0x81, 0x90, 0x10]))]:
        msgs = [m for m in ev1_126720
                if len(m['data']) >= len(prefix_bytes) + 2
                and m['data'][2:2+len(prefix_bytes)] == prefix_bytes]
        if not msgs:
            continue

        print(f"\n  {prefix_name}: {len(msgs)} messages")

        # Show unique patterns
        patterns = Counter(m['data'].hex() for m in msgs)
        for pat, count in patterns.most_common(10):
            raw = bytes.fromhex(pat)
            # Find what mode was active for this pattern
            ex = next(m for m in msgs if m['data'].hex() == pat)
            mode_idx_2 = 0
            mode_2 = "STANDBY"
            for mc in mode_msgs:
                if mc['ts'] <= ex['ts']:
                    mode_2 = mc['mode']
            print(f"    [{count:>4}x] {raw.hex(' ')}  (during {mode_2})")

    # =========================================================================
    # 6. P70's own 126720: f0819000 — detailed
    # =========================================================================
    print("\n" + "=" * 80)
    print("P70 OWN 126720: f0819000")
    print("=" * 80)

    p70_status = [m for m in p70_126720
                  if len(m['data']) >= 6 and m['data'][2:6] == bytes([0xf0, 0x81, 0x90, 0x00])]
    print(f"\n  Total: {len(p70_status)} messages")
    patterns = Counter(m['data'].hex() for m in p70_status)
    for pat, count in patterns.most_common():
        raw = bytes.fromhex(pat)
        print(f"    [{count}x] {raw.hex(' ')}")
    if len(p70_status) > 1:
        total_time = p70_status[-1]['ts'] - p70_status[0]['ts']
        print(f"  Rate: ~{len(p70_status)/total_time:.1f} Hz")

    # Correlate with mode
    b6_by_mode2 = defaultdict(Counter)
    mode_idx = 0
    current_mode = "STANDBY"
    for m in p70_status:
        while mode_idx < len(mode_msgs) and mode_msgs[mode_idx]['ts'] <= m['ts']:
            current_mode = mode_msgs[mode_idx]['mode']
            mode_idx += 1
        b6 = m['data'][6]
        b6_by_mode2[current_mode][b6] += 1

    print(f"\n  Byte[6] by pilot mode:")
    for mode, counter in sorted(b6_by_mode2.items()):
        values = {f"0x{k:02x}": v for k, v in counter.most_common()}
        print(f"    {mode:10s}: {values}")

    # =========================================================================
    # 7. ACU's 126720 messages
    # =========================================================================
    print("\n" + "=" * 80)
    print("ACU (182) 126720 MESSAGES")
    print("=" * 80)

    print(f"\n  Total: {len(acu_126720)} messages")
    for m in acu_126720[:5]:
        print(f"    {ts_str(m['ts'])} {m['data'].hex(' ')}")

    sigs = Counter(m['data'][2:6].hex() if len(m['data']) >= 6 else '?' for m in acu_126720)
    print(f"\n  Signatures:")
    for sig, count in sigs.most_common():
        print(f"    {sig}: {count}")

    # =========================================================================
    # 8. Timeline: mode change → what 126720 traffic changes
    # =========================================================================
    print("\n" + "=" * 80)
    print("TIMELINE: MODE CHANGE AND 126720 TRAFFIC")
    print("=" * 80)

    # Show first AUTO engage sequence in detail
    auto_start = next((mc for mc in mode_msgs if mc['mode'] == 'AUTO'), None)
    if auto_start:
        t0 = auto_start['ts']
        print(f"\n  AUTO mode engaged at {ts_str(t0)}")
        print(f"  126720 messages in first 5 seconds after AUTO:")

        for m in ev1_126720:
            dt = m['ts'] - t0
            if dt < -1:
                continue
            if dt > 5:
                break
            sig = m['data'][2:6].hex() if len(m['data']) >= 6 else '?'
            # Only show non-f088 messages (those are continuous)
            if not sig.startswith('f088'):
                print(f"    {ts_str(m['ts'])} [{dt:+6.3f}s] sig={sig} len={len(m['data'])} data={m['data'].hex(' ')}")

    # Show WIND engage
    wind_start = next((mc for mc in mode_msgs if mc['mode'] == 'WIND'), None)
    if wind_start:
        t0 = wind_start['ts']
        print(f"\n  WIND mode engaged at {ts_str(t0)}")
        print(f"  Non-f088 126720 messages in first 5 seconds after WIND:")

        for m in ev1_126720:
            dt = m['ts'] - t0
            if dt < -1:
                continue
            if dt > 5:
                break
            sig = m['data'][2:6].hex() if len(m['data']) >= 6 else '?'
            if not sig.startswith('f088'):
                print(f"    {ts_str(m['ts'])} [{dt:+6.3f}s] sig={sig} len={len(m['data'])} data={m['data'].hex(' ')}")


if __name__ == "__main__":
    main()
