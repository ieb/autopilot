#!/usr/bin/env python3
"""
Cross-reference EV-1 f088XX00 27-byte messages with known sensor data
to identify what each int16 field represents.

Known EV-1 standard PGNs for ground truth:
  127250 - Vessel Heading (magnetic, radians)
  127251 - Rate of Turn (radians/s)
  127257 - Attitude (yaw/pitch/roll, radians)
  65359  - Pilot Heading (Seatalk proprietary)
  127245 - Rudder
"""

import struct
import math
from collections import defaultdict
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


def parse_n2k_heading(data):
    """Parse PGN 127250: SID(1) + Heading(U16*0.0001rad) + Dev(S16) + Var(S16) + Ref(1)"""
    if len(data) < 4:
        return None
    raw = struct.unpack_from('<H', data, 1)[0]
    if raw == 0xFFFF:
        return None
    return math.degrees(raw * 0.0001)


def parse_n2k_rot(data):
    """Parse PGN 127251: SID(1) + RateOfTurn(S32*3.125e-8 rad/s)"""
    if len(data) < 5:
        return None
    raw = struct.unpack_from('<i', data, 1)[0]
    if raw == 0x7FFFFFFF:
        return None
    return math.degrees(raw * 3.125e-8)  # deg/s


def parse_n2k_attitude(data):
    """Parse PGN 127257: SID(1) + Yaw(S16*0.0001rad) + Pitch(S16*0.0001rad) + Roll(S16*0.0001rad)"""
    if len(data) < 7:
        return None
    yaw_raw, pitch_raw, roll_raw = struct.unpack_from('<hhh', data, 1)
    result = {}
    if yaw_raw != 0x7FFF:
        result['yaw'] = math.degrees(yaw_raw * 0.0001)
    if pitch_raw != 0x7FFF:
        result['pitch'] = math.degrees(pitch_raw * 0.0001)
    if roll_raw != 0x7FFF:
        result['roll'] = math.degrees(roll_raw * 0.0001)
    return result if result else None


def parse_n2k_rudder(data):
    """Parse PGN 127245: Instance(1) + DirOrder(1) + AngleOrder(S16*0.0001rad) + Position(S16*0.0001rad)"""
    if len(data) < 6:
        return None
    angle_raw = struct.unpack_from('<h', data, 2)[0]
    pos_raw = struct.unpack_from('<h', data, 4)[0]
    result = {}
    if angle_raw != 0x7FFF:
        result['commanded'] = math.degrees(angle_raw * 0.0001)
    if pos_raw != 0x7FFF:
        result['position'] = math.degrees(pos_raw * 0.0001)
    return result if result else None


def parse_pilot_heading(data):
    """Parse PGN 65359 (Seatalk Pilot Heading): MFR(2) + SID(1) + HeadingTrue(U16) + HeadingMag(U16) + reserved"""
    if len(data) < 7:
        return None
    if data[0] != 0x3B or data[1] != 0x9F:
        return None
    # Heading magnetic at bytes 5-6 (radians * 10000)
    raw = struct.unpack_from('<H', data, 5)[0]
    if raw == 0xFFFF:
        return None
    return math.degrees(raw / 10000.0)


def parse_wind(data):
    """Parse PGN 130306: SID(1) + WindSpeed(U16*0.01 m/s) + WindAngle(U16*0.0001 rad) + Ref(1)"""
    if len(data) < 6:
        return None
    speed_raw = struct.unpack_from('<H', data, 1)[0]
    angle_raw = struct.unpack_from('<H', data, 3)[0]
    ref = data[5] & 0x07
    if speed_raw == 0xFFFF or angle_raw == 0xFFFF:
        return None
    speed_kts = (speed_raw * 0.01) / 0.514444
    angle_deg = math.degrees(angle_raw * 0.0001)
    if angle_deg > 180:
        angle_deg -= 360
    return {'speed_kts': speed_kts, 'angle_deg': angle_deg, 'ref': ref}


def parse_stw(data):
    """Parse PGN 128259: SID(1) + STW(U16*0.01 m/s)"""
    if len(data) < 3:
        return None
    raw = struct.unpack_from('<H', data, 1)[0]
    if raw == 0xFFFF:
        return None
    return (raw * 0.01) / 0.514444  # kts


def main():
    log_dir = Path("n2klogs/logged")
    print("Reading CAN log files...")
    frames = read_all_frames(log_dir)
    print(f"Total frames: {len(frames):,}")

    fp = FastPacketReassembler()

    # Build time-series of known values from EV-1 (source 205) and other sources
    # Use closest-in-time for cross-reference
    ev1_heading = []   # (ts, deg) from PGN 127250 source 205
    ev1_rot = []       # (ts, deg/s) from PGN 127251 source 205
    ev1_attitude = []  # (ts, {yaw, pitch, roll}) from PGN 127257 source 205
    ev1_rudder = []    # (ts, {commanded, position}) from PGN 127245 source 205
    pilot_heading = [] # (ts, deg) from PGN 65359 source 205
    wind_data = []     # (ts, {speed_kts, angle_deg, ref}) from PGN 130306 any source
    stw_data = []      # (ts, kts) from PGN 128259 any source

    # f088 messages from EV-1 (27-byte only)
    f088_msgs = []  # (ts, channel, [10 int16 fields])

    for frame in frames:
        ts = frame['ts']
        src = frame['source']
        pgn = frame['pgn']
        data = frame['data']

        # Standard PGNs (single-frame, no fast-packet needed)
        if src == 205:
            if pgn == 127250:
                h = parse_n2k_heading(data)
                if h is not None:
                    ev1_heading.append((ts, h))
            elif pgn == 127251:
                r = parse_n2k_rot(data)
                if r is not None:
                    ev1_rot.append((ts, r))
            elif pgn == 127257:
                a = parse_n2k_attitude(data)
                if a is not None:
                    ev1_attitude.append((ts, a))
            elif pgn == 127245:
                r = parse_n2k_rudder(data)
                if r is not None:
                    ev1_rudder.append((ts, r))

        # Fast-packet PGNs
        if pgn in (126720, 65359, 130306):
            assembled = fp.process_frame(src, pgn, data)
            if assembled is not None:
                if pgn == 126720 and src == 205:
                    # Check for f088 27-byte messages
                    if (len(assembled) == 27 and len(assembled) >= 6
                            and assembled[0] == 0x3B and assembled[1] == 0x9F
                            and assembled[2] == 0xF0 and assembled[3] == 0x88
                            and assembled[5] == 0x00):
                        channel = assembled[4]
                        # Decode 10 int16 LE fields from bytes 7-26
                        fields = list(struct.unpack_from('<10h', assembled, 7))
                        f088_msgs.append((ts, channel, fields))
                elif pgn == 65359 and src == 205:
                    h = parse_pilot_heading(assembled)
                    if h is not None:
                        pilot_heading.append((ts, h))
                elif pgn == 130306:
                    w = parse_wind(assembled)
                    if w is not None:
                        wind_data.append((ts, w))

        # STW (single frame)
        if pgn == 128259:
            s = parse_stw(data)
            if s is not None:
                stw_data.append((ts, s))

    print(f"\nData series collected:")
    print(f"  EV-1 heading (127250):   {len(ev1_heading):,}")
    print(f"  EV-1 rate of turn (127251): {len(ev1_rot):,}")
    print(f"  EV-1 attitude (127257):  {len(ev1_attitude):,}")
    print(f"  EV-1 rudder (127245):    {len(ev1_rudder):,}")
    print(f"  Pilot heading (65359):   {len(pilot_heading):,}")
    print(f"  Wind (130306):           {len(wind_data):,}")
    print(f"  STW (128259):            {len(stw_data):,}")
    print(f"  f088 27-byte messages:   {len(f088_msgs):,}")

    if not f088_msgs:
        print("No f088 messages found!")
        return

    # Helper: find closest value in sorted time series
    def find_closest(series, target_ts):
        if not series:
            return None
        # Binary search
        lo, hi = 0, len(series) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if series[mid][0] < target_ts:
                lo = mid + 1
            else:
                hi = mid
        # Check lo and lo-1
        best = lo
        if best > 0 and abs(series[best-1][0] - target_ts) < abs(series[best][0] - target_ts):
            best = best - 1
        # Only use if within 2 seconds
        if abs(series[best][0] - target_ts) > 2.0:
            return None
        return series[best][1]

    # =========================================================================
    # Cross-reference: for each f088 message, find concurrent known values
    # =========================================================================
    print("\n" + "=" * 100)
    print("CROSS-REFERENCE: f088 int16 fields vs known sensor data")
    print("=" * 100)

    # Use channel 0x00 as representative (all channels have same field structure)
    # Collect matched pairs for correlation analysis
    matched = []
    for ts, channel, fields in f088_msgs:
        heading = find_closest(ev1_heading, ts)
        rot = find_closest(ev1_rot, ts)
        att = find_closest(ev1_attitude, ts)
        rudder = find_closest(ev1_rudder, ts)
        p_hdg = find_closest(pilot_heading, ts)
        wind = find_closest(wind_data, ts)
        stw = find_closest(stw_data, ts)

        if heading is not None:
            matched.append({
                'ts': ts,
                'channel': channel,
                'fields': fields,
                'heading': heading,
                'rot': rot,
                'attitude': att,
                'rudder': rudder,
                'pilot_heading': p_hdg,
                'wind': wind,
                'stw': stw,
            })

    print(f"\n  Matched f088 messages with concurrent heading: {len(matched):,}")

    # Print first 30 samples showing all fields and known values
    print(f"\n  First 30 matched samples (all channels):")
    print(f"  {'Time':>12s}  ch  {'F0':>6s} {'F1':>6s} {'F2':>6s} {'F3':>6s} {'F4':>6s} {'F5':>6s} {'F6':>6s} {'F7':>6s} {'F8':>6s} {'F9':>6s}  |  {'Hdg°':>6s} {'ROT°/s':>7s} {'Roll°':>6s} {'Pitch°':>6s} {'Yaw°':>6s} {'RudPos':>6s} {'AWA°':>6s} {'AWS':>5s} {'STW':>5s}")
    for m in matched[:30]:
        f = m['fields']
        hdg = m['heading']
        rot = m['rot']
        att = m['attitude']
        rud = m['rudder']
        wind = m['wind']
        stw = m['stw']

        rot_s = f"{rot:+7.2f}" if rot is not None else "   N/A"
        roll_s = f"{att['roll']:+6.1f}" if att and 'roll' in att else "  N/A"
        pitch_s = f"{att['pitch']:+6.1f}" if att and 'pitch' in att else "  N/A"
        yaw_s = f"{att['yaw']:+6.1f}" if att and 'yaw' in att else "  N/A"
        rud_s = f"{rud['position']:+6.1f}" if rud and 'position' in rud else "  N/A"
        awa_s = f"{wind['angle_deg']:+6.1f}" if wind else "  N/A"
        aws_s = f"{wind['speed_kts']:5.1f}" if wind else " N/A"
        stw_s = f"{stw:5.1f}" if stw is not None else " N/A"

        print(f"  {ts_str(m['ts']):>12s}  {m['channel']:02x}  {f[0]:+6d} {f[1]:+6d} {f[2]:+6d} {f[3]:+6d} {f[4]:+6d} {f[5]:+6d} {f[6]:+6d} {f[7]:+6d} {f[8]:+6d} {f[9]:+6d}  |  {hdg:6.1f} {rot_s} {roll_s} {pitch_s} {yaw_s} {rud_s} {awa_s} {aws_s} {stw_s}")

    # =========================================================================
    # Compute correlations between each f088 field and known values
    # =========================================================================
    print("\n" + "=" * 100)
    print("CORRELATION ANALYSIS")
    print("=" * 100)

    import statistics

    def pearson_r(xs, ys):
        n = len(xs)
        if n < 5:
            return None
        mx = statistics.mean(xs)
        my = statistics.mean(ys)
        sx = statistics.stdev(xs)
        sy = statistics.stdev(ys)
        if sx < 1e-10 or sy < 1e-10:
            return None
        cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (n - 1)
        return cov / (sx * sy)

    # For each field, compute correlation with each known sensor value
    known_extractors = {
        'heading': lambda m: m['heading'],
        'ROT': lambda m: m['rot'] if m['rot'] is not None else None,
        'roll': lambda m: m['attitude']['roll'] if m['attitude'] and 'roll' in m['attitude'] else None,
        'pitch': lambda m: m['attitude']['pitch'] if m['attitude'] and 'pitch' in m['attitude'] else None,
        'yaw': lambda m: m['attitude']['yaw'] if m['attitude'] and 'yaw' in m['attitude'] else None,
        'rudder_pos': lambda m: m['rudder']['position'] if m['rudder'] and 'position' in m['rudder'] else None,
        'AWA': lambda m: m['wind']['angle_deg'] if m['wind'] else None,
        'AWS': lambda m: m['wind']['speed_kts'] if m['wind'] else None,
        'STW': lambda m: m['stw'],
    }

    print(f"\n  Pearson correlation of each f088 field with known sensor values:")
    print(f"  (Using all channels combined, {len(matched)} samples)")
    print()

    header = f"  {'Field':>5s}"
    for name in known_extractors:
        header += f"  {name:>10s}"
    header += f"   {'Mean':>8s}  {'StdDev':>8s}  {'Min':>8s}  {'Max':>8s}"
    print(header)
    print("  " + "-" * (len(header) - 2))

    for fi in range(10):
        row = f"  F{fi:>3d} "
        field_vals = [m['fields'][fi] for m in matched]

        for name, extractor in known_extractors.items():
            pairs = [(m['fields'][fi], extractor(m)) for m in matched if extractor(m) is not None]
            if len(pairs) >= 5:
                xs = [p[0] for p in pairs]
                ys = [p[1] for p in pairs]
                r = pearson_r(xs, ys)
                if r is not None:
                    row += f"  {r:+10.4f}"
                else:
                    row += f"  {'const':>10s}"
            else:
                row += f"  {'N/A':>10s}"

        mean_v = statistics.mean(field_vals)
        std_v = statistics.stdev(field_vals) if len(field_vals) > 1 else 0
        min_v = min(field_vals)
        max_v = max(field_vals)
        row += f"   {mean_v:+8.1f}  {std_v:8.1f}  {min_v:+8d}  {max_v:+8d}"
        print(row)

    # =========================================================================
    # Try scaling hypotheses
    # =========================================================================
    print("\n" + "=" * 100)
    print("SCALING HYPOTHESES")
    print("=" * 100)

    # For fields with high correlation, try to find the scaling factor
    # heading * scale = field_value → scale = field_value / heading
    print("\n  For each field, test common N2K/BNO055 scaling factors:")
    print()

    # Hypothesis: field might be heading in some unit
    # BNO055 uses 1/16 deg = 16 LSB/deg
    # N2K uses radians * 10000
    # Or raw ADC counts

    for fi in range(10):
        field_vals = [m['fields'][fi] for m in matched]
        headings = [m['heading'] for m in matched]
        rots = [m['rot'] for m in matched if m['rot'] is not None]
        rot_fields = [m['fields'][fi] for m in matched if m['rot'] is not None]

        std_f = statistics.stdev(field_vals) if len(field_vals) > 1 else 0
        if std_f < 1:
            continue  # Skip near-constant fields

        print(f"  F{fi}: mean={statistics.mean(field_vals):+.1f}  std={std_f:.1f}  range=[{min(field_vals)}, {max(field_vals)}]")

        # Test heading correlation
        r_hdg = pearson_r(field_vals, headings)
        if r_hdg is not None and abs(r_hdg) > 0.3:
            # Compute scaling: linear regression field = a * heading + b
            n = len(field_vals)
            mx = statistics.mean(headings)
            my = statistics.mean(field_vals)
            num = sum((h - mx) * (f - my) for h, f in zip(headings, field_vals))
            den = sum((h - mx) ** 2 for h in headings)
            if den > 0:
                a = num / den
                b = my - a * mx
                residuals = [f - (a * h + b) for h, f in zip(headings, field_vals)]
                rmse = (sum(r**2 for r in residuals) / len(residuals)) ** 0.5
                print(f"       Heading corr: r={r_hdg:+.4f}  scale={a:.4f}  offset={b:.1f}  RMSE={rmse:.1f}")
                # Check known scalings
                if abs(a - 16.0) < 2:
                    print(f"       → LIKELY BNO055 heading (1 LSB = 1/16 deg)")
                elif abs(a - 174.5) < 20:
                    print(f"       → LIKELY N2K radians*10000 (1 rad = 10000)")

        # Test ROT correlation
        if rots and rot_fields:
            r_rot = pearson_r(rot_fields, rots)
            if r_rot is not None and abs(r_rot) > 0.3:
                n = len(rots)
                mx = statistics.mean(rots)
                my = statistics.mean(rot_fields)
                num = sum((r - mx) * (f - my) for r, f in zip(rots, rot_fields))
                den = sum((r - mx) ** 2 for r in rots)
                if den > 0:
                    a = num / den
                    b = my - a * mx
                    residuals = [f - (a * r + b) for r, f in zip(rots, rot_fields)]
                    rmse = (sum(r**2 for r in residuals) / len(residuals)) ** 0.5
                    print(f"       ROT corr: r={r_rot:+.4f}  scale={a:.4f}  offset={b:.1f}  RMSE={rmse:.1f}")

        # Test roll correlation
        rolls = [m['attitude']['roll'] for m in matched if m['attitude'] and 'roll' in m['attitude']]
        roll_fields = [m['fields'][fi] for m in matched if m['attitude'] and 'roll' in m['attitude']]
        if rolls and roll_fields:
            r_roll = pearson_r(roll_fields, rolls)
            if r_roll is not None and abs(r_roll) > 0.3:
                n = len(rolls)
                mx = statistics.mean(rolls)
                my = statistics.mean(roll_fields)
                num = sum((r - mx) * (f - my) for r, f in zip(rolls, roll_fields))
                den = sum((r - mx) ** 2 for r in rolls)
                if den > 0:
                    a = num / den
                    b = my - a * mx
                    residuals = [f - (a * r + b) for r, f in zip(rolls, roll_fields)]
                    rmse = (sum(r**2 for r in residuals) / len(residuals)) ** 0.5
                    print(f"       Roll corr: r={r_roll:+.4f}  scale={a:.4f}  offset={b:.1f}  RMSE={rmse:.1f}")

        # Test pitch correlation
        pitches = [m['attitude']['pitch'] for m in matched if m['attitude'] and 'pitch' in m['attitude']]
        pitch_fields = [m['fields'][fi] for m in matched if m['attitude'] and 'pitch' in m['attitude']]
        if pitches and pitch_fields:
            r_pitch = pearson_r(pitch_fields, pitches)
            if r_pitch is not None and abs(r_pitch) > 0.3:
                n = len(pitches)
                mx = statistics.mean(pitches)
                my = statistics.mean(pitch_fields)
                num = sum((p - mx) * (f - my) for p, f in zip(pitches, pitch_fields))
                den = sum((p - mx) ** 2 for p in pitches)
                if den > 0:
                    a = num / den
                    b = my - a * mx
                    residuals = [f - (a * p + b) for p, f in zip(pitches, pitch_fields)]
                    rmse = (sum(r**2 for r in residuals) / len(residuals)) ** 0.5
                    print(f"       Pitch corr: r={r_pitch:+.4f}  scale={a:.4f}  offset={b:.1f}  RMSE={rmse:.1f}")

        # Test AWA correlation
        awas = [m['wind']['angle_deg'] for m in matched if m['wind']]
        awa_fields = [m['fields'][fi] for m in matched if m['wind']]
        if awas and awa_fields:
            r_awa = pearson_r(awa_fields, awas)
            if r_awa is not None and abs(r_awa) > 0.3:
                n = len(awas)
                mx = statistics.mean(awas)
                my = statistics.mean(awa_fields)
                num = sum((a - mx) * (f - my) for a, f in zip(awas, awa_fields))
                den = sum((a - mx) ** 2 for a in awas)
                if den > 0:
                    a_coef = num / den
                    b = my - a_coef * mx
                    print(f"       AWA corr: r={r_awa:+.4f}  scale={a_coef:.4f}  offset={b:.1f}")

        print()

    # =========================================================================
    # Check if F0 is a timestamp / counter
    # =========================================================================
    print("=" * 100)
    print("F0 COUNTER/TIMESTAMP ANALYSIS")
    print("=" * 100)

    # Group by channel and check if F0 increments
    by_channel = defaultdict(list)
    for ts, ch, fields in f088_msgs:
        by_channel[ch].append((ts, fields[0]))

    print(f"\n  Checking F0 for channel 0x00:")
    ch0 = by_channel[0]
    if ch0:
        print(f"  First 20 values: {[v for _, v in ch0[:20]]}")
        # Check deltas
        deltas_t = [ch0[i+1][0] - ch0[i][0] for i in range(min(20, len(ch0)-1))]
        deltas_v = [ch0[i+1][1] - ch0[i][1] for i in range(min(20, len(ch0)-1))]
        print(f"  Time deltas (s): {[f'{d:.3f}' for d in deltas_t[:15]]}")
        print(f"  Value deltas:    {deltas_v[:15]}")

    # =========================================================================
    # Check per-channel differences (is F0 channel-dependent?)
    # =========================================================================
    print(f"\n  F0 values at same timestamp across different channels:")
    # Group by approximate timestamp (within 0.1s)
    ts_groups = defaultdict(list)
    for ts, ch, fields in f088_msgs:
        ts_key = round(ts * 10) / 10  # round to 0.1s
        ts_groups[ts_key].append((ch, fields))

    count = 0
    for ts_key in sorted(ts_groups.keys()):
        if len(ts_groups[ts_key]) >= 3 and count < 10:
            items = sorted(ts_groups[ts_key], key=lambda x: x[0])
            f0_vals = [(ch, f[0]) for ch, f in items]
            print(f"  {datetime.datetime.fromtimestamp(ts_key).strftime('%H:%M:%S.%f')[:-4]}: " +
                  "  ".join(f"ch{ch:02x}={v:+d}" for ch, v in f0_vals[:8]))
            count += 1

    # =========================================================================
    # Deeper: try treating field pairs as different encodings
    # =========================================================================
    print("\n" + "=" * 100)
    print("ALTERNATIVE ENCODINGS")
    print("=" * 100)

    # Try bytes 7-10 as float32
    print("\n  Bytes 7-10 as float32 LE:")
    for ts, ch, fields in f088_msgs[:10]:
        # Reconstruct raw bytes from f088 message
        pass

    # We need raw bytes. Let's re-extract from matched data
    f088_raw = []
    fp2 = FastPacketReassembler()
    for frame in frames:
        if frame['source'] == 205 and frame['pgn'] == 126720:
            assembled = fp2.process_frame(frame['source'], frame['pgn'], frame['data'])
            if assembled and len(assembled) == 27:
                if (assembled[0] == 0x3B and assembled[1] == 0x9F
                        and assembled[2] == 0xF0 and assembled[3] == 0x88
                        and assembled[5] == 0x00):
                    f088_raw.append((frame['ts'], assembled[4], assembled))

    print(f"  Raw 27-byte messages: {len(f088_raw):,}")

    # Try bytes 7-10 as uint32
    print("\n  Byte 6 (length/format): ", set(msg[6] for _, _, msg in f088_raw[:100]))

    # Bytes 7-8 as uint16 (unsigned) — could be a heading in some unit
    print("\n  Bytes 7-8 as uint16 LE vs heading (first 20 of channel 0x00):")
    ch0_raw = [(ts, msg) for ts, ch, msg in f088_raw if ch == 0]
    for ts, msg in ch0_raw[:20]:
        u16_val = struct.unpack_from('<H', msg, 7)[0]
        h = find_closest(ev1_heading, ts)
        h_str = f"{h:.1f}" if h is not None else "N/A"
        # Try as radians * 10000 (like N2K heading)
        deg_if_rad10k = math.degrees(u16_val / 10000.0) if u16_val < 65000 else None
        deg_str = f"{deg_if_rad10k:.1f}" if deg_if_rad10k else "N/A"
        # Try as 1/16 degree (BNO055)
        deg_16 = u16_val / 16.0
        print(f"  {ts_str(ts)} u16={u16_val:5d}  if_rad10k={deg_str:>7s}  if_1/16deg={deg_16:7.1f}  heading={h_str}")

    # Try fields as various rad/deg scalings
    print("\n" + "=" * 100)
    print("FIELD-BY-FIELD PHYSICAL UNITS HYPOTHESIS TEST")
    print("=" * 100)

    # For each near-constant field, identify the constant value
    # and test if it maps to a known physical constant
    print("\n  Near-constant fields and their hypothetical physical meanings:")
    for fi in range(10):
        vals = [m['fields'][fi] for m in matched]
        mean_v = statistics.mean(vals)
        std_v = statistics.stdev(vals) if len(vals) > 1 else 0

        if std_v < 5:  # Near-constant
            # Test hypotheses
            hypotheses = []
            # As radians * 10000 → degrees
            deg = math.degrees(mean_v / 10000.0)
            hypotheses.append(f"rad*10000→{deg:.2f}°")
            # As 1/16 degree
            deg16 = mean_v / 16.0
            hypotheses.append(f"1/16°→{deg16:.2f}°")
            # As raw degrees
            hypotheses.append(f"raw°→{mean_v:.1f}°")
            # As m/s * 100
            ms = mean_v / 100.0
            kts = ms / 0.514444
            hypotheses.append(f"m/s*100→{kts:.2f}kts")
            # As gravity * 100 (acceleration)
            g = mean_v / 100.0
            hypotheses.append(f"*0.01→{g:.2f}")

            print(f"  F{fi}: mean={mean_v:+.1f} std={std_v:.2f}  |  " + "  |  ".join(hypotheses))

    # =========================================================================
    # Check 28-byte messages (the float32 variant)
    # =========================================================================
    print("\n" + "=" * 100)
    print("28-BYTE f088 MESSAGES (float32 variant)")
    print("=" * 100)

    f088_28 = []
    fp3 = FastPacketReassembler()
    for frame in frames:
        if frame['source'] == 205 and frame['pgn'] == 126720:
            assembled = fp3.process_frame(frame['source'], frame['pgn'], frame['data'])
            if assembled and len(assembled) == 28:
                if (assembled[0] == 0x3B and assembled[1] == 0x9F
                        and assembled[2] == 0xF0 and assembled[3] == 0x88
                        and assembled[5] == 0x00):
                    f088_28.append((frame['ts'], assembled[4], assembled))

    print(f"  28-byte messages: {len(f088_28):,}")

    if f088_28:
        print("\n  Decode bytes 7-27 as float32 LE pairs:")
        print(f"  {'Time':>12s}  ch  {'float[7:11]':>12s} {'float[11:15]':>12s} {'float[15:19]':>12s} {'bytes[19:22]':>12s} {'bytes[23:27]':>14s}  |  heading")

        for ts, ch, msg in f088_28[:20]:
            f1 = struct.unpack_from('<f', msg, 7)[0]
            f2 = struct.unpack_from('<f', msg, 11)[0]
            f3 = struct.unpack_from('<f', msg, 15)[0]
            rest1 = msg[19:23].hex()
            rest2 = msg[23:28].hex()
            h = find_closest(ev1_heading, ts)
            h_str = f"{h:.1f}" if h is not None else "N/A"

            # Check if float values could be heading in degrees or radians
            f1_deg = f"({math.degrees(f1):.1f}° if rad)" if abs(f1) < 10 else ""
            f2_deg = f"({math.degrees(f2):.1f}° if rad)" if abs(f2) < 10 else ""

            print(f"  {ts_str(ts):>12s}  {ch:02x}  {f1:12.4f} {f2:12.4f} {f3:12.4f} {rest1:>12s} {rest2:>14s}  |  {h_str}")

    # =========================================================================
    # Check 36-byte messages (another variant with float32 + constants)
    # =========================================================================
    print("\n" + "=" * 100)
    print("36-BYTE f088 MESSAGES")
    print("=" * 100)

    f088_36 = []
    fp4 = FastPacketReassembler()
    for frame in frames:
        if frame['source'] == 205 and frame['pgn'] == 126720:
            assembled = fp4.process_frame(frame['source'], frame['pgn'], frame['data'])
            if assembled and len(assembled) == 36:
                if (assembled[0] == 0x3B and assembled[1] == 0x9F
                        and assembled[2] == 0xF0 and assembled[3] == 0x88
                        and assembled[5] == 0x00):
                    f088_36.append((frame['ts'], assembled[4], assembled))

    print(f"  36-byte messages: {len(f088_36):,}")

    if f088_36:
        print("\n  Decode as float32 fields:")
        # Bytes 0-5: header, byte 6: format
        # Try bytes 7-10, 11-14, 15-18 as float32
        print(f"  {'Time':>12s}  ch  fmt  {'float[7:11]':>12s} {'float[11:15]':>12s} {'float[15:19]':>12s}  |  heading  ROT°/s")

        for ts, ch, msg in f088_36[:20]:
            fmt = msg[6]
            f1 = struct.unpack_from('<f', msg, 7)[0]
            f2 = struct.unpack_from('<f', msg, 11)[0]
            f3 = struct.unpack_from('<f', msg, 15)[0]
            h = find_closest(ev1_heading, ts)
            rot = find_closest(ev1_rot, ts)
            h_str = f"{h:.1f}" if h is not None else "N/A"
            rot_str = f"{rot:+.3f}" if rot is not None else "N/A"

            # Check if f1 could be heading in degrees
            print(f"  {ts_str(ts):>12s}  {ch:02x}  {fmt:02x}  {f1:12.4f} {f2:12.4f} {f3:12.4f}  |  {h_str:>7s}  {rot_str}")

    # =========================================================================
    # Check 35-byte messages (the calibration-looking variant)
    # =========================================================================
    print("\n" + "=" * 100)
    print("35-BYTE f088 MESSAGES")
    print("=" * 100)

    f088_35 = []
    fp5 = FastPacketReassembler()
    for frame in frames:
        if frame['source'] == 205 and frame['pgn'] == 126720:
            assembled = fp5.process_frame(frame['source'], frame['pgn'], frame['data'])
            if assembled and len(assembled) == 35:
                if (assembled[0] == 0x3B and assembled[1] == 0x9F
                        and assembled[2] == 0xF0 and assembled[3] == 0x88
                        and assembled[5] == 0x00):
                    f088_35.append((frame['ts'], assembled[4], assembled))

    print(f"  35-byte messages: {len(f088_35):,}")

    if f088_35:
        # From earlier analysis, bytes 19-22 = 9a999140 = float 4.55
        # bytes 23-26 = 2fdd243e = float 0.161
        # bytes 27-30 = 43c65141 = float 13.11
        print("\n  Decode as float32 fields (bytes 7 onwards):")
        print(f"  {'Time':>12s}  ch  fmt  {'f[7:11]':>10s} {'f[11:15]':>10s} {'f[15:19]':>10s} {'f[19:23]':>10s} {'f[23:27]':>10s} {'f[27:31]':>10s}  |  heading")
        for ts, ch, msg in f088_35[:20]:
            fmt = msg[6]
            floats = []
            for off in range(7, 31, 4):
                if off + 4 <= len(msg):
                    floats.append(struct.unpack_from('<f', msg, off)[0])
                else:
                    floats.append(float('nan'))
            h = find_closest(ev1_heading, ts)
            h_str = f"{h:.1f}" if h is not None else "N/A"
            fstr = " ".join(f"{v:10.4f}" for v in floats)
            print(f"  {ts_str(ts):>12s}  {ch:02x}  {fmt:02x}  {fstr}  |  {h_str}")


if __name__ == '__main__':
    main()
