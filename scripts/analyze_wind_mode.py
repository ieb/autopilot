#!/usr/bin/env python3
"""
Analyze how the p70 tells the EV-1 to steer in wind mode (AWA vs TWA).

Builds a correlated timeline of:
  - p70 PGN 126208 commands (mode, wind datum, heading)
  - EV-1 heartbeat mode changes (f081ae02)
  - Wind data (PGN 130306) and heading (PGN 127250)
  - EV-1 126208 ACKs

Focuses on identifying AWA vs TWA from 65379 submode bytes.
"""

import struct
import math
import datetime
from collections import defaultdict, Counter
from pathlib import Path

HEADER_STRUCT = struct.Struct('<4sIII d 8x')
FRAME_STRUCT = struct.Struct('<II BB BB 8s I')

FAST_PACKET_PGNS = {
    126208, 126464, 126720, 126996, 128275,
    129029, 129038, 129039, 129040, 129041, 129044,
    129283, 129540, 129793, 129794, 129797, 129809, 129810,
    130822, 130846, 130916,
}

PGN_NAMES = {
    65345: "Wind Datum",
    65359: "Pilot Heading",
    65360: "Locked Heading",
    65362: "Pilot Speed",
    65379: "Pilot Mode",
    65384: "Keypad Heartbeat",
    127237: "Heading/Track Control",
    127245: "Rudder",
    127250: "Vessel Heading",
    126720: "Raymarine Proprietary",
}


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
            payload = data[1:8]
            session['data'].extend(payload)
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
                    'priority': priority,
                    'data': data[:dlc],
                    'pgn': pgn,
                })
    return frames


def ts_str(ts):
    return datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3]


def decode_mode_bytes(raw):
    """Decode 65379 mode command payload.
    Format after PGN 126208 header:
      raw[0]=FC(0x01), raw[1:4]=PGN, raw[4]=priority, raw[5]=num_params,
      then param pairs: raw[6]=param_idx, raw[7]=param_size?,
      followed by Raymarine header bytes (3b 07 03 04 04) then mode data.

    Known mode byte patterns from the p70:
      byte[12] byte[13] = mode, submode_lo
      byte[14] = num_params_2?
      byte[15:17] = submode_hi bytes (ff ff = unspecified)
    """
    if len(raw) < 14:
        return "TOO SHORT"

    mode_byte = raw[12]
    submode_lo = raw[13]

    # Decode the primary mode
    mode_map = {
        (0x00, 0x00): "STANDBY",
        (0x40, 0x00): "AUTO (Compass)",
        (0x00, 0x01): "WIND",
        (0x80, 0x01): "TRACK",
        (0x81, 0x00): "NO DRIFT",
    }
    mode_name = mode_map.get((mode_byte, submode_lo),
                              f"UNKNOWN mode=0x{mode_byte:02x} sub=0x{submode_lo:02x}")

    # For WIND mode, decode the submode_hi bytes
    submode_detail = ""
    if submode_lo == 0x01 and len(raw) >= 17:
        sub_hi = raw[15]
        sub_hi2 = raw[16]
        if sub_hi == 0xFF and sub_hi2 == 0xFF:
            submode_detail = " [submode=keep/default]"
        else:
            submode_detail = f" [submode_hi=0x{sub_hi:02x} 0x{sub_hi2:02x}]"

    return f"{mode_name}{submode_detail}"


def decode_wind_datum(raw):
    """Decode 65345 wind datum command — extract wind angle."""
    if len(raw) < 14:
        return None, "TOO SHORT"
    wind_raw = raw[12] | (raw[13] << 8)
    wind_deg = math.degrees(wind_raw / 10000.0)
    return wind_deg, f"wind={wind_deg:.1f}°"


def decode_heading(raw):
    """Decode 65360 locked heading command."""
    if len(raw) < 14:
        return None, "TOO SHORT"
    hdg_raw = raw[12] | (raw[13] << 8)
    hdg_deg = math.degrees(hdg_raw / 10000.0)
    return hdg_deg, f"heading={hdg_deg:.1f}°"


def main():
    log_dir = Path("/Users/boston/ieb/autopilot/n2klogs/logged")
    frames = read_all_frames(log_dir)
    print(f"Total frames: {len(frames):,}")

    reassembler_126208 = FastPacketReassembler()
    reassembler_126720 = FastPacketReassembler()

    # Collect events into a unified timeline
    events = []

    # Track latest sensor values for context
    latest_heading = None
    latest_awa = None
    latest_aws = None
    latest_rudder = None

    for f in frames:
        ts = f['ts']
        src = f['source']
        pgn = f['pgn']
        data = f['data']

        # --- Single-frame PGNs (sensor data for context) ---

        # Vessel Heading (127250) — 8 bytes, single frame
        if pgn == 127250 and f['dlc'] >= 8:
            # bytes 1-2: heading in radians * 10000 (uint16 LE)
            hdg_raw = data[1] | (data[2] << 8)
            if hdg_raw != 0xFFFF:
                latest_heading = math.degrees(hdg_raw / 10000.0)

        # Wind Data (130306) — 8 bytes, single frame
        elif pgn == 130306 and f['dlc'] >= 6:
            # byte 0: SID, bytes 1-2: wind speed (0.01 m/s), bytes 3-4: wind angle (rad*10000)
            ws_raw = data[1] | (data[2] << 8)
            wa_raw = data[3] | (data[4] << 8)
            wind_ref = data[5] & 0x07
            if wa_raw != 0xFFFF:
                awa_deg = math.degrees(wa_raw / 10000.0)
                if wind_ref in (0, 2):  # Apparent (0) or True ground (2) or True boat (3)
                    latest_awa = awa_deg
            if ws_raw != 0xFFFF:
                latest_aws = ws_raw * 0.01 * 1.94384  # m/s to knots

        # Rudder (127245) — 8 bytes, single frame
        elif pgn == 127245 and f['dlc'] >= 6:
            rud_raw = struct.unpack_from('<h', data, 4)[0]
            if rud_raw != 0x7FFF:
                latest_rudder = math.degrees(rud_raw / 10000.0)

        # --- Fast-packet PGNs ---

        # PGN 126208 (Command/Request/Ack)
        elif pgn == 126208:
            result = reassembler_126208.process_frame(src, pgn, data)
            if result:
                fc = result[0]
                if fc <= 0x02 and len(result) >= 4:
                    target_pgn = result[1] | (result[2] << 8) | (result[3] << 16)

                    if src == 0 and fc == 0x01:  # p70 COMMAND
                        detail = ""
                        if target_pgn == 65379:
                            detail = decode_mode_bytes(result)
                        elif target_pgn == 65345:
                            _, detail = decode_wind_datum(result)
                        elif target_pgn == 65360:
                            _, detail = decode_heading(result)
                        else:
                            detail = result.hex(' ')
                        events.append({
                            'ts': ts, 'type': 'p70_CMD',
                            'pgn': target_pgn, 'detail': detail,
                            'raw': result,
                            'heading': latest_heading, 'awa': latest_awa,
                            'aws': latest_aws, 'rudder': latest_rudder,
                        })

                    elif src == 205 and fc == 0x02:  # EV-1 ACK
                        pgn_err = result[4] if len(result) > 4 else 0xFF
                        err_name = {0: "OK", 1: "Not supported", 2: "Unavail"}.get(pgn_err, f"err={pgn_err:#x}")
                        events.append({
                            'ts': ts, 'type': 'ev1_ACK',
                            'pgn': target_pgn, 'detail': err_name,
                            'raw': result,
                        })

        # PGN 126720 — EV-1 heartbeat (f081ae02)
        elif pgn == 126720 and src == 205:
            result = reassembler_126720.process_frame(src, pgn, data)
            if result and len(result) >= 7:
                if result[:4] == b'\x3b\x9f\xf0\x81' and len(result) >= 7:
                    sig = result[4]
                    if sig == 0xae and result[5] == 0x02:
                        mode_byte = result[6]
                        hb_modes = {0x00: "STANDBY", 0x01: "AUTO", 0x03: "WIND/TRACK"}
                        mode_name = hb_modes.get(mode_byte, f"0x{mode_byte:02x}")
                        events.append({
                            'ts': ts, 'type': 'ev1_HB',
                            'pgn': 126720, 'detail': f"heartbeat mode={mode_name}",
                            'raw': result,
                            'heading': latest_heading, 'awa': latest_awa,
                        })

    events.sort(key=lambda e: e['ts'])
    print(f"Total timeline events: {len(events)}")

    # =========================================================================
    # Section 1: Full decoded list of all 65379 mode commands
    # =========================================================================
    print("\n" + "=" * 80)
    print("ALL PGN 65379 MODE COMMANDS FROM P70")
    print("=" * 80)

    mode_cmds = [e for e in events if e['type'] == 'p70_CMD' and e['pgn'] == 65379]
    print(f"\nTotal mode commands: {len(mode_cmds)}")

    for e in mode_cmds:
        raw = e['raw']
        hdg_ctx = f"  hdg={e.get('heading', 0):.1f}°" if e.get('heading') is not None else ""
        awa_ctx = f"  awa={e.get('awa', 0):.1f}°" if e.get('awa') is not None else ""
        print(f"  {ts_str(e['ts'])}  {e['detail']}{hdg_ctx}{awa_ctx}")
        # Show raw bytes for detailed analysis
        print(f"    raw: {raw.hex(' ')}")
        if len(raw) >= 17:
            print(f"    bytes[12:17]: mode=0x{raw[12]:02x} sub_lo=0x{raw[13]:02x} "
                  f"nparam2=0x{raw[14]:02x} sub_hi=[0x{raw[15]:02x} 0x{raw[16]:02x}]")

    # =========================================================================
    # Section 2: Submode byte analysis
    # =========================================================================
    print("\n" + "=" * 80)
    print("65379 SUBMODE BYTE ANALYSIS")
    print("=" * 80)

    wind_cmds = [e for e in mode_cmds if b'\x00\x01' == e['raw'][12:14]]
    print(f"\nWIND mode commands: {len(wind_cmds)}")

    submode_groups = defaultdict(list)
    for e in wind_cmds:
        raw = e['raw']
        if len(raw) >= 17:
            key = (raw[14], raw[15], raw[16])
            submode_groups[key].append(e)
        else:
            submode_groups[('short',)].append(e)

    for key, group in sorted(submode_groups.items()):
        if isinstance(key[0], int):
            print(f"\n  Submode bytes: nparam2=0x{key[0]:02x} hi=[0x{key[1]:02x} 0x{key[2]:02x}]  ({len(group)} occurrences)")
        else:
            print(f"\n  Short payload ({len(group)} occurrences)")
        for e in group:
            hdg_ctx = f"  hdg={e.get('heading', 0):.1f}°" if e.get('heading') is not None else ""
            awa_ctx = f"  awa={e.get('awa', 0):.1f}°" if e.get('awa') is not None else ""
            print(f"    {ts_str(e['ts'])}  {e['detail']}{hdg_ctx}{awa_ctx}")

    # =========================================================================
    # Section 3: Full timeline around wind mode periods
    # =========================================================================
    print("\n" + "=" * 80)
    print("WIND MODE TIMELINE (all events during wind periods)")
    print("=" * 80)

    # Identify wind mode periods from mode commands
    # A wind period starts with a WIND mode command and ends at the next non-WIND command
    in_wind_mode = False
    wind_start = None
    wind_periods = []

    for e in events:
        if e['type'] == 'p70_CMD' and e['pgn'] == 65379:
            raw = e['raw']
            if len(raw) >= 14:
                is_wind = (raw[12] == 0x00 and raw[13] == 0x01)
                if is_wind and not in_wind_mode:
                    wind_start = e['ts']
                    in_wind_mode = True
                elif not is_wind and in_wind_mode:
                    wind_periods.append((wind_start, e['ts']))
                    in_wind_mode = False

    if in_wind_mode:
        wind_periods.append((wind_start, events[-1]['ts']))

    print(f"\nFound {len(wind_periods)} wind mode period(s)")

    for i, (wstart, wend) in enumerate(wind_periods):
        # Show events from 2s before wind start to 2s after wind end
        margin = 2.0
        period_events = [e for e in events
                         if wstart - margin <= e['ts'] <= wend + margin]

        print(f"\n--- Wind Period {i+1}: {ts_str(wstart)} → {ts_str(wend)} "
              f"(duration: {wend - wstart:.1f}s) ---")

        prev_ts = None
        for e in period_events:
            gap = ""
            if prev_ts is not None:
                dt = e['ts'] - prev_ts
                if dt > 1.0:
                    gap = f"  [+{dt:.1f}s]"
            prev_ts = e['ts']

            # Format based on event type
            name = PGN_NAMES.get(e['pgn'], f"PGN {e['pgn']}")
            ctx = ""
            if e.get('heading') is not None and e['type'] in ('p70_CMD', 'ev1_HB'):
                ctx += f"  hdg={e['heading']:.1f}°"
            if e.get('awa') is not None and e['type'] in ('p70_CMD', 'ev1_HB'):
                ctx += f"  awa={e['awa']:.1f}°"
            if e.get('rudder') is not None and e['type'] == 'p70_CMD':
                ctx += f"  rud={e['rudder']:.1f}°"

            marker = {'p70_CMD': '>>>', 'ev1_ACK': '  <<<', 'ev1_HB': '  [HB]'}.get(e['type'], '  ???')
            print(f"  {ts_str(e['ts'])} {marker} {name}: {e['detail']}{ctx}{gap}")

    # =========================================================================
    # Section 4: Wind datum command analysis
    # =========================================================================
    print("\n" + "=" * 80)
    print("WIND DATUM (65345) COMMANDS — ALL DECODED")
    print("=" * 80)

    datum_cmds = [e for e in events if e['type'] == 'p70_CMD' and e['pgn'] == 65345]
    print(f"\nTotal wind datum commands: {len(datum_cmds)}")

    prev_wind = None
    for e in datum_cmds:
        raw = e['raw']
        wind_deg, detail = decode_wind_datum(raw)

        delta = ""
        if prev_wind is not None and wind_deg is not None:
            d = wind_deg - prev_wind
            delta = f"  (delta={d:+.1f}°)"
        prev_wind = wind_deg

        hdg_ctx = f"  hdg={e.get('heading', 0):.1f}°" if e.get('heading') is not None else ""
        awa_ctx = f"  awa={e.get('awa', 0):.1f}°" if e.get('awa') is not None else ""
        print(f"  {ts_str(e['ts'])}  {detail}{delta}{hdg_ctx}{awa_ctx}")
        print(f"    raw bytes[12:14]: 0x{raw[12]:02x} 0x{raw[13]:02x}  (uint16 LE = {raw[12] | (raw[13] << 8)})")

    # =========================================================================
    # Section 5: Heading commands during wind mode
    # =========================================================================
    print("\n" + "=" * 80)
    print("HEADING (65360) COMMANDS — CHECKING IF ANY DURING WIND MODE")
    print("=" * 80)

    hdg_cmds = [e for e in events if e['type'] == 'p70_CMD' and e['pgn'] == 65360]
    print(f"\nTotal heading commands: {len(hdg_cmds)}")

    for e in hdg_cmds:
        in_wind = any(ws <= e['ts'] <= we for ws, we in wind_periods)
        raw = e['raw']
        _, detail = decode_heading(raw)
        wind_marker = " ** DURING WIND MODE **" if in_wind else ""
        print(f"  {ts_str(e['ts'])}  {detail}{wind_marker}")

    # =========================================================================
    # Section 6: Summary — submode correlation
    # =========================================================================
    print("\n" + "=" * 80)
    print("SUMMARY: WIND SUBMODE → BEHAVIOR CORRELATION")
    print("=" * 80)

    for key, group in sorted(submode_groups.items()):
        if not isinstance(key[0], int):
            continue
        print(f"\n  Submode hi=[0x{key[1]:02x} 0x{key[2]:02x}]:")
        for e in group:
            # Find what datum/heading commands follow within 30s
            following = [ev for ev in events
                         if ev['ts'] > e['ts'] and ev['ts'] < e['ts'] + 30
                         and ev['type'] == 'p70_CMD'
                         and ev['pgn'] in (65345, 65360, 65379)]
            print(f"    At {ts_str(e['ts'])}:")
            if following:
                for fev in following[:5]:
                    name = PGN_NAMES.get(fev['pgn'], f"PGN {fev['pgn']}")
                    dt = fev['ts'] - e['ts']
                    print(f"      +{dt:.1f}s  {name}: {fev['detail']}")
            else:
                print(f"      (no follow-up commands within 30s)")


if __name__ == "__main__":
    main()
