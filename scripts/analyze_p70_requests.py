#!/usr/bin/env python3
"""
Analyze p70 REQUEST messages (PGN 126208 FC=0x00) and EV-1 responses.
Decode what the p70 is polling for and what it expects back.
"""

import struct
import math
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


def decode_126208_request(data):
    """Decode PGN 126208 Request (FC=0x00).
    Format: FC(1) + requested_PGN(3 LE) + transmission_interval(4) +
            transmission_interval_offset(2) + num_params(1) +
            [param_index(1) + param_value(variable)]...
    """
    if len(data) < 4 or data[0] != 0x00:
        return None

    req_pgn = data[1] | (data[2] << 8) | (data[3] << 16)

    info = {'requested_pgn': req_pgn, 'raw': data}

    if len(data) >= 11:
        # transmission interval (4 bytes) and offset (2 bytes)
        tx_interval = struct.unpack_from('<I', data, 4)[0]
        tx_offset = struct.unpack_from('<H', data, 8)[0]
        num_params = data[10]
        info['tx_interval'] = tx_interval
        info['tx_offset'] = tx_offset
        info['num_params'] = num_params

        # Parse parameter pairs
        params = []
        pos = 11
        for _ in range(num_params):
            if pos >= len(data):
                break
            param_idx = data[pos]
            pos += 1
            # Parameter value — we need to know the field size from the target PGN
            # For Seatalk proprietary PGNs, parameters are typically 1-2 bytes
            # Read remaining bytes as parameter value
            if pos < len(data):
                # Heuristic: read 1 byte for field index, value depends on PGN
                param_val = data[pos:pos+2] if pos + 2 <= len(data) else data[pos:pos+1]
                params.append((param_idx, param_val))
                pos += len(param_val)
        info['params'] = params

    return info


def decode_126208_command(data):
    """Decode PGN 126208 Command (FC=0x01)."""
    if len(data) < 4 or data[0] != 0x01:
        return None

    target_pgn = data[1] | (data[2] << 8) | (data[3] << 16)
    info = {'target_pgn': target_pgn, 'raw': data}

    if len(data) >= 12:
        # priority/reserved(1) + num_params(1) + param pairs
        num_params = data[5] if len(data) > 5 else 0
        info['num_params'] = num_params

    return info


def decode_126208_acknowledge(data):
    """Decode PGN 126208 Acknowledge (FC=0x02)."""
    if len(data) < 4 or data[0] != 0x02:
        return None

    target_pgn = data[1] | (data[2] << 8) | (data[3] << 16)
    info = {'target_pgn': target_pgn, 'raw': data}

    # Acknowledge has: PGN(3) + PGN_error_code(1) + transmission_priority(1) +
    # num_params(1) + [param_error(1)]...
    if len(data) >= 7:
        pgn_err = data[4]
        tx_priority = data[5]
        num_params = data[6]
        info['pgn_error'] = pgn_err
        info['tx_priority'] = tx_priority
        info['num_params'] = num_params
        param_errors = []
        for i in range(num_params):
            if 7 + i < len(data):
                param_errors.append(data[7 + i])
        info['param_errors'] = param_errors

    return info


PGN_NAMES = {
    65345: "Seatalk Wind Datum",
    65359: "Seatalk Pilot Heading",
    65360: "Seatalk Locked Heading",
    65362: "Seatalk Pilot Speed",
    65379: "Seatalk Pilot Mode",
    65384: "Seatalk Keypad Heartbeat",
    127237: "Heading/Track Control",
    127245: "Rudder",
    127250: "Vessel Heading",
    126720: "Raymarine Proprietary",
    61184: "Proprietary Single-Frame",
}


def main():
    log_dir = Path("/Users/boston/ieb/autopilot/n2klogs/logged")
    frames = read_all_frames(log_dir)
    print(f"Total frames: {len(frames):,}")

    reassembler = FastPacketReassembler()

    # Collect all reassembled 126208 messages with timestamps
    messages_126208 = []
    for f in frames:
        if f['pgn'] == 126208:
            result = reassembler.process_frame(f['source'], f['pgn'], f['data'])
            if result:
                messages_126208.append({
                    'ts': f['ts'],
                    'source': f['source'],
                    'data': result,
                })

    print(f"Reassembled 126208 messages: {len(messages_126208)}")

    # =========================================================================
    # Analyze p70 (Source 0) REQUESTs
    # =========================================================================
    print("\n" + "=" * 80)
    print("P70 (Source 0) REQUEST MESSAGES")
    print("=" * 80)

    p70_requests = [m for m in messages_126208 if m['source'] == 0 and m['data'][0] == 0x00]
    print(f"\nTotal p70 requests: {len(p70_requests)}")

    # Group by requested PGN
    by_pgn = defaultdict(list)
    for m in p70_requests:
        info = decode_126208_request(m['data'])
        if info:
            by_pgn[info['requested_pgn']].append({**info, 'ts': m['ts']})

    for pgn in sorted(by_pgn):
        reqs = by_pgn[pgn]
        name = PGN_NAMES.get(pgn, f"PGN {pgn}")
        print(f"\n  Requesting PGN {pgn} ({name}): {len(reqs)} times")

        # Show unique request patterns
        patterns = Counter()
        for r in reqs:
            patterns[r['raw'].hex()] += 1

        for pattern, count in patterns.most_common(10):
            raw = bytes.fromhex(pattern)
            info = decode_126208_request(raw)
            print(f"    [{count:>4}x] {raw.hex(' ')}")
            if info and 'tx_interval' in info:
                interval_ms = info['tx_interval']
                if interval_ms == 0xFFFFFFFF:
                    print(f"           interval=default  offset={info.get('tx_offset', '?')}")
                else:
                    print(f"           interval={interval_ms}ms  offset={info.get('tx_offset', '?')}")
                if 'params' in info:
                    for idx, val in info['params']:
                        print(f"           param[{idx}] = {val.hex(' ')}")

        # Check timing / rate
        if len(reqs) > 1:
            intervals = [reqs[i+1]['ts'] - reqs[i]['ts'] for i in range(len(reqs)-1)]
            avg_interval = sum(intervals) / len(intervals)
            print(f"    Rate: ~{1.0/avg_interval:.1f} Hz (avg interval {avg_interval*1000:.0f}ms)")

    # =========================================================================
    # Analyze p70 COMMAND messages
    # =========================================================================
    print("\n" + "=" * 80)
    print("P70 (Source 0) COMMAND MESSAGES")
    print("=" * 80)

    p70_commands = [m for m in messages_126208 if m['source'] == 0 and m['data'][0] == 0x01]
    print(f"\nTotal p70 commands: {len(p70_commands)}")

    cmd_by_pgn = defaultdict(list)
    for m in p70_commands:
        info = decode_126208_command(m['data'])
        if info:
            cmd_by_pgn[info['target_pgn']].append({**info, 'ts': m['ts']})

    for pgn in sorted(cmd_by_pgn):
        cmds = cmd_by_pgn[pgn]
        name = PGN_NAMES.get(pgn, f"PGN {pgn}")
        print(f"\n  Commanding PGN {pgn} ({name}): {len(cmds)} times")

        patterns = Counter()
        for c in cmds:
            patterns[c['raw'].hex()] += 1

        for pattern, count in patterns.most_common(20):
            raw = bytes.fromhex(pattern)
            print(f"    [{count:>3}x] {raw.hex(' ')}")

            # Decode mode changes
            if pgn == 65379 and len(raw) >= 14:
                mode = raw[12]
                submode = raw[13]
                mode_names = {
                    (0x00, 0x00): "STANDBY",
                    (0x40, 0x00): "AUTO (Compass)",
                    (0x00, 0x01): "WIND",
                    (0x80, 0x01): "TRACK",
                    (0x81, 0x00): "NO DRIFT",
                }
                mn = mode_names.get((mode, submode), f"mode=0x{mode:02x} sub=0x{submode:02x}")
                print(f"           → {mn}")

            # Decode heading set
            if pgn == 65360 and len(raw) >= 14:
                heading_raw = raw[12] | (raw[13] << 8)
                heading_deg = math.degrees(heading_raw / 10000.0)
                print(f"           → heading={heading_deg:.1f}°")

            # Decode wind datum
            if pgn == 65345 and len(raw) >= 14:
                wind_raw = raw[12] | (raw[13] << 8)
                wind_deg = math.degrees(wind_raw / 10000.0)
                print(f"           → wind datum={wind_deg:.1f}°")

    # =========================================================================
    # Analyze EV-1 (Source 205) responses — FC=0x02 (Acknowledge)
    # =========================================================================
    print("\n" + "=" * 80)
    print("EV-1 (Source 205) ACKNOWLEDGE MESSAGES")
    print("=" * 80)

    ev1_acks = [m for m in messages_126208 if m['source'] == 205 and m['data'][0] == 0x02]
    print(f"\nTotal EV-1 acknowledges: {len(ev1_acks)}")

    ack_by_pgn = defaultdict(list)
    for m in ev1_acks:
        info = decode_126208_acknowledge(m['data'])
        if info:
            ack_by_pgn[info['target_pgn']].append({**info, 'ts': m['ts']})

    for pgn in sorted(ack_by_pgn):
        acks = ack_by_pgn[pgn]
        name = PGN_NAMES.get(pgn, f"PGN {pgn}")
        print(f"\n  Acknowledging PGN {pgn} ({name}): {len(acks)} times")

        patterns = Counter()
        for a in acks:
            patterns[a['raw'].hex()] += 1

        for pattern, count in patterns.most_common(10):
            raw = bytes.fromhex(pattern)
            info = decode_126208_acknowledge(raw)
            print(f"    [{count:>3}x] {raw.hex(' ')}")
            if info:
                err_names = {0: "OK", 1: "PGN not supported", 2: "PGN temporarily unavailable",
                             3: "Access denied", 4: "Request not supported"}
                pgn_err = info.get('pgn_error', 0xFF)
                err_name = err_names.get(pgn_err, f"error={pgn_err:#x}")
                print(f"           pgn_error={err_name}  num_params={info.get('num_params', '?')}")
                if 'param_errors' in info:
                    pe_names = {0: "OK", 1: "Invalid param", 2: "Temporary error",
                                3: "Param out of range", 4: "Access denied"}
                    for i, pe in enumerate(info['param_errors']):
                        pe_name = pe_names.get(pe, f"0x{pe:02x}")
                        print(f"           param[{i}] error: {pe_name}")

    # =========================================================================
    # Analyze EV-1 (Source 205) REQUEST messages (it also polls)
    # =========================================================================
    print("\n" + "=" * 80)
    print("EV-1 (Source 205) REQUEST MESSAGES")
    print("=" * 80)

    ev1_requests = [m for m in messages_126208 if m['source'] == 205 and m['data'][0] == 0x00]
    print(f"\nTotal EV-1 requests: {len(ev1_requests)}")

    ev1_req_by_pgn = defaultdict(list)
    for m in ev1_requests:
        info = decode_126208_request(m['data'])
        if info:
            ev1_req_by_pgn[info['requested_pgn']].append({**info, 'ts': m['ts']})

    for pgn in sorted(ev1_req_by_pgn):
        reqs = ev1_req_by_pgn[pgn]
        name = PGN_NAMES.get(pgn, f"PGN {pgn}")
        print(f"\n  Requesting PGN {pgn} ({name}): {len(reqs)} times")

        patterns = Counter()
        for r in reqs:
            patterns[r['raw'].hex()] += 1
        for pattern, count in patterns.most_common(5):
            raw = bytes.fromhex(pattern)
            print(f"    [{count:>4}x] {raw.hex(' ')}")

    # =========================================================================
    # Timeline: p70 command → EV-1 response correlation
    # =========================================================================
    print("\n" + "=" * 80)
    print("COMMAND → RESPONSE TIMELINE (first 30 p70 commands)")
    print("=" * 80)

    import datetime
    all_p70_cmds = sorted(
        [m for m in messages_126208 if m['source'] == 0 and m['data'][0] == 0x01],
        key=lambda m: m['ts']
    )
    all_ev1_responses = sorted(
        [m for m in messages_126208 if m['source'] == 205 and m['data'][0] == 0x02],
        key=lambda m: m['ts']
    )

    for cmd in all_p70_cmds[:30]:
        ts_str = datetime.datetime.fromtimestamp(cmd['ts']).strftime('%H:%M:%S.%f')[:-3]
        pgn = cmd['data'][1] | (cmd['data'][2] << 8) | (cmd['data'][3] << 16)
        name = PGN_NAMES.get(pgn, f"PGN {pgn}")

        detail = ""
        if pgn == 65379 and len(cmd['data']) >= 14:
            mode = cmd['data'][12]
            submode = cmd['data'][13]
            mode_map = {(0x00,0x00): "STANDBY", (0x40,0x00): "AUTO", (0x00,0x01): "WIND"}
            detail = f" → {mode_map.get((mode,submode), f'mode=0x{mode:02x} sub=0x{submode:02x}')}"
        elif pgn == 65360 and len(cmd['data']) >= 14:
            h = math.degrees((cmd['data'][12] | (cmd['data'][13] << 8)) / 10000.0)
            detail = f" → hdg={h:.1f}°"
        elif pgn == 65345 and len(cmd['data']) >= 14:
            w = math.degrees((cmd['data'][12] | (cmd['data'][13] << 8)) / 10000.0)
            detail = f" → wind={w:.1f}°"

        # Find closest EV-1 response within 500ms
        response = None
        for resp in all_ev1_responses:
            dt = resp['ts'] - cmd['ts']
            if dt > 0 and dt < 0.5:
                resp_pgn = resp['data'][1] | (resp['data'][2] << 8) | (resp['data'][3] << 16)
                if resp_pgn == pgn:
                    response = resp
                    break

        resp_str = ""
        if response:
            resp_dt = (response['ts'] - cmd['ts']) * 1000
            resp_info = decode_126208_acknowledge(response['data'])
            if resp_info:
                err = resp_info.get('pgn_error', 0xFF)
                err_name = {0: "OK", 1: "Not supported", 2: "Unavailable"}.get(err, f"err={err:#x}")
                resp_str = f"  ← EV-1 ACK ({resp_dt:.0f}ms): {err_name}"

        print(f"  {ts_str} p70 CMD {name}{detail}{resp_str}")

    # =========================================================================
    # What PGNs does the p70 need to see on the bus?
    # =========================================================================
    print("\n" + "=" * 80)
    print("PGNs THE P70 READS (ISO Request PGN 59904 from Source 0)")
    print("=" * 80)

    # Also check ISO Request (PGN 59904) from p70
    p70_iso_requests = [f for f in frames if f['pgn'] == 59904 and f['source'] == 0]
    print(f"\nISO Requests from p70: {len(p70_iso_requests)}")
    if p70_iso_requests:
        for f in p70_iso_requests[:10]:
            if f['dlc'] >= 3:
                req_pgn = f['data'][0] | (f['data'][1] << 8) | (f['data'][2] << 16)
                name = PGN_NAMES.get(req_pgn, f"PGN {req_pgn}")
                print(f"  Requests PGN {req_pgn} ({name})")

    # Check ISO Requests from all sources to understand the bus dynamics
    print("\n" + "=" * 80)
    print("ALL ISO REQUESTS (PGN 59904) SUMMARY")
    print("=" * 80)
    iso_reqs = [f for f in frames if f['pgn'] == 59904]
    req_summary = defaultdict(Counter)
    for f in iso_reqs:
        if f['dlc'] >= 3:
            req_pgn = f['data'][0] | (f['data'][1] << 8) | (f['data'][2] << 16)
            req_summary[f['source']][req_pgn] += 1

    for source in sorted(req_summary):
        print(f"\n  Source {source}:")
        for pgn, count in req_summary[source].most_common():
            name = PGN_NAMES.get(pgn, f"PGN {pgn}")
            print(f"    Requests PGN {pgn:6d} ({name}): {count}")


if __name__ == "__main__":
    main()
