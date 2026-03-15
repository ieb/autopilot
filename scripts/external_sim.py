#!/usr/bin/env python3
"""
External simulator for the HAL sim — drives yacht dynamics over TCP socket.

Connects to the HAL sim's socket port, sends CAN frames (N2K PGNs) and
IMU data from the Python yacht_dynamics model, and reads back rudder
commands from the firmware.

Keyboard acts as a Raymarine p70 control head:
    Left/Right arrows or a/d  : adjust heading +-1 degree
    Shift+Left/Right or A/D   : adjust heading +-10 degrees
    s or Space                 : standby
    c                          : compass (auto) mode
    w                          : wind (AWA) mode
    t                          : tack starboard
    T                          : tack port

Usage:
    uv run python scripts/external_sim.py [--tws 12] [--twd 225] [--host localhost] [--port 9876]
"""

import argparse
import math
import os
import select
import socket
import struct
import sys
import termios
import time
import tty

# Add project root to path
sys.path.insert(0, ".")
from src.simulation.yacht_dynamics import YachtDynamics, YachtState
from src.simulation.wave_model import WaveModel

# Wire protocol message types
MSG_CAN_RX = 0x01  # Python→ESP: CAN frame
MSG_CAN_TX = 0x02  # ESP→Python: CAN frame
MSG_IMU = 0x03  # Python→ESP: IMU registers

# N2K wind reference values
N2K_WIND_APPARENT = 2

# N2K CAN ID construction (29-bit)
# PDU2 (PGN >= 0xF000): priority(3) | 0 | data_page(1) | PGN(16) | source(8)


def make_can_id(pgn: int, source: int = 10, priority: int = 2) -> int:
    """Build 29-bit NMEA2000 CAN ID for broadcast (PDU2) PGNs."""
    return (priority << 26) | (pgn << 8) | source


def extract_pgn(can_id: int) -> int:
    """Extract PGN from a 29-bit NMEA2000 CAN ID."""
    # PDU format field is bits 16-23
    pdu_format = (can_id >> 16) & 0xFF
    if pdu_format < 240:
        # PDU1: PGN is bits 8-23 with PS field masked
        return (can_id >> 8) & 0x1FF00
    else:
        # PDU2: PGN is bits 8-23
        return (can_id >> 8) & 0x1FFFF


def n2k_2byte_udouble(value: float, resolution: float) -> bytes:
    """Encode a float as N2K unsigned 16-bit with given resolution."""
    raw = int(round(value / resolution))
    raw = max(0, min(0xFFFE, raw))
    return struct.pack("<H", raw)


def n2k_2byte_double(value: float, resolution: float) -> bytes:
    """Encode a float as N2K signed 16-bit with given resolution."""
    raw = int(round(value / resolution))
    raw = max(-0x7FFE, min(0x7FFE, raw))
    return struct.pack("<h", raw)


def encode_wind(awa_deg: float, aws_kts: float) -> tuple[int, bytes]:
    """Encode PGN 130306 (Wind Speed) — apparent only."""
    # AWA: convert signed to 0-360, then to radians
    awa_360 = awa_deg % 360.0
    awa_rad = math.radians(awa_360)
    aws_ms = aws_kts * 0.514444

    # SID + WindSpeed(UDouble*0.01) + WindAngle(UDouble*0.0001) + Ref + Reserved(2)
    data = (
        struct.pack("B", 0)  # SID
        + n2k_2byte_udouble(aws_ms, 0.01)
        + n2k_2byte_udouble(awa_rad, 0.0001)
        + struct.pack("B", N2K_WIND_APPARENT)
        + b"\xff\xff"  # reserved
    )
    return make_can_id(130306), data


def encode_stw(stw_kts: float) -> tuple[int, bytes]:
    """Encode PGN 128259 (Boat Speed)."""
    stw_ms = stw_kts * 0.514444
    # SID + WaterRef(UDouble*0.01) + GroundRef(UDouble*0.01) + SWRT + Reserved(2)
    data = (
        struct.pack("B", 0)
        + n2k_2byte_udouble(stw_ms, 0.01)
        + b"\xff\xff"  # ground ref N/A
        + struct.pack("B", 0)  # paddle wheel
        + b"\xff\xff"
    )
    return make_can_id(128259), data


def encode_cog_sog(cog_deg: float, sog_kts: float) -> tuple[int, bytes]:
    """Encode PGN 129026 (COG/SOG Rapid)."""
    cog_rad = math.radians(cog_deg % 360.0)
    sog_ms = sog_kts * 0.514444
    # SID + Ref(magnetic=0, packed) + COG(UDouble*0.0001) + SOG(UDouble*0.01) + Reserved(2)
    data = (
        struct.pack("B", 0)  # SID
        + struct.pack("B", 0xFC | 0x00)  # ref=magnetic
        + n2k_2byte_udouble(cog_rad, 0.0001)
        + n2k_2byte_udouble(sog_ms, 0.01)
        + b"\xff\xff"
    )
    return make_can_id(129026), data


def encode_heading(heading_deg: float) -> tuple[int, bytes]:
    """Encode PGN 127250 (Vessel Heading)."""
    heading_rad = math.radians(heading_deg % 360.0)
    # SID + Heading(UDouble*0.0001) + Deviation + Variation + Ref
    data = (
        struct.pack("B", 0)  # SID
        + n2k_2byte_udouble(heading_rad, 0.0001)
        + b"\xff\x7f"  # deviation N/A (signed)
        + b"\xff\x7f"  # variation N/A (signed)
        + struct.pack("B", 0xFC | 0x00)  # ref=magnetic
    )
    return make_can_id(127250), data


def send_can_frame(sock: socket.socket, can_id: int, data: bytes):
    """Send a CAN frame message (type 0x01) over the socket."""
    payload = struct.pack("<IB", can_id, len(data)) + data
    header = struct.pack("<BH", MSG_CAN_RX, len(payload))
    sock.sendall(header + payload)


def send_imu(
    sock: socket.socket,
    heading: float,
    roll: float,
    pitch: float,
    gyro_x: float,
    gyro_y: float,
    gyro_z: float,
):
    """Send IMU register update (type 0x03). BNO055 raw: 1 LSB = 1/16 deg or dps."""
    payload = struct.pack(
        "<6h",
        int(heading * 16),
        int(roll * 16),
        int(pitch * 16),
        int(gyro_x * 16),
        int(gyro_y * 16),
        int(gyro_z * 16),
    )
    header = struct.pack("<BH", MSG_IMU, len(payload))
    sock.sendall(header + payload)


def decode_rudder(data: bytes) -> tuple[float, float] | None:
    """Decode PGN 127245 (Rudder). Returns (actual_deg, commanded_deg) or None."""
    if len(data) < 6:
        return None
    # Instance(1) + DirOrder(1) + AngleOrder(2, signed*0.0001 rad) + RudderPos(2, signed*0.0001 rad)
    angle_order_raw = struct.unpack_from("<h", data, 2)[0]
    rudder_pos_raw = struct.unpack_from("<h", data, 4)[0]

    if angle_order_raw == 0x7FFF or rudder_pos_raw == 0x7FFF:
        return None

    commanded_rad = angle_order_raw * 0.0001
    actual_rad = rudder_pos_raw * 0.0001
    return math.degrees(actual_rad), math.degrees(commanded_rad)


def recv_messages(sock: socket.socket) -> list[tuple[int, bytes]]:
    """Read all available messages (non-blocking). Returns list of (type, payload)."""
    messages = []
    sock.setblocking(False)
    buf = b""
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Socket closed")
            buf += chunk
    except BlockingIOError:
        pass
    finally:
        sock.setblocking(True)

    # Parse framed messages from buffer
    while len(buf) >= 3:
        msg_type = buf[0]
        length = buf[1] | (buf[2] << 8)
        if len(buf) < 3 + length:
            break
        payload = buf[3 : 3 + length]
        buf = buf[3 + length :]
        messages.append((msg_type, payload))

    return messages


def parse_can_frame(payload: bytes) -> tuple[int, bytes] | None:
    """Parse a CAN frame from message payload. Returns (can_id, data)."""
    if len(payload) < 5:
        return None
    can_id = struct.unpack_from("<I", payload, 0)[0]
    dlc = payload[4]
    data = payload[5 : 5 + dlc]
    return can_id, data


# ============================================================================
# p70 emulation — send Seatalk PGNs as CAN frames
# ============================================================================

# Raymarine manufacturer header
RAYMARINE_MFR_LO = 0x3B
RAYMARINE_MFR_HI = 0x9F

# Keystroke codes
KEY_PLUS_1 = 0x07F8
KEY_MINUS_1 = 0x05FA
KEY_PLUS_10 = 0x08F7
KEY_MINUS_10 = 0x06F9
KEY_TACK_STBD = 0x22DD
KEY_TACK_PORT = 0x21DE

# Mode bytes
SEATALK_MODE_STANDBY = 0x00
SEATALK_MODE_AUTO = 0x40
SEATALK_SUBMODE_STANDBY = 0x00
SEATALK_SUBMODE_WIND = 0x01

P70_SOURCE = 70  # Simulated p70 source address
_fast_packet_seq = 0  # 3-bit sequence counter (0-7)

# Tracked target state (p70 sends absolute values, not deltas)
_target_heading = None  # float or None (not yet initialized)
_target_wind = None     # float or None (not yet initialized)


def _next_fp_seq() -> int:
    """Increment and return fast-packet sequence counter (0-7)."""
    global _fast_packet_seq
    seq = _fast_packet_seq
    _fast_packet_seq = (seq + 1) & 0x07
    return seq


def send_fast_packet(sock: socket.socket, pgn: int, data: bytes, source: int = P70_SOURCE):
    """Send a multi-frame fast-packet N2K message as individual CAN frames."""
    can_id = make_can_id(pgn, source=source, priority=3)
    seq = _next_fp_seq()
    total_len = len(data)
    offset = 0

    # Frame 0: [seq<<5 | 0] [total_len] [up to 6 data bytes]
    frame0 = bytes([(seq << 5) | 0, total_len]) + data[0:6]
    frame0 = frame0.ljust(8, b"\xff")
    send_can_frame(sock, can_id, frame0)
    offset = 6

    frame_num = 1
    while offset < total_len:
        chunk = data[offset : offset + 7]
        frame = bytes([(seq << 5) | frame_num]) + chunk
        frame = frame.ljust(8, b"\xff")
        send_can_frame(sock, can_id, frame)
        offset += 7
        frame_num += 1


def send_p70_keystroke(sock: socket.socket, key_code: int):
    """Send PGN 126720 keystroke command (22 bytes, fast-packet)."""
    data = bytes([
        RAYMARINE_MFR_LO, RAYMARINE_MFR_HI,
        0xF0, 0x81, 0x86, 0x21,
        (key_code >> 8) & 0xFF, key_code & 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xC1, 0xC2, 0xCD, 0x66, 0x80, 0xD3, 0x42, 0xB1, 0xC8,
    ])
    send_fast_packet(sock, 126720, data)


def send_p70_mode(sock: socket.socket, mode: int, submode: int):
    """Send PGN 126208 mode change command (17 bytes, fast-packet)."""
    data = bytes([
        0x01, 0x63, 0xFF, 0x00, 0xF8, 0x04, 0x01, 0x3B,
        0x07, 0x03, 0x04, 0x04, mode, submode, 0x05, 0xFF, 0xFF,
    ])
    send_fast_packet(sock, 126208, data)


def send_p70_heartbeat(sock: socket.socket):
    """Send PGN 65374 heartbeat (8 bytes, single frame)."""
    data = bytes([RAYMARINE_MFR_LO, RAYMARINE_MFR_HI, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF])
    can_id = make_can_id(65374, source=P70_SOURCE, priority=7)
    send_can_frame(sock, can_id, data)


def send_p70_heading_set(sock: socket.socket, heading_deg: float):
    """Send PGN 126208 CMD→PGN 65360 with absolute heading value."""
    heading_rad = math.radians(heading_deg % 360.0)
    heading_raw = int(heading_rad * 10000)
    lo, hi = heading_raw & 0xFF, (heading_raw >> 8) & 0xFF
    data = bytes([0x01, 0x50, 0xFF, 0x00, 0xF8, 0x03, 0x01, 0x3B,
                  0x07, 0x03, 0x04, 0x06, lo, hi])
    send_fast_packet(sock, 126208, data)


def send_p70_wind_datum(sock: socket.socket, wind_deg: float):
    """Send PGN 126208 CMD→PGN 65345 with absolute wind angle."""
    wind_360 = wind_deg % 360.0
    wind_rad = math.radians(wind_360)
    wind_raw = int(wind_rad * 10000)
    lo, hi = wind_raw & 0xFF, (wind_raw >> 8) & 0xFF
    data = bytes([0x01, 0x41, 0xFF, 0x00, 0xF8, 0x03, 0x01, 0x3B,
                  0x07, 0x03, 0x04, 0x06, lo, hi])
    send_fast_packet(sock, 126208, data)


# ============================================================================
# Firmware broadcast parsing (track current targets for +/- adjustments)
# ============================================================================

def parse_seatalk_locked_heading(data: bytes) -> float | None:
    """Parse PGN 65360 (locked heading) from firmware broadcast. Returns degrees or None."""
    if len(data) < 8:
        return None
    # Bytes 0-1: Raymarine header, byte 2: SID, bytes 3-4: true (N/A),
    # bytes 5-6: magnetic heading raw, byte 7: reserved
    if data[0] != RAYMARINE_MFR_LO or data[1] != RAYMARINE_MFR_HI:
        return None
    raw = struct.unpack_from("<H", data, 5)[0]
    if raw == 0xFFFF:
        return None
    return math.degrees(raw / 10000.0)


def parse_seatalk_wind_datum(data: bytes) -> float | None:
    """Parse PGN 65345 (wind datum) from firmware broadcast. Returns signed degrees or None."""
    if len(data) < 4:
        return None
    if data[0] != RAYMARINE_MFR_LO or data[1] != RAYMARINE_MFR_HI:
        return None
    raw = struct.unpack_from("<H", data, 2)[0]
    if raw == 0xFFFF:
        return None
    deg = math.degrees(raw / 10000.0)
    if deg > 180.0:
        deg -= 360.0
    return deg


# ============================================================================
# Keyboard input (raw terminal, non-blocking)
# ============================================================================

class RawTerminal:
    """Context manager for raw terminal input."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        # Set non-blocking
        os.set_blocking(self.fd, False)
        return self

    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self) -> str | None:
        """Read a keypress. Returns key string or None if nothing available."""
        try:
            ch = os.read(self.fd, 1)
        except (BlockingIOError, OSError):
            return None
        if not ch:
            return None
        b = ch[0]
        if b == 0x1B:  # Escape sequence
            try:
                seq = os.read(self.fd, 2)
            except (BlockingIOError, OSError):
                return "ESC"
            if len(seq) == 2 and seq[0] == ord("["):
                code = chr(seq[1])
                # Check for modifier (e.g. 1;2C = shift+right)
                if code == "1":
                    try:
                        rest = os.read(self.fd, 3)
                    except (BlockingIOError, OSError):
                        return None
                    if len(rest) >= 2 and rest[0] == ord(";") and rest[1] == ord("2"):
                        arrow = chr(rest[2]) if len(rest) > 2 else ""
                        if arrow == "C":
                            return "SHIFT_RIGHT"
                        elif arrow == "D":
                            return "SHIFT_LEFT"
                    return None
                elif code == "C":
                    return "RIGHT"
                elif code == "D":
                    return "LEFT"
            return None
        return chr(b)


def handle_keypress(key: str, sock: socket.socket, pilot_mode: str) -> str | None:
    """Process a keypress and send the appropriate p70 CAN message.

    Uses PGN 126208 absolute commands (matching real p70 behavior) for heading
    and wind adjustments. Keystroke path kept only for tack (Raymarine remote).

    Args:
        key: Key string from RawTerminal
        sock: Socket to firmware
        pilot_mode: Current pilot mode string ("standby", "compass", "wind")

    Returns action description or None.
    """
    global _target_heading, _target_wind

    if key in ("d", "RIGHT"):
        if pilot_mode == "wind" and _target_wind is not None:
            _target_wind += 1.0
            send_p70_wind_datum(sock, _target_wind)
        elif _target_heading is not None:
            _target_heading = (_target_heading + 1.0) % 360.0
            send_p70_heading_set(sock, _target_heading)
        return "+1"
    elif key in ("a", "LEFT"):
        if pilot_mode == "wind" and _target_wind is not None:
            _target_wind -= 1.0
            send_p70_wind_datum(sock, _target_wind)
        elif _target_heading is not None:
            _target_heading = (_target_heading - 1.0) % 360.0
            send_p70_heading_set(sock, _target_heading)
        return "-1"
    elif key in ("D", "SHIFT_RIGHT"):
        if pilot_mode == "wind" and _target_wind is not None:
            _target_wind += 10.0
            send_p70_wind_datum(sock, _target_wind)
        elif _target_heading is not None:
            _target_heading = (_target_heading + 10.0) % 360.0
            send_p70_heading_set(sock, _target_heading)
        return "+10"
    elif key in ("A", "SHIFT_LEFT"):
        if pilot_mode == "wind" and _target_wind is not None:
            _target_wind -= 10.0
            send_p70_wind_datum(sock, _target_wind)
        elif _target_heading is not None:
            _target_heading = (_target_heading - 10.0) % 360.0
            send_p70_heading_set(sock, _target_heading)
        return "-10"
    elif key in ("s", " "):
        send_p70_mode(sock, SEATALK_MODE_STANDBY, SEATALK_SUBMODE_STANDBY)
        _target_heading = None
        _target_wind = None
        return "STANDBY"
    elif key == "c":
        send_p70_mode(sock, SEATALK_MODE_AUTO, SEATALK_SUBMODE_STANDBY)
        # Target heading will be initialized from firmware broadcast
        return "COMPASS"
    elif key == "w":
        send_p70_mode(sock, SEATALK_MODE_STANDBY, SEATALK_SUBMODE_WIND)
        # Target wind will be initialized from firmware broadcast
        return "WIND"
    elif key == "t":
        # Tack: negate wind target and send as absolute
        if _target_wind is not None:
            _target_wind = -_target_wind
            send_p70_wind_datum(sock, _target_wind)
        else:
            send_p70_keystroke(sock, KEY_TACK_STBD)
        return "TACK STBD"
    elif key == "T":
        if _target_wind is not None:
            _target_wind = -_target_wind
            send_p70_wind_datum(sock, _target_wind)
        else:
            send_p70_keystroke(sock, KEY_TACK_PORT)
        return "TACK PORT"
    return None


def main():
    parser = argparse.ArgumentParser(description="External simulator for HAL sim")
    parser.add_argument("--host", default="localhost", help="HAL sim host")
    parser.add_argument("--port", type=int, default=9876, help="HAL sim socket port")
    parser.add_argument("--tws", type=float, default=12.0, help="True wind speed (kts)")
    parser.add_argument("--twd", type=float, default=225.0, help="True wind direction (deg)")
    parser.add_argument("--heading", type=float, default=180.0, help="Initial heading (deg)")
    parser.add_argument("--dt", type=float, default=0.05, help="Timestep (seconds)")
    parser.add_argument(
        "--sea",
        choices=["calm", "moderate", "rough", "none"],
        default="moderate",
        help="Sea state for wave model (default: moderate)",
    )
    args = parser.parse_args()

    # Initialize yacht dynamics
    yacht = YachtDynamics()
    yacht.reset(heading=args.heading, twd=args.twd, tws=args.tws)

    # Initialize wave model
    if args.sea == "none":
        waves = None
    elif args.sea == "calm":
        waves = WaveModel.calm()
    elif args.sea == "rough":
        waves = WaveModel.rough()
    else:
        waves = WaveModel.moderate()

    print(f"Connecting to {args.host}:{args.port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.host, args.port))
    except ConnectionRefusedError:
        print(f"Connection refused. Is the HAL sim running with --socket-port {args.port}?")
        return 1

    print(f"Connected. TWS={args.tws:.1f} kn TWD={args.twd:.0f}° heading={args.heading:.0f}° sea={args.sea}")
    print()
    print("+---------------------------------------------------------+")
    print("|                  Raymarine p70 Controls                  |")
    print("+---------------------------------------------------------+")
    print("|  Heading adjust:                                        |")
    print("|    a / Left arrow        -1 degree                      |")
    print("|    d / Right arrow       +1 degree                      |")
    print("|    A / Shift+Left       -10 degrees                     |")
    print("|    D / Shift+Right      +10 degrees                     |")
    print("|                                                         |")
    print("|  Mode select:                                           |")
    print("|    c                     Compass (auto)                 |")
    print("|    w                     Wind (AWA)                     |")
    print("|    s / Space             Standby                        |")
    print("|                                                         |")
    print("|  Tack:                                                  |")
    print("|    t                     Tack starboard                 |")
    print("|    T                     Tack port                      |")
    print("|                                                         |")
    print("|  q / Ctrl-C              Quit                           |")
    print("+---------------------------------------------------------+")
    print()

    rudder_from_firmware = 0.0  # degrees, from PGN 127245
    step_count = 0
    dt = args.dt
    prev_pitch = 0.0  # for computing pitch rate from wave model
    last_heartbeat = 0.0
    last_action = ""  # last p70 action for display
    pilot_mode = "standby"  # tracked from firmware PGN 65379 broadcasts

    with RawTerminal() as term:
        try:
            while True:
                t_start = time.monotonic()

                # Handle keyboard input (p70 emulation)
                key = term.read_key()
                if key == "q" or key == "\x03":  # q or Ctrl-C
                    break
                if key is not None:
                    action = handle_keypress(key, sock, pilot_mode)
                    if action:
                        last_action = action

                # Send p70 heartbeat at ~1 Hz
                now = time.monotonic()
                if now - last_heartbeat >= 1.0:
                    last_heartbeat = now
                    send_p70_heartbeat(sock)

                # Step wave model (if enabled)
                wave_roll = 0.0
                wave_pitch = 0.0
                wave_yaw = 0.0
                if waves is not None:
                    wave_roll, wave_pitch, wave_yaw = waves.step(
                        dt,
                        tws=args.tws,
                        heading=yacht.state.heading,
                        twa=yacht.state.twd - yacht.state.heading,
                        heel=yacht.state.roll,
                    )

                # Step yacht dynamics with rudder command + wave yaw perturbation
                state = yacht.step(rudder_from_firmware, dt, wave_yaw=wave_yaw)

                # Compute pitch rate from wave-induced pitch changes
                total_pitch = state.pitch + wave_pitch
                pitch_rate = (total_pitch - prev_pitch) / dt
                prev_pitch = total_pitch

                # Send CAN frames: wind, STW, COG/SOG, heading
                can_id, data = encode_wind(state.awa, state.aws)
                send_can_frame(sock, can_id, data)

                can_id, data = encode_stw(state.stw)
                send_can_frame(sock, can_id, data)

                can_id, data = encode_cog_sog(state.cog, state.sog)
                send_can_frame(sock, can_id, data)

                can_id, data = encode_heading(state.heading)
                send_can_frame(sock, can_id, data)

                # Send IMU data — add wave-induced pitch and roll to yacht state
                send_imu(
                    sock,
                    heading=state.heading,
                    roll=state.roll + wave_roll,
                    pitch=total_pitch,
                    gyro_x=state.roll_rate,
                    gyro_y=pitch_rate,
                    gyro_z=state.heading_rate,
                )

                # Read responses from firmware (rudder PGN + Seatalk broadcasts)
                try:
                    messages = recv_messages(sock)
                    for msg_type, payload in messages:
                        if msg_type == MSG_CAN_TX:
                            result = parse_can_frame(payload)
                            if result is None:
                                continue
                            can_id, frame_data = result
                            pgn = extract_pgn(can_id)
                            if pgn == 127245:  # Rudder
                                rudder = decode_rudder(frame_data)
                                if rudder:
                                    actual_deg, commanded_deg = rudder
                                    rudder_from_firmware = commanded_deg
                            elif pgn == 65379:  # Pilot mode
                                if len(frame_data) >= 6:
                                    mode_byte = frame_data[3]
                                    submode_byte = frame_data[5]
                                    if mode_byte == SEATALK_MODE_AUTO:
                                        pilot_mode = "compass"
                                    elif mode_byte == SEATALK_MODE_STANDBY and submode_byte == SEATALK_SUBMODE_WIND:
                                        pilot_mode = "wind"
                                    else:
                                        pilot_mode = "standby"
                            elif pgn == 65360:  # Locked heading
                                hdg = parse_seatalk_locked_heading(frame_data)
                                if hdg is not None and _target_heading is None:
                                    _target_heading = hdg
                            elif pgn == 65345:  # Wind datum
                                wnd = parse_seatalk_wind_datum(frame_data)
                                if wnd is not None and _target_wind is None:
                                    _target_wind = wnd
                except ConnectionError:
                    print("\r\nConnection lost")
                    break

                step_count += 1
                if step_count % 20 == 0:  # Print at ~1 Hz (if dt=0.05)
                    action_str = f" [{last_action}]" if last_action else ""
                    sys.stdout.write(
                        f"\r[{step_count * dt:6.1f}s] "
                        f"hdg={state.heading:5.1f}\xb0 "
                        f"awa={state.awa:5.1f}\xb0 "
                        f"stw={state.stw:4.1f}kn "
                        f"rud={rudder_from_firmware:+5.1f}\xb0 "
                        f"roll={state.roll + wave_roll:+5.1f}\xb0 "
                        f"pitch={state.pitch + wave_pitch:+4.1f}\xb0 "
                        f"wyaw={wave_yaw:+4.1f}"
                        f"{action_str:>14s}"
                        "    "
                    )
                    sys.stdout.flush()
                    last_action = ""

                # Pace the loop
                elapsed = time.monotonic() - t_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass

    print("\r\nStopping.")
    sock.close()

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
