#!/usr/bin/env python3
"""
External simulator for the HAL sim — drives yacht dynamics over TCP socket.

Connects to the HAL sim's socket port, sends CAN frames (N2K PGNs) and
IMU data from the Python yacht_dynamics model, and reads back rudder
commands from the firmware.

Usage:
    uv run python scripts/external_sim.py [--tws 12] [--twd 225] [--host localhost] [--port 9876]
"""

import argparse
import math
import select
import socket
import struct
import sys
import time

# Add project root to path
sys.path.insert(0, ".")
from src.simulation.yacht_dynamics import YachtDynamics, YachtState

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


def main():
    parser = argparse.ArgumentParser(description="External simulator for HAL sim")
    parser.add_argument("--host", default="localhost", help="HAL sim host")
    parser.add_argument("--port", type=int, default=9876, help="HAL sim socket port")
    parser.add_argument("--tws", type=float, default=12.0, help="True wind speed (kts)")
    parser.add_argument("--twd", type=float, default=225.0, help="True wind direction (deg)")
    parser.add_argument("--heading", type=float, default=180.0, help="Initial heading (deg)")
    parser.add_argument("--dt", type=float, default=0.05, help="Timestep (seconds)")
    args = parser.parse_args()

    # Initialize yacht dynamics
    yacht = YachtDynamics()
    yacht.reset(heading=args.heading, twd=args.twd, tws=args.tws)

    print(f"Connecting to {args.host}:{args.port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.host, args.port))
    except ConnectionRefusedError:
        print(f"Connection refused. Is the HAL sim running with --socket-port {args.port}?")
        return 1

    print(f"Connected. TWS={args.tws:.1f} kn TWD={args.twd:.0f}° heading={args.heading:.0f}°")
    print("Press Ctrl+C to stop.\n")

    rudder_from_firmware = 0.0  # degrees, from PGN 127245
    step_count = 0
    dt = args.dt

    try:
        while True:
            t_start = time.monotonic()

            # Step yacht dynamics with the rudder command from firmware
            state = yacht.step(rudder_from_firmware, dt)

            # Send CAN frames: wind, STW, COG/SOG, heading
            can_id, data = encode_wind(state.awa, state.aws)
            send_can_frame(sock, can_id, data)

            can_id, data = encode_stw(state.stw)
            send_can_frame(sock, can_id, data)

            can_id, data = encode_cog_sog(state.cog, state.sog)
            send_can_frame(sock, can_id, data)

            can_id, data = encode_heading(state.heading)
            send_can_frame(sock, can_id, data)

            # Send IMU data
            send_imu(
                sock,
                heading=state.heading,
                roll=state.roll,
                pitch=state.pitch,
                gyro_x=state.roll_rate,
                gyro_y=0.0,
                gyro_z=state.heading_rate,
            )

            # Read responses from firmware (rudder PGN)
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
            except ConnectionError:
                print("\nConnection lost")
                break

            step_count += 1
            if step_count % 20 == 0:  # Print at ~1 Hz (if dt=0.05)
                print(
                    f"\r[{step_count * dt:6.1f}s] "
                    f"hdg={state.heading:5.1f}° "
                    f"awa={state.awa:5.1f}° "
                    f"aws={state.aws:4.1f}kn "
                    f"stw={state.stw:4.1f}kn "
                    f"rud={rudder_from_firmware:+5.1f}° "
                    f"yaw={state.heading_rate:+5.1f}°/s",
                    end="",
                    flush=True,
                )

            # Pace the loop
            elapsed = time.monotonic() - t_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nStopping.")
    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
