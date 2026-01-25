"""
CAN Simulator
=============

Simulates NMEA2000 bus by emitting PGN frames.

On Linux: Uses virtual CAN interface (vcan0) via SocketCAN.
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

On macOS: Runs in stub mode (logs frames but cannot send to real CAN bus).
    For full CAN testing, use a Linux VM or container.

Emits the following PGNs:
    - 130306: Wind Data (AWA, AWS)
    - 128259: Speed (STW)
    - 129026: COG/SOG
    - 127250: Vessel Heading
    - 129029: GNSS Position
"""

import time
import struct
import math
import logging
import sys
from dataclasses import dataclass
from typing import Optional, List, Tuple
from multiprocessing.managers import DictProxy

logger = logging.getLogger(__name__)

# Check platform
IS_LINUX = sys.platform.startswith('linux')
IS_MACOS = sys.platform == 'darwin'

# Try to import python-can
try:
    import can
    HAS_CAN = True
except ImportError:
    can = None
    HAS_CAN = False


# PGN definitions
class PGN:
    WIND_DATA = 130306
    SPEED = 128259
    COG_SOG = 129026
    VESSEL_HEADING = 127250
    GNSS_POSITION = 129029
    RUDDER = 127245


@dataclass
class CANSimulatorConfig:
    """Configuration for CAN simulator."""
    channel: str = "vcan0"
    bustype: str = "socketcan"
    update_rate_hz: float = 1.0  # PGN transmission rate
    source_address: int = 42     # Our source address on the bus
    stub_mode: bool = False      # If True, don't send to CAN bus (for macOS/testing)
    

class CANSimulator:
    """
    Simulates NMEA2000 bus on virtual CAN interface.
    
    Reads yacht state from shared memory and transmits PGN frames
    at realistic rates (~1Hz for most navigation data).
    
    On macOS or when stub_mode=True, runs without a real CAN bus,
    which is useful for testing the autopilot logic without CAN hardware.
    """
    
    def __init__(
        self,
        shared_state: DictProxy,
        config: Optional[CANSimulatorConfig] = None
    ):
        self.config = config or CANSimulatorConfig()
        self.state = shared_state
        self.running = False
        self._stub_mode = self.config.stub_mode
        
        self._bus: Optional['can.interface.Bus'] = None
        self._sid = 0  # Sequence ID for PGNs
        
        # Frame buffer for stub mode (for testing/inspection)
        self._frame_buffer: List[Tuple[int, bytes]] = []
        
    def start(self) -> bool:
        """Start CAN bus interface."""
        # Auto-enable stub mode on macOS
        if IS_MACOS and not self._stub_mode:
            logger.warning(
                "vcan not available on macOS - running in stub mode. "
                "CAN frames will be generated but not sent to a real bus."
            )
            self._stub_mode = True
            
        if self._stub_mode:
            self.running = True
            logger.info("CAN simulator started in stub mode (no real CAN bus)")
            return True
            
        if not HAS_CAN:
            logger.error("python-can not installed. Run: pip install python-can")
            return False
            
        try:
            self._bus = can.interface.Bus(
                channel=self.config.channel,
                bustype=self.config.bustype
            )
            self.running = True
            logger.info(f"CAN simulator started on {self.config.channel}")
            return True
        except Exception as e:
            logger.warning(f"Failed to open CAN bus: {e} - falling back to stub mode")
            self._stub_mode = True
            self.running = True
            return True
            
    def stop(self):
        """Stop CAN bus interface."""
        self.running = False
        if self._bus:
            self._bus.shutdown()
            self._bus = None
        logger.info("CAN simulator stopped")
        
    def step(self):
        """Transmit all PGN frames."""
        if not self._bus and not self._stub_mode:
            return
            
        self._sid = (self._sid + 1) % 253
        
        # Send all PGNs
        self._send_wind_data()
        self._send_speed()
        self._send_cog_sog()
        self._send_heading()
        self._send_position()
        
    def _make_can_id(self, pgn: int, priority: int = 6) -> int:
        """
        Create CAN arbitration ID for a PGN.
        
        Format: Priority(3) | Reserved(1) | DataPage(1) | PDU Format(8) | PDU Specific(8) | Source(8)
        """
        # Extract data page
        data_page = (pgn >> 16) & 0x01
        
        # PDU format is bits 15-8 of PGN
        pdu_format = (pgn >> 8) & 0xFF
        
        # PDU specific is bits 7-0 for PDU2 format (pdu_format >= 240)
        if pdu_format >= 240:
            pdu_specific = pgn & 0xFF
        else:
            pdu_specific = 0xFF  # Broadcast for PDU1
            
        can_id = (
            ((priority & 0x07) << 26) |
            (data_page << 24) |
            (pdu_format << 16) |
            (pdu_specific << 8) |
            (self.config.source_address & 0xFF)
        )
        
        return can_id
        
    def _send_pgn(self, pgn: int, data: bytes, priority: int = 6):
        """Send a PGN frame."""
        can_id = self._make_can_id(pgn, priority)
        
        if self._stub_mode:
            # Store frame for inspection (keep last 100 frames)
            self._frame_buffer.append((pgn, data))
            if len(self._frame_buffer) > 100:
                self._frame_buffer.pop(0)
            return
            
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        try:
            self._bus.send(msg)
        except Exception as e:
            logger.warning(f"Failed to send PGN {pgn}: {e}")
            
    def get_frame_buffer(self) -> List[Tuple[int, bytes]]:
        """Get buffered frames (stub mode only, for testing)."""
        return list(self._frame_buffer)
        
    def clear_frame_buffer(self):
        """Clear the frame buffer."""
        self._frame_buffer.clear()
            
    def _send_wind_data(self):
        """
        Send PGN 130306: Wind Data
        
        Byte 0: SID
        Bytes 1-2: Wind speed (0.01 m/s)
        Bytes 3-4: Wind angle (0.0001 rad)
        Byte 5: Reference (2 = Apparent)
        """
        aws_kts = self.state.get('aws', 0.0)
        awa_deg = self.state.get('awa', 0.0)
        
        # Convert to NMEA2000 units
        aws_ms = aws_kts / 1.94384  # knots to m/s
        awa_rad = awa_deg * math.pi / 180.0
        
        # Handle negative angles
        if awa_rad < 0:
            awa_rad += 2 * math.pi
            
        speed_raw = int(aws_ms / 0.01)
        angle_raw = int(awa_rad / 0.0001)
        
        data = struct.pack('<BHHBxx',
            self._sid,
            min(speed_raw, 0xFFFE),
            min(angle_raw, 0xFFFE),
            2  # Apparent wind reference
        )
        
        self._send_pgn(PGN.WIND_DATA, data)
        
    def _send_speed(self):
        """
        Send PGN 128259: Speed
        
        Byte 0: SID
        Bytes 1-2: STW (0.01 m/s)
        Bytes 3-4: SOG (0.01 m/s)
        Byte 5: Speed water type
        """
        stw_kts = self.state.get('stw', 0.0)
        sog_kts = self.state.get('sog', stw_kts)
        
        stw_ms = stw_kts / 1.94384
        sog_ms = sog_kts / 1.94384
        
        stw_raw = int(stw_ms / 0.01)
        sog_raw = int(sog_ms / 0.01)
        
        data = struct.pack('<BHHB',
            self._sid,
            min(stw_raw, 0xFFFE),
            min(sog_raw, 0xFFFE),
            0  # Paddle wheel
        )
        
        self._send_pgn(PGN.SPEED, data)
        
    def _send_cog_sog(self):
        """
        Send PGN 129026: COG & SOG, Rapid Update
        
        Byte 0: SID
        Byte 1: COG Reference
        Bytes 2-3: COG (0.0001 rad)
        Bytes 4-5: SOG (0.01 m/s)
        """
        cog_deg = self.state.get('cog', self.state.get('heading', 0.0))
        sog_kts = self.state.get('sog', self.state.get('stw', 0.0))
        
        cog_rad = cog_deg * math.pi / 180.0
        sog_ms = sog_kts / 1.94384
        
        cog_raw = int(cog_rad / 0.0001)
        sog_raw = int(sog_ms / 0.01)
        
        data = struct.pack('<BBHH',
            self._sid,
            0,  # True reference
            min(cog_raw, 0xFFFE),
            min(sog_raw, 0xFFFE)
        )
        
        self._send_pgn(PGN.COG_SOG, data)
        
    def _send_heading(self):
        """
        Send PGN 127250: Vessel Heading
        
        Byte 0: SID
        Bytes 1-2: Heading (0.0001 rad)
        Bytes 3-4: Deviation (0.0001 rad)
        Bytes 5-6: Variation (0.0001 rad)
        Byte 7: Reference
        """
        heading_deg = self.state.get('heading', 0.0)
        heading_rad = heading_deg * math.pi / 180.0
        heading_raw = int(heading_rad / 0.0001)
        
        data = struct.pack('<BHHHB',
            self._sid,
            min(heading_raw, 0xFFFE),
            0x7FFF,  # Deviation not available
            0x7FFF,  # Variation not available
            0  # True reference
        )
        
        self._send_pgn(PGN.VESSEL_HEADING, data)
        
    def _send_position(self):
        """
        Send PGN 129029: GNSS Position
        
        This is a multi-frame fast packet - simplified single frame for basic position.
        """
        lat = self.state.get('latitude', 0.0)
        lon = self.state.get('longitude', 0.0)
        
        # Convert to raw format (scaled integers)
        lat_raw = int(lat * 1e7)
        lon_raw = int(lon * 1e7)
        
        # Simplified 8-byte message with lat/lon only
        data = struct.pack('<ii',
            lat_raw,
            lon_raw
        )
        
        self._send_pgn(PGN.GNSS_POSITION, data)


def run_can_sim(
    shared_state: DictProxy,
    config: Optional[CANSimulatorConfig] = None
):
    """
    Entry point for running CAN simulator as a separate process.
    
    Args:
        shared_state: Shared state dict from multiprocessing.Manager
        config: Optional configuration
    """
    import signal
    
    config = config or CANSimulatorConfig()
    sim = CANSimulator(shared_state, config)
    
    # Handle shutdown signals
    running = True
    def shutdown(signum, frame):
        nonlocal running
        running = False
        
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    
    if sim.start():
        logger.info("CAN simulator running")
        update_interval = 1.0 / config.update_rate_hz
        next_update = time.time()
        
        try:
            while running and sim.running:
                now = time.time()
                if now >= next_update:
                    sim.step()
                    next_update += update_interval
                    if next_update < now:
                        next_update = now + update_interval
                else:
                    time.sleep(min(0.01, next_update - now))
        except KeyboardInterrupt:
            pass
        finally:
            sim.stop()
    else:
        logger.error("Failed to start CAN simulator")
