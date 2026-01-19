"""
NMEA2000 Interface Module
=========================

Interfaces with the NMEA2000 bus via CandleLite USB-CAN adapter.
Uses python-can for SocketCAN interface and decodes relevant PGNs.

PGN data arrives at ~1Hz from the bus, providing context for the ML model.
The IMU provides the high-frequency control feedback.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, Any
from enum import IntEnum
import struct
import logging

try:
    import can
except ImportError:
    can = None
    
logger = logging.getLogger(__name__)


class PGN(IntEnum):
    """Relevant NMEA2000 Parameter Group Numbers."""
    # Navigation
    VESSEL_HEADING = 127250
    RATE_OF_TURN = 127251
    ATTITUDE = 127257
    
    # Speed
    SPEED = 128259
    COG_SOG = 129026
    
    # Position
    GNSS_POSITION = 129029
    
    # Environment  
    WIND_DATA = 130306
    
    # Autopilot
    RUDDER = 127245
    HEADING_TRACK_CONTROL = 127237


@dataclass
class N2KData:
    """
    Aggregated NMEA2000 data from the bus.
    
    All angles in degrees, speeds in knots unless otherwise noted.
    """
    timestamp: float = 0.0
    
    # Wind (from PGN 130306)
    awa: float = 0.0          # Apparent wind angle (degrees, 0=bow, +ve starboard)
    aws: float = 0.0          # Apparent wind speed (knots)
    awa_timestamp: float = 0.0
    
    # Speed (from PGN 128259)
    stw: float = 0.0          # Speed through water (knots)
    stw_timestamp: float = 0.0
    
    # COG/SOG (from PGN 129026)
    cog: float = 0.0          # Course over ground (degrees true)
    sog: float = 0.0          # Speed over ground (knots)
    cog_sog_timestamp: float = 0.0
    
    # Heading (from PGN 127250) - secondary to IMU
    heading_n2k: float = 0.0  # Magnetic heading from bus (degrees)
    heading_timestamp: float = 0.0
    
    # Position (from PGN 129029)
    latitude: float = 0.0     # Decimal degrees
    longitude: float = 0.0    # Decimal degrees
    position_timestamp: float = 0.0
    
    # Rudder (from PGN 127245) - for logging/comparison
    rudder_angle_n2k: float = 0.0  # From bus (degrees, +ve starboard)
    rudder_timestamp: float = 0.0
    
    def get_age_ms(self, field_timestamp: float) -> float:
        """Get age of a specific field in milliseconds."""
        if field_timestamp == 0:
            return float('inf')
        return (time.time() - field_timestamp) * 1000


@dataclass
class N2KConfig:
    """Configuration for NMEA2000 interface."""
    channel: str = "can0"
    bitrate: int = 250000
    max_age_ms: float = 5000.0  # 5 seconds max for 1Hz data


class NMEA2000Interface:
    """
    NMEA2000 bus interface using SocketCAN.
    
    Decodes relevant PGNs and maintains current state.
    Designed for CandleLite USB-CAN adapter appearing as can0.
    """
    
    def __init__(self, config: Optional[N2KConfig] = None):
        self.config = config or N2KConfig()
        self._bus: Optional[can.interface.Bus] = None
        self._data = N2KData()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: Dict[PGN, list[Callable]] = {}
        
        # Fast message reassembly buffers
        self._fast_packets: Dict[tuple, bytearray] = {}
        
        # Statistics
        self._msg_count = 0
        self._pgn_counts: Dict[int, int] = {}
        
    def start(self) -> bool:
        """Start the CAN bus reader."""
        if can is None:
            logger.error("python-can not installed. Run: pip install python-can")
            return False
            
        try:
            self._bus = can.interface.Bus(
                channel=self.config.channel,
                bustype='socketcan',
                bitrate=self.config.bitrate
            )
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            logger.info(f"NMEA2000 started on {self.config.channel}")
            return True
        except Exception as e:
            logger.error(f"Failed to open CAN bus: {e}")
            return False
            
    def stop(self):
        """Stop the CAN bus reader."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._bus:
            self._bus.shutdown()
            self._bus = None
        logger.info("NMEA2000 stopped")
        
    def get_data(self) -> N2KData:
        """Get current aggregated NMEA2000 data (thread-safe)."""
        with self._lock:
            # Return a copy
            return N2KData(
                timestamp=time.time(),
                awa=self._data.awa,
                aws=self._data.aws,
                awa_timestamp=self._data.awa_timestamp,
                stw=self._data.stw,
                stw_timestamp=self._data.stw_timestamp,
                cog=self._data.cog,
                sog=self._data.sog,
                cog_sog_timestamp=self._data.cog_sog_timestamp,
                heading_n2k=self._data.heading_n2k,
                heading_timestamp=self._data.heading_timestamp,
                latitude=self._data.latitude,
                longitude=self._data.longitude,
                position_timestamp=self._data.position_timestamp,
                rudder_angle_n2k=self._data.rudder_angle_n2k,
                rudder_timestamp=self._data.rudder_timestamp
            )
            
    def add_callback(self, pgn: PGN, callback: Callable[[int, bytes], None]):
        """Register a callback for a specific PGN."""
        if pgn not in self._callbacks:
            self._callbacks[pgn] = []
        self._callbacks[pgn].append(callback)
        
    def _read_loop(self):
        """Background thread reading CAN frames."""
        while self._running and self._bus:
            try:
                msg = self._bus.recv(timeout=0.1)
                if msg:
                    self._process_frame(msg)
            except Exception as e:
                logger.warning(f"CAN read error: {e}")
                time.sleep(0.01)
                
    def _process_frame(self, msg: can.Message):
        """Process a received CAN frame."""
        # Extract PGN from CAN ID (J1939/NMEA2000 format)
        # CAN ID format: Priority(3) | Reserved(1) | Data Page(1) | PDU Format(8) | PDU Specific(8) | Source(8)
        can_id = msg.arbitration_id
        
        # Extract PDU format and specific
        pdu_format = (can_id >> 16) & 0xFF
        pdu_specific = (can_id >> 8) & 0xFF
        source = can_id & 0xFF
        
        # Calculate PGN
        if pdu_format < 240:
            # PDU1 format (destination specific)
            pgn = pdu_format << 8
        else:
            # PDU2 format (broadcast)
            pgn = (pdu_format << 8) | pdu_specific
            
        # Add data page bit if set
        if (can_id >> 24) & 0x01:
            pgn |= 0x10000
            
        self._msg_count += 1
        self._pgn_counts[pgn] = self._pgn_counts.get(pgn, 0) + 1
        
        # Decode known PGNs
        self._decode_pgn(pgn, msg.data, source)
        
        # Notify callbacks
        if pgn in self._callbacks:
            for callback in self._callbacks[pgn]:
                try:
                    callback(pgn, msg.data)
                except Exception as e:
                    logger.warning(f"PGN callback error: {e}")
                    
    def _decode_pgn(self, pgn: int, data: bytes, source: int):
        """Decode known PGNs and update state."""
        now = time.time()
        
        with self._lock:
            if pgn == PGN.WIND_DATA:
                # PGN 130306: Wind Data
                # Byte 0: SID
                # Bytes 1-2: Wind speed (0.01 m/s)
                # Bytes 3-4: Wind angle (0.0001 rad)
                # Byte 5: Reference (0=True, 1=Magnetic, 2=Apparent, 3=True boat)
                if len(data) >= 6:
                    speed_raw = struct.unpack('<H', data[1:3])[0]
                    angle_raw = struct.unpack('<H', data[3:5])[0]
                    reference = data[5] & 0x07
                    
                    if reference == 2:  # Apparent wind
                        if speed_raw != 0xFFFF:
                            self._data.aws = speed_raw * 0.01 * 1.94384  # m/s to knots
                        if angle_raw != 0xFFFF:
                            angle_rad = angle_raw * 0.0001
                            self._data.awa = angle_rad * 180 / 3.14159
                            # Normalize to -180 to +180
                            if self._data.awa > 180:
                                self._data.awa -= 360
                        self._data.awa_timestamp = now
                        
            elif pgn == PGN.SPEED:
                # PGN 128259: Speed
                # Byte 0: SID
                # Bytes 1-2: Speed water referenced (0.01 m/s)
                # Bytes 3-4: Speed ground referenced (0.01 m/s)
                # Byte 5: Speed water type
                if len(data) >= 4:
                    stw_raw = struct.unpack('<H', data[1:3])[0]
                    if stw_raw != 0xFFFF:
                        self._data.stw = stw_raw * 0.01 * 1.94384  # m/s to knots
                        self._data.stw_timestamp = now
                        
            elif pgn == PGN.COG_SOG:
                # PGN 129026: COG & SOG, Rapid Update
                # Byte 0: SID
                # Byte 1: COG Reference
                # Bytes 2-3: COG (0.0001 rad)
                # Bytes 4-5: SOG (0.01 m/s)
                if len(data) >= 6:
                    cog_raw = struct.unpack('<H', data[2:4])[0]
                    sog_raw = struct.unpack('<H', data[4:6])[0]
                    
                    if cog_raw != 0xFFFF:
                        self._data.cog = cog_raw * 0.0001 * 180 / 3.14159
                    if sog_raw != 0xFFFF:
                        self._data.sog = sog_raw * 0.01 * 1.94384
                    self._data.cog_sog_timestamp = now
                    
            elif pgn == PGN.VESSEL_HEADING:
                # PGN 127250: Vessel Heading
                # Byte 0: SID
                # Bytes 1-2: Heading (0.0001 rad)
                # Bytes 3-4: Deviation (0.0001 rad)
                # Bytes 5-6: Variation (0.0001 rad)
                # Byte 7: Reference
                if len(data) >= 4:
                    heading_raw = struct.unpack('<H', data[1:3])[0]
                    if heading_raw != 0xFFFF:
                        self._data.heading_n2k = heading_raw * 0.0001 * 180 / 3.14159
                        self._data.heading_timestamp = now
                        
            elif pgn == PGN.RUDDER:
                # PGN 127245: Rudder
                # Byte 0: Instance
                # Byte 1: Direction order
                # Bytes 2-3: Angle order (0.0001 rad)
                # Bytes 4-5: Position (0.0001 rad)
                if len(data) >= 6:
                    position_raw = struct.unpack('<h', data[4:6])[0]  # Signed
                    if position_raw != 0x7FFF:
                        self._data.rudder_angle_n2k = position_raw * 0.0001 * 180 / 3.14159
                        self._data.rudder_timestamp = now
                        
    @property
    def stats(self) -> dict:
        """Get statistics about NMEA2000 communication."""
        return {
            "message_count": self._msg_count,
            "pgn_counts": dict(self._pgn_counts),
            "connected": self._bus is not None
        }


def calculate_true_wind(awa: float, aws: float, stw: float, heading: float) -> tuple[float, float]:
    """
    Calculate true wind from apparent wind.
    
    Args:
        awa: Apparent wind angle (degrees, +ve starboard)
        aws: Apparent wind speed (knots)
        stw: Speed through water (knots)
        heading: Boat heading (degrees)
        
    Returns:
        (twa, tws): True wind angle and speed
    """
    import math
    
    # Convert to radians
    awa_rad = math.radians(awa)
    
    # Apparent wind components (boat reference)
    aw_x = aws * math.cos(awa_rad)  # Forward component
    aw_y = aws * math.sin(awa_rad)  # Starboard component
    
    # Remove boat speed to get true wind
    tw_x = aw_x - stw
    tw_y = aw_y
    
    # True wind speed and angle
    tws = math.sqrt(tw_x**2 + tw_y**2)
    twa = math.degrees(math.atan2(tw_y, tw_x))
    
    return twa, tws
