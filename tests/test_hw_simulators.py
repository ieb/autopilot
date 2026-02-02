"""
Tests for Hardware Simulators
=============================

Tests for IMU, Actuator, and CAN simulators including:
- Base class functionality
- Protocol message generation and parsing
- Socket communication
- Shared state handling
- Stub mode for macOS compatibility
"""

import pytest
import time
import socket
import os
import threading
import struct
from multiprocessing import Manager
from unittest.mock import MagicMock, patch

from src.simulation.hw_simulators.base import (
    HardwareSimulator,
    SimulatorConfig,
)
from src.simulation.hw_simulators.imu_sim import (
    IMUSimulator,
    IMUSimulatorConfig,
)
from src.simulation.hw_simulators.actuator_sim import (
    ActuatorSimulator,
    ActuatorSimulatorConfig,
    FaultCode,
)
from src.simulation.hw_simulators.can_sim import (
    CANSimulator,
    CANSimulatorConfig,
    PGN,
)


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def manager():
    """Create a multiprocessing Manager for shared state."""
    mgr = Manager()
    yield mgr
    mgr.shutdown()


@pytest.fixture
def shared_state(manager):
    """Create a shared state dict with default values."""
    state = manager.dict()
    state['heading'] = 45.0
    state['pitch'] = 2.0
    state['roll'] = 5.0
    state['yaw_rate'] = 1.5
    state['pitch_rate'] = 0.3
    state['roll_rate'] = 0.8
    state['accel_x'] = 0.1
    state['accel_y'] = 0.2
    state['accel_z'] = 9.81
    state['stw'] = 6.5
    state['sog'] = 6.8
    state['cog'] = 48.0
    state['awa'] = 35.0
    state['aws'] = 18.0
    state['latitude'] = 51.5
    state['longitude'] = -1.2
    state['rudder_angle'] = 0.0
    return state


@pytest.fixture
def temp_socket_path(tmp_path):
    """Create a temporary socket path.
    
    Note: macOS has a 104-byte limit for AF_UNIX socket paths.
    pytest's tmp_path can be very long, so we use /tmp directly.
    """
    import uuid
    short_name = f"ap_test_{uuid.uuid4().hex[:8]}.sock"
    path = f"/tmp/{short_name}"
    yield path
    # Cleanup
    if os.path.exists(path):
        os.unlink(path)


# =============================================================================
# Base Simulator Tests
# =============================================================================

class TestBaseSimulator:
    """Tests for HardwareSimulator base class."""
    
    def test_compute_checksum(self):
        """Test NMEA-style checksum computation."""
        # Known checksum values
        assert HardwareSimulator.compute_checksum("IMU,45.0,2.0,5.0") == "65"
        assert HardwareSimulator.compute_checksum("RUD,0.500,1") == "59"
        assert HardwareSimulator.compute_checksum("STS,0.000,0.000,0,12.4,0.0,0") == "63"
        
    def test_verify_checksum_valid(self):
        """Test checksum verification with valid message."""
        assert HardwareSimulator.verify_checksum("$IMU,45.0,2.0,5.0*65")
        assert HardwareSimulator.verify_checksum("$RUD,0.500,1*59")
        
    def test_verify_checksum_invalid(self):
        """Test checksum verification with invalid message."""
        assert not HardwareSimulator.verify_checksum("$IMU,45.0,2.0,5.0*00")  # Wrong checksum
        assert not HardwareSimulator.verify_checksum("IMU,45.0,2.0,5.0*65")   # Missing $
        assert not HardwareSimulator.verify_checksum("$IMU,45.0,2.0,5.0")     # Missing *XX
        
    def test_simulator_config_update_interval(self):
        """Test update interval calculation."""
        config = SimulatorConfig(socket_path="/tmp/test.sock", update_rate_hz=100.0)
        assert config.update_interval == pytest.approx(0.01, rel=1e-3)
        
        config = SimulatorConfig(socket_path="/tmp/test.sock", update_rate_hz=20.0)
        assert config.update_interval == pytest.approx(0.05, rel=1e-3)


# =============================================================================
# IMU Simulator Tests
# =============================================================================

class TestIMUSimulator:
    """Tests for IMU simulator."""
    
    def test_config_defaults(self):
        """Test default configuration values."""
        config = IMUSimulatorConfig()
        assert config.socket_path == "/tmp/autopilot_imu.sock"
        assert config.update_rate_hz == 100.0
        assert config.heading_noise_std > 0
        
    def test_simulator_creation(self, shared_state):
        """Test simulator can be created."""
        sim = IMUSimulator(shared_state)
        assert sim is not None
        assert sim.running is False
        
    def test_simulator_start_stop(self, shared_state, temp_socket_path):
        """Test simulator can start and stop."""
        config = IMUSimulatorConfig(socket_path=temp_socket_path)
        sim = IMUSimulator(shared_state, config)
        
        assert sim.start()
        assert sim.running
        assert os.path.exists(temp_socket_path)
        
        sim.stop()
        assert not sim.running
        assert not os.path.exists(temp_socket_path)
        
    def test_socket_accepts_connection(self, shared_state, temp_socket_path):
        """Test that clients can connect and receive startup sequence."""
        config = IMUSimulatorConfig(socket_path=temp_socket_path)
        sim = IMUSimulator(shared_state, config)
        sim.start()
        
        try:
            # Connect as client
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Should receive startup sequence ($VER, $RDY)
            time.sleep(0.1)
            data = client.recv(1024)
            assert len(data) > 0
            assert b'$VER,' in data
            assert b'$RDY' in data
            
            client.close()
        finally:
            sim.stop()
            
    def test_imu_message_format(self, shared_state, temp_socket_path):
        """Test IMU message format is correct after starting streaming."""
        config = IMUSimulatorConfig(
            socket_path=temp_socket_path,
            heading_noise_std=0,  # Disable noise for predictable output
            attitude_noise_std=0,
            rate_noise_std=0,
            accel_noise_std=0,
            timing_jitter_std=0
        )
        sim = IMUSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Wait for startup sequence
            time.sleep(0.1)
            client.recv(1024)  # Consume $VER, $RDY
            
            # Send START command to begin streaming
            start_cs = HardwareSimulator.compute_checksum("START")
            cmd = f"$START*{start_cs}\r\n"
            client.send(cmd.encode('ascii'))
            
            time.sleep(0.1)
            data = client.recv(1024).decode('ascii')
            lines = [l.strip() for l in data.split('\n') if l.strip()]
            
            # Find first $IMU line (skip $ACK)
            imu_lines = [l for l in lines if l.startswith('$IMU,')]
            assert len(imu_lines) > 0
            line = imu_lines[0]
            
            # Verify format: $IMU,heading,pitch,roll,yaw_rate,pitch_rate,roll_rate,ax,ay,az*XX
            assert line.startswith('$IMU,')
            assert '*' in line
            
            # Verify checksum
            assert HardwareSimulator.verify_checksum(line)
            
            # Parse values
            payload = line[1:line.index('*')]
            parts = payload.split(',')
            assert len(parts) == 10  # IMU + 9 values
            
            heading = float(parts[1])
            assert 0 <= heading < 360
            
            client.close()
        finally:
            sim.stop()
            
    def test_imu_reads_shared_state(self, shared_state, temp_socket_path):
        """Test IMU reads values from shared state after streaming starts."""
        shared_state['heading'] = 123.4
        shared_state['pitch'] = 5.6
        shared_state['roll'] = 7.8
        
        config = IMUSimulatorConfig(
            socket_path=temp_socket_path,
            heading_noise_std=0,
            attitude_noise_std=0,
            rate_noise_std=0,
            accel_noise_std=0,
            timing_jitter_std=0
        )
        sim = IMUSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Wait for and consume startup sequence
            time.sleep(0.1)
            client.recv(1024)
            
            # Send START command
            start_cs = HardwareSimulator.compute_checksum("START")
            cmd = f"$START*{start_cs}\r\n"
            client.send(cmd.encode('ascii'))
            
            time.sleep(0.1)
            data = client.recv(1024).decode('ascii')
            
            # Find $IMU line
            imu_lines = [l.strip() for l in data.split('\n') if l.strip().startswith('$IMU,')]
            assert len(imu_lines) > 0
            line = imu_lines[0]
            
            payload = line[1:line.index('*')]
            parts = payload.split(',')
            
            heading = float(parts[1])
            pitch = float(parts[2])
            roll = float(parts[3])
            
            assert heading == pytest.approx(123.4, abs=0.1)
            assert pitch == pytest.approx(5.6, abs=0.1)
            assert roll == pytest.approx(7.8, abs=0.1)
            
            client.close()
        finally:
            sim.stop()


# =============================================================================
# Actuator Simulator Tests
# =============================================================================

class TestActuatorSimulator:
    """Tests for Actuator simulator."""
    
    def test_config_defaults(self):
        """Test default configuration values."""
        config = ActuatorSimulatorConfig()
        assert config.socket_path == "/tmp/autopilot_actuator.sock"
        assert config.update_rate_hz == 20.0
        assert config.max_rudder_rate == 4.0
        assert config.watchdog_timeout == 2.0
        
    def test_simulator_creation(self, shared_state):
        """Test simulator can be created."""
        sim = ActuatorSimulator(shared_state)
        assert sim is not None
        assert not sim._clutch_engaged
        
    def test_simulator_start_stop(self, shared_state, temp_socket_path):
        """Test simulator can start and stop."""
        config = ActuatorSimulatorConfig(socket_path=temp_socket_path)
        sim = ActuatorSimulator(shared_state, config)
        
        assert sim.start()
        assert sim.running
        
        sim.stop()
        assert not sim.running
        
    def test_status_message_format(self, shared_state, temp_socket_path):
        """Test status message format."""
        config = ActuatorSimulatorConfig(socket_path=temp_socket_path)
        sim = ActuatorSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            time.sleep(0.1)
            data = client.recv(1024).decode('ascii')
            lines = [l.strip() for l in data.split('\n') if l.strip()]
            
            assert len(lines) > 0
            line = lines[0]
            
            # Format: $STS,target,actual,clutch,voltage,current,fault*XX
            assert line.startswith('$STS,')
            assert HardwareSimulator.verify_checksum(line)
            
            payload = line[1:line.index('*')]
            parts = payload.split(',')
            assert len(parts) == 7  # STS + 6 values
            
            client.close()
        finally:
            sim.stop()
            
    def test_rudder_command_parsing(self, shared_state, temp_socket_path):
        """Test rudder command is parsed correctly."""
        config = ActuatorSimulatorConfig(socket_path=temp_socket_path)
        sim = ActuatorSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Send engage command
            checksum = HardwareSimulator.compute_checksum("RUD,0.500,1")
            cmd = f"$RUD,0.500,1*{checksum}\r\n"
            client.send(cmd.encode('ascii'))
            
            # Wait for processing
            time.sleep(0.2)
            
            # Should be engaging
            assert sim._clutch_engaging or sim._clutch_engaged
            assert sim._target_angle == pytest.approx(0.5, abs=0.01)
            
            client.close()
        finally:
            sim.stop()
            
    def test_clutch_disengage(self, shared_state, temp_socket_path):
        """Test clutch disengages on command."""
        config = ActuatorSimulatorConfig(socket_path=temp_socket_path)
        sim = ActuatorSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Engage
            engage_cs = HardwareSimulator.compute_checksum("RUD,0.000,1")
            cmd = f"$RUD,0.000,1*{engage_cs}\r\n"
            client.send(cmd.encode('ascii'))
            time.sleep(0.2)
            
            # Disengage
            disengage_cs = HardwareSimulator.compute_checksum("RUD,0.000,0")
            cmd = f"$RUD,0.000,0*{disengage_cs}\r\n"
            client.send(cmd.encode('ascii'))
            time.sleep(0.1)
            
            assert not sim._clutch_engaged
            assert not sim._clutch_engaging
            
            client.close()
        finally:
            sim.stop()
            
    def test_watchdog_timeout(self, shared_state, temp_socket_path):
        """Test clutch disengages on watchdog timeout."""
        config = ActuatorSimulatorConfig(
            socket_path=temp_socket_path,
            watchdog_timeout=0.3  # Short timeout for test
        )
        sim = ActuatorSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Engage
            checksum = HardwareSimulator.compute_checksum("RUD,0.000,1")
            cmd = f"$RUD,0.000,1*{checksum}\r\n"
            client.send(cmd.encode('ascii'))
            time.sleep(0.2)
            
            assert sim._clutch_engaged or sim._clutch_engaging
            
            # Wait for watchdog
            time.sleep(0.5)
            
            # Should have disengaged
            assert not sim._clutch_engaged
            assert sim._fault_code == FaultCode.WATCHDOG
            
            client.close()
        finally:
            sim.stop()
            
    def test_rudder_rate_limiting(self, shared_state, temp_socket_path):
        """Test rudder movement is rate limited."""
        config = ActuatorSimulatorConfig(
            socket_path=temp_socket_path,
            max_rudder_rate=4.0,
            clutch_engage_delay=0.01  # Fast engage for test
        )
        sim = ActuatorSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Command full deflection
            checksum = HardwareSimulator.compute_checksum("RUD,1.000,1")
            cmd = f"$RUD,1.000,1*{checksum}\r\n"
            client.send(cmd.encode('ascii'))
            
            time.sleep(0.05)  # Let it engage
            
            # After 0.1s, should have moved ~0.4° out of 30°
            # Normalized: 0.4/30 ≈ 0.013
            time.sleep(0.1)
            
            # Keep sending commands to prevent watchdog
            client.send(cmd.encode('ascii'))
            
            # Should not have reached target yet (would need ~7.5s)
            assert sim._actual_angle < sim._target_angle
            
            client.close()
        finally:
            sim.stop()
            
    def test_updates_shared_state(self, shared_state, temp_socket_path):
        """Test actuator writes rudder angle to shared state."""
        config = ActuatorSimulatorConfig(
            socket_path=temp_socket_path,
            clutch_engage_delay=0.01
        )
        sim = ActuatorSimulator(shared_state, config)
        sim.start()
        
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.settimeout(2.0)
            client.connect(temp_socket_path)
            
            # Command some rudder
            checksum = HardwareSimulator.compute_checksum("RUD,0.500,1")
            cmd = f"$RUD,0.500,1*{checksum}\r\n"
            client.send(cmd.encode('ascii'))
            
            time.sleep(0.3)
            
            # Shared state should be updated
            rudder_angle = shared_state.get('rudder_angle', 0.0)
            assert rudder_angle != 0.0  # Should have moved some
            
            client.close()
        finally:
            sim.stop()


# =============================================================================
# CAN Simulator Tests
# =============================================================================

class TestCANSimulator:
    """Tests for CAN simulator."""
    
    def test_config_defaults(self):
        """Test default configuration."""
        config = CANSimulatorConfig()
        assert config.channel == "vcan0"
        assert config.update_rate_hz == 1.0
        assert config.source_address == 42
        
    def test_stub_mode_on_macos(self, shared_state):
        """Test stub mode is auto-enabled on non-Linux."""
        config = CANSimulatorConfig()
        sim = CANSimulator(shared_state, config)
        
        started = sim.start()
        assert started
        # On macOS, should be in stub mode
        # On Linux without vcan, should fallback to stub mode
        # Either way, should have started
        
        sim.stop()
        
    def test_explicit_stub_mode(self, shared_state):
        """Test explicit stub mode works."""
        config = CANSimulatorConfig(stub_mode=True)
        sim = CANSimulator(shared_state, config)
        
        assert sim.start()
        assert sim._stub_mode
        
        sim.stop()
        
    def test_frame_generation(self, shared_state):
        """Test PGN frames are generated."""
        config = CANSimulatorConfig(stub_mode=True)
        sim = CANSimulator(shared_state, config)
        sim.start()
        
        # Generate frames
        sim.step()
        
        frames = sim.get_frame_buffer()
        assert len(frames) == 5  # Wind, Speed, COG/SOG, Heading, Position
        
        # Check PGN types
        pgns = [pgn for pgn, _ in frames]
        assert PGN.WIND_DATA in pgns
        assert PGN.SPEED in pgns
        assert PGN.COG_SOG in pgns
        assert PGN.VESSEL_HEADING in pgns
        assert PGN.GNSS_POSITION in pgns
        
        sim.stop()
        
    def test_wind_data_encoding(self, shared_state):
        """Test wind data PGN is encoded correctly."""
        shared_state['awa'] = 45.0  # degrees
        shared_state['aws'] = 15.0  # knots
        
        config = CANSimulatorConfig(stub_mode=True)
        sim = CANSimulator(shared_state, config)
        sim.start()
        sim.step()
        
        frames = sim.get_frame_buffer()
        wind_frame = next((d for pgn, d in frames if pgn == PGN.WIND_DATA), None)
        assert wind_frame is not None
        
        # Parse the frame
        # Byte 0: SID
        # Bytes 1-2: Speed (0.01 m/s)
        # Bytes 3-4: Angle (0.0001 rad)
        # Byte 5: Reference
        sid = wind_frame[0]
        speed_raw = struct.unpack('<H', wind_frame[1:3])[0]
        angle_raw = struct.unpack('<H', wind_frame[3:5])[0]
        reference = wind_frame[5]
        
        # Verify reference is Apparent (2)
        assert reference == 2
        
        # Verify speed (15 kts ≈ 7.72 m/s → 772 raw)
        expected_speed_ms = 15.0 / 1.94384
        expected_speed_raw = int(expected_speed_ms / 0.01)
        assert speed_raw == pytest.approx(expected_speed_raw, abs=1)
        
        # Verify angle (45° ≈ 0.785 rad → 7854 raw)
        import math
        expected_angle_rad = 45.0 * math.pi / 180.0
        expected_angle_raw = int(expected_angle_rad / 0.0001)
        assert angle_raw == pytest.approx(expected_angle_raw, abs=10)
        
        sim.stop()
        
    def test_heading_encoding(self, shared_state):
        """Test vessel heading PGN is encoded correctly."""
        shared_state['heading'] = 90.0  # degrees
        
        config = CANSimulatorConfig(stub_mode=True)
        sim = CANSimulator(shared_state, config)
        sim.start()
        sim.step()
        
        frames = sim.get_frame_buffer()
        heading_frame = next((d for pgn, d in frames if pgn == PGN.VESSEL_HEADING), None)
        assert heading_frame is not None
        
        # Parse: Byte 0: SID, Bytes 1-2: Heading (0.0001 rad)
        heading_raw = struct.unpack('<H', heading_frame[1:3])[0]
        
        import math
        expected_heading_rad = 90.0 * math.pi / 180.0
        expected_heading_raw = int(expected_heading_rad / 0.0001)
        assert heading_raw == pytest.approx(expected_heading_raw, abs=10)
        
        sim.stop()
        
    def test_frame_buffer_limit(self, shared_state):
        """Test frame buffer doesn't grow unbounded."""
        config = CANSimulatorConfig(stub_mode=True)
        sim = CANSimulator(shared_state, config)
        sim.start()
        
        # Generate many frames
        for _ in range(50):
            sim.step()
            
        frames = sim.get_frame_buffer()
        assert len(frames) <= 100  # Buffer limit
        
        sim.stop()
        
    def test_clear_frame_buffer(self, shared_state):
        """Test frame buffer can be cleared."""
        config = CANSimulatorConfig(stub_mode=True)
        sim = CANSimulator(shared_state, config)
        sim.start()
        sim.step()
        
        assert len(sim.get_frame_buffer()) > 0
        
        sim.clear_frame_buffer()
        assert len(sim.get_frame_buffer()) == 0
        
        sim.stop()
        
    def test_can_id_generation(self, shared_state):
        """Test CAN arbitration ID generation."""
        config = CANSimulatorConfig(stub_mode=True, source_address=42)
        sim = CANSimulator(shared_state, config)
        
        # Test PDU2 format (broadcast, pdu_format >= 240)
        # PGN 130306 = 0x1FD02
        can_id = sim._make_can_id(PGN.WIND_DATA, priority=6)
        
        # Extract components
        source = can_id & 0xFF
        pdu_specific = (can_id >> 8) & 0xFF
        pdu_format = (can_id >> 16) & 0xFF
        
        assert source == 42
        # PGN 130306 = 0x1FD02 → PDU format = 0xFD, PDU specific = 0x02
        assert pdu_format == 0xFD
        assert pdu_specific == 0x02


# =============================================================================
# Integration Tests
# =============================================================================

class TestSimulatorIntegration:
    """Integration tests for simulator communication."""
    
    def test_imu_interface_socket_connection(self, shared_state, temp_socket_path):
        """Test IMUFusion can connect to IMU simulator via socket."""
        from src.sensors.imu_fusion import IMUFusion, IMUConfig
        
        # Start simulator
        sim_config = IMUSimulatorConfig(
            socket_path=temp_socket_path,
            heading_noise_std=0,
            attitude_noise_std=0,
            rate_noise_std=0,
            accel_noise_std=0,
            timing_jitter_std=0
        )
        sim = IMUSimulator(shared_state, sim_config)
        sim.start()
        
        try:
            # Connect via IMUFusion
            imu_config = IMUConfig(
                port=temp_socket_path,
                use_socket=True,
                timeout=1.0
            )
            imu = IMUFusion(imu_config)
            assert imu.start()
            
            # Wait for data
            time.sleep(0.2)
            
            data = imu.get_data()
            assert data.heading == pytest.approx(shared_state['heading'], abs=1.0)
            
            imu.stop()
        finally:
            sim.stop()
            
    def test_actuator_interface_socket_connection(self, shared_state, temp_socket_path):
        """Test ActuatorInterface can connect to Actuator simulator via socket."""
        from src.control.actuator_interface import ActuatorInterface, ActuatorConfig
        
        # Start simulator
        sim_config = ActuatorSimulatorConfig(
            socket_path=temp_socket_path,
            clutch_engage_delay=0.01
        )
        sim = ActuatorSimulator(shared_state, sim_config)
        sim.start()
        
        try:
            # Connect via ActuatorInterface
            act_config = ActuatorConfig(
                port=temp_socket_path,
                use_socket=True,
                timeout=1.0
            )
            actuator = ActuatorInterface(act_config)
            assert actuator.start()
            
            # Wait for connection
            time.sleep(0.1)
            
            # Send command
            actuator.send_command(0.5, engage=True)
            
            # Wait for response
            time.sleep(0.3)
            
            # Get status
            status = actuator.get_status()
            assert status.target_angle == pytest.approx(0.5, abs=0.01)
            
            actuator.stop()
        finally:
            sim.stop()


# =============================================================================
# Orchestrator Tests  
# =============================================================================

class TestOrchestratorUnit:
    """Unit tests for orchestrator components (without full process launch)."""
    
    def test_orchestrator_config_defaults(self):
        """Test orchestrator config defaults."""
        from src.simulation.hw_simulators.orchestrator import OrchestratorConfig
        
        config = OrchestratorConfig()
        assert config.physics_rate_hz == 50.0
        assert config.imu_config.update_rate_hz == 100.0
        assert config.actuator_config.update_rate_hz == 20.0
        
    def test_apparent_wind_calculation(self, shared_state):
        """Test apparent wind computation."""
        from src.simulation.hw_simulators.orchestrator import SimulatorOrchestrator
        
        orch = SimulatorOrchestrator()
        
        # Boat heading north at 6 kts, wind from east (90°) at 10 kts
        awa, aws = orch._compute_apparent_wind(
            boat_speed=6.0,
            heading=0.0,
            tws=10.0,
            twd=90.0
        )
        
        # TWA = 90° - 0° = 90° (beam wind from starboard)
        # Apparent wind should be faster than true (more forward)
        import math
        expected_aws = math.sqrt(6**2 + 10**2)  # ≈ 11.66 kts
        assert aws == pytest.approx(expected_aws, rel=0.01)
        
        # AWA should be less than 90° (wind comes more from ahead)
        assert awa < 90.0
        assert awa > 0
        
    def test_twa_calculation(self, shared_state):
        """Test true wind angle calculation."""
        from src.simulation.hw_simulators.orchestrator import SimulatorOrchestrator
        
        orch = SimulatorOrchestrator()
        
        # Heading 45°, TWD 90° → TWA = 45°
        twa = orch._compute_twa(heading=45.0, twd=90.0)
        assert twa == pytest.approx(45.0, abs=0.1)
        
        # Heading 180°, TWD 90° → TWA = -90°
        twa = orch._compute_twa(heading=180.0, twd=90.0)
        assert twa == pytest.approx(-90.0, abs=0.1)
        
        # Heading 0°, TWD 0° → TWA = 0° (dead upwind)
        twa = orch._compute_twa(heading=0.0, twd=0.0)
        assert twa == pytest.approx(0.0, abs=0.1)
