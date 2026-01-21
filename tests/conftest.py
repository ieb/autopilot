"""
Shared test fixtures for autopilot unit tests.
"""

import time
import pytest
import numpy as np

# Import data classes for fixtures
from src.sensors.imu_fusion import IMUData, IMUConfig
from src.sensors.nmea2000_interface import N2KData, N2KConfig
from src.sensors.adc_reader import RudderData, RudderConfig
from src.ml.feature_engineering import FeatureConfig, FeatureEngineering, TargetState
from src.ml.polar import Polar
from src.control.safety import SafetyConfig, SafetyLayer
from src.control.actuator_interface import ActuatorStatus, ActuatorConfig, MCUFaultCode


@pytest.fixture
def sample_imu_data():
    """Sample IMU data for testing."""
    return IMUData(
        timestamp=time.time(),
        heading=180.0,
        pitch=5.0,
        roll=-10.0,
        yaw_rate=2.0,
        pitch_rate=0.5,
        roll_rate=1.0,
        accel_x=0.1,
        accel_y=-0.2,
        accel_z=9.8,
        valid=True
    )


@pytest.fixture
def sample_n2k_data():
    """Sample NMEA2000 data for testing."""
    now = time.time()
    return N2KData(
        timestamp=now,
        awa=45.0,
        aws=12.0,
        awa_timestamp=now,
        stw=6.5,
        stw_timestamp=now,
        cog=180.0,
        sog=7.0,
        cog_sog_timestamp=now,
        heading_n2k=179.0,
        heading_timestamp=now,
        latitude=51.5,
        longitude=-1.5,
        position_timestamp=now,
        rudder_angle_n2k=2.0,
        rudder_timestamp=now
    )


@pytest.fixture
def sample_rudder_data():
    """Sample rudder position data for testing."""
    return RudderData(
        timestamp=time.time(),
        angle_deg=5.0,
        raw_adc=17000,
        valid=True
    )


@pytest.fixture
def feature_config():
    """Default feature engineering configuration."""
    return FeatureConfig()


@pytest.fixture
def feature_engineering(feature_config):
    """Feature engineering instance for testing."""
    return FeatureEngineering(config=feature_config)


@pytest.fixture
def polar():
    """Pogo 1250 polar diagram for testing."""
    return Polar.pogo_1250()


@pytest.fixture
def safety_layer():
    """Safety layer instance for testing."""
    return SafetyLayer()


@pytest.fixture
def safety_config():
    """Default safety configuration."""
    return SafetyConfig()


@pytest.fixture
def valid_actuator_status():
    """Valid actuator status for testing."""
    return ActuatorStatus(
        timestamp=time.time(),
        target_angle=0.0,
        actual_angle=0.0,
        clutch_engaged=True,
        voltage=12.6,
        current=0.5,
        fault_code=MCUFaultCode.NONE,
        valid=True
    )


@pytest.fixture
def stale_actuator_status():
    """Stale actuator status for testing (old timestamp)."""
    return ActuatorStatus(
        timestamp=time.time() - 1.0,  # 1 second old
        target_angle=0.0,
        actual_angle=0.0,
        clutch_engaged=True,
        voltage=12.6,
        current=0.5,
        fault_code=MCUFaultCode.NONE,
        valid=True
    )


@pytest.fixture
def faulted_actuator_status():
    """Actuator status with MCU fault."""
    return ActuatorStatus(
        timestamp=time.time(),
        target_angle=0.0,
        actual_angle=0.0,
        clutch_engaged=False,
        voltage=12.6,
        current=15.0,  # High current
        fault_code=MCUFaultCode.OVERCURRENT,
        valid=True
    )
