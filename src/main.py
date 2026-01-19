"""
Main Autopilot Application
==========================

Main entry point that coordinates all autopilot subsystems.
"""

import time
import signal
import sys
import argparse
import logging
from dataclasses import dataclass
from typing import Optional
from pathlib import Path

# Local imports
from .sensors.imu_fusion import IMUFusion, IMUConfig
from .sensors.nmea2000_interface import NMEA2000Interface, N2KConfig
from .sensors.adc_reader import ADCReader, RudderConfig, MockADCReader
from .ml.feature_engineering import FeatureEngineering
from .ml.autopilot_model import AutopilotInference, MockAutopilotInference
from .ml.polar import Polar
from .control.rudder_controller import RudderController, MockRudderController
from .control.mode_manager import ModeManager, AutopilotMode
from .control.safety import SafetyLayer, EmergencyStop, AlarmCode

logger = logging.getLogger(__name__)


@dataclass
class AutopilotConfig:
    """Main autopilot configuration."""
    # Hardware interfaces
    imu_port: str = "/dev/ttyUSB0"
    imu_baudrate: int = 115200
    can_channel: str = "can0"
    
    # Model
    model_path: str = "models/autopilot.tflite"
    
    # Control loop timing
    ml_rate_hz: float = 10.0       # ML inference rate
    rudder_rate_hz: float = 50.0   # Inner control loop rate
    
    # Modes
    simulation: bool = False       # Use mock hardware
    
    # Logging
    log_dir: str = "logs"
    log_data: bool = True


class Autopilot:
    """
    Main autopilot controller.
    
    Coordinates:
    - Sensor data acquisition (IMU, NMEA2000, rudder ADC)
    - Feature engineering
    - ML model inference
    - Safety validation
    - Rudder control
    - Mode management
    """
    
    def __init__(self, config: Optional[AutopilotConfig] = None):
        self.config = config or AutopilotConfig()
        
        # Subsystems (initialized in start())
        self._imu: Optional[IMUFusion] = None
        self._n2k: Optional[NMEA2000Interface] = None
        self._adc: Optional[ADCReader] = None
        self._features: Optional[FeatureEngineering] = None
        self._model: Optional[AutopilotInference] = None
        self._rudder: Optional[RudderController] = None
        self._mode_manager: Optional[ModeManager] = None
        self._safety: Optional[SafetyLayer] = None
        self._emergency_stop: Optional[EmergencyStop] = None
        
        # State
        self._running = False
        self._last_ml_time = 0.0
        self._last_rudder_time = 0.0
        self._ml_output = 0.0
        self._commanded_position = 0.0
        
        # Statistics
        self._loop_count = 0
        self._ml_inference_count = 0
        
    def start(self) -> bool:
        """Initialize and start all subsystems."""
        logger.info("Starting autopilot...")
        
        try:
            # Initialize polar
            polar = Polar.pogo_1250()
            
            # Initialize sensors
            if self.config.simulation:
                logger.info("Running in SIMULATION mode")
                self._adc = MockADCReader()
            else:
                self._imu = IMUFusion(IMUConfig(
                    port=self.config.imu_port,
                    baudrate=self.config.imu_baudrate
                ))
                if not self._imu.start():
                    logger.warning("IMU failed to start")
                    
                self._n2k = NMEA2000Interface(N2KConfig(
                    channel=self.config.can_channel
                ))
                if not self._n2k.start():
                    logger.warning("NMEA2000 failed to start")
                    
                self._adc = ADCReader()
                
            if not self._adc.start():
                logger.warning("ADC failed to start")
                
            # Initialize ML
            self._features = FeatureEngineering(polar=polar)
            
            model_path = Path(self.config.model_path)
            if model_path.exists():
                self._model = AutopilotInference(str(model_path))
                logger.info(f"Loaded model from {model_path}")
            else:
                logger.warning(f"Model not found at {model_path}, using mock model")
                self._model = MockAutopilotInference()
                
            # Initialize control
            if self.config.simulation:
                self._rudder = MockRudderController()
            else:
                self._rudder = RudderController()
                
            if not self._rudder.start():
                logger.error("Rudder controller failed to start")
                return False
                
            self._mode_manager = ModeManager(polar=polar)
            self._safety = SafetyLayer()
            self._emergency_stop = EmergencyStop()
            
            self._running = True
            logger.info("Autopilot started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start autopilot: {e}")
            return False
            
    def stop(self):
        """Stop all subsystems."""
        logger.info("Stopping autopilot...")
        self._running = False
        
        # Emergency stop rudder
        if self._rudder:
            self._rudder.emergency_stop()
            self._rudder.stop()
            
        if self._imu:
            self._imu.stop()
        if self._n2k:
            self._n2k.stop()
        if self._adc:
            self._adc.stop()
            
        logger.info("Autopilot stopped")
        
    def run(self):
        """Main control loop."""
        ml_interval = 1.0 / self.config.ml_rate_hz
        rudder_interval = 1.0 / self.config.rudder_rate_hz
        
        logger.info(f"Running at {self.config.ml_rate_hz}Hz ML, "
                   f"{self.config.rudder_rate_hz}Hz rudder")
        
        while self._running:
            try:
                now = time.time()
                
                # Check emergency stop
                if self._emergency_stop.is_triggered:
                    self._rudder.emergency_stop()
                    time.sleep(0.1)
                    continue
                    
                # ML inference at lower rate
                if now - self._last_ml_time >= ml_interval:
                    self._ml_step()
                    self._last_ml_time = now
                    
                # Rudder control at higher rate
                if now - self._last_rudder_time >= rudder_interval:
                    self._rudder_step()
                    self._last_rudder_time = now
                    
                # Short sleep to prevent busy-waiting
                time.sleep(0.001)
                
                self._loop_count += 1
                
            except KeyboardInterrupt:
                logger.info("Interrupted by user")
                break
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                time.sleep(0.1)
                
    def _ml_step(self):
        """Execute ML inference step."""
        # Get sensor data
        from .sensors.imu_fusion import IMUData
        from .sensors.nmea2000_interface import N2KData
        from .sensors.adc_reader import RudderData
        
        if self._imu:
            imu_data = self._imu.get_data()
        else:
            # Mock IMU data for simulation
            imu_data = IMUData(timestamp=time.time(), valid=True)
            
        if self._n2k:
            n2k_data = self._n2k.get_data()
        else:
            n2k_data = N2KData(timestamp=time.time())
            
        rudder_data = self._adc.get_data() if self._adc else RudderData()
        
        # Skip if in standby
        if not self._mode_manager.is_active:
            return
            
        # Update mode manager
        target_heading = self._mode_manager.update(imu_data.heading, n2k_data)
        
        # Update feature engineering target
        mode = self._mode_manager.get_state().mode
        if mode == AutopilotMode.COMPASS:
            self._features.set_target("compass", target_heading)
        elif mode == AutopilotMode.WIND_AWA:
            self._features.set_target("wind_awa", self._mode_manager.get_state().target_value)
        elif mode in [AutopilotMode.WIND_TWA, AutopilotMode.VMG_UP, AutopilotMode.VMG_DOWN]:
            self._features.set_target("wind_twa", self._mode_manager.get_state().target_value)
            
        # Compute features
        sequence = self._features.update(imu_data, n2k_data, rudder_data)
        
        # Run inference
        if self._features.is_valid:
            raw_output = self._model.predict(sequence)
        else:
            raw_output = 0.0
            logger.warning("Invalid feature data, outputting zero")
            
        # Safety validation
        heading_error = self._features.target.target_heading - imu_data.heading
        safe_output, safety_state = self._safety.validate(
            ml_output=raw_output,
            heading_error=heading_error,
            rudder_position=rudder_data.angle_deg,
            imu_age_ms=imu_data.age_ms,
            rudder_age_ms=(time.time() - rudder_data.timestamp) * 1000,
            commanded_position=self._commanded_position * 30.0
        )
        
        if not safety_state.is_safe:
            logger.warning(f"Safety: {safety_state.alarm_message}")
            if safety_state.alarm_code == AlarmCode.MANUAL_OVERRIDE:
                self.set_mode(AutopilotMode.STANDBY)
                
        self._ml_output = safe_output
        self._ml_inference_count += 1
        
    def _rudder_step(self):
        """Execute rudder control step."""
        rudder_data = self._adc.get_data() if self._adc else None
        current_position = rudder_data.angle_deg if rudder_data else 0.0
        
        # Update rudder controller
        if self._mode_manager.is_active:
            self._rudder.update(self._ml_output, current_position)
            self._commanded_position = self._ml_output
        else:
            # In standby, don't drive rudder
            self._rudder.update(0, current_position)
            
    def set_mode(self, mode: AutopilotMode, target: float = 0.0, tack_side: int = 1):
        """Set autopilot mode."""
        if self._mode_manager:
            self._mode_manager.set_mode(mode, target, tack_side)
            
            if mode == AutopilotMode.STANDBY:
                self._safety.reset()
                
    def engage_compass(self, heading: float):
        """Engage compass mode at specified heading."""
        self.set_mode(AutopilotMode.COMPASS, heading)
        
    def engage_wind(self, angle: float, true_wind: bool = False):
        """Engage wind mode."""
        mode = AutopilotMode.WIND_TWA if true_wind else AutopilotMode.WIND_AWA
        tack_side = 1 if angle >= 0 else -1
        self.set_mode(mode, angle, tack_side)
        
    def disengage(self):
        """Disengage autopilot."""
        self.set_mode(AutopilotMode.STANDBY)
        
    def tack(self):
        """Execute a tack."""
        if self._mode_manager:
            self._mode_manager.tack()
            
    def adjust(self, delta: float):
        """Adjust target by delta degrees."""
        if self._mode_manager:
            self._mode_manager.adjust_target(delta)
            
    def emergency_stop(self, reason: str = "Manual"):
        """Trigger emergency stop."""
        if self._emergency_stop:
            self._emergency_stop.trigger(reason)
        if self._rudder:
            self._rudder.emergency_stop()
            
    @property
    def status(self) -> dict:
        """Get current autopilot status."""
        return {
            "running": self._running,
            "mode": self._mode_manager.get_state().mode_name if self._mode_manager else "UNKNOWN",
            "target": self._mode_manager.get_state().target_value if self._mode_manager else 0,
            "ml_output": self._ml_output,
            "loop_count": self._loop_count,
            "ml_inference_count": self._ml_inference_count,
            "emergency_stop": self._emergency_stop.is_triggered if self._emergency_stop else False
        }


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="ML Yacht Autopilot")
    parser.add_argument("--model", "-m", default="models/autopilot.tflite",
                       help="Path to TFLite model")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0",
                       help="IMU serial port")
    parser.add_argument("--can-channel", default="can0",
                       help="CAN bus channel")
    parser.add_argument("--simulation", "-s", action="store_true",
                       help="Run in simulation mode")
    parser.add_argument("--verbose", "-v", action="store_true",
                       help="Verbose logging")
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create config
    config = AutopilotConfig(
        model_path=args.model,
        imu_port=args.imu_port,
        can_channel=args.can_channel,
        simulation=args.simulation
    )
    
    # Create and start autopilot
    autopilot = Autopilot(config)
    
    # Signal handler for graceful shutdown
    def signal_handler(sig, frame):
        logger.info("Shutdown signal received")
        autopilot.stop()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if autopilot.start():
        logger.info("Autopilot running. Press Ctrl+C to stop.")
        autopilot.run()
    else:
        logger.error("Failed to start autopilot")
        sys.exit(1)


if __name__ == "__main__":
    main()
