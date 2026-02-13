#!/usr/bin/env python3
"""
BNO055 IMU Web Testing Tool
===========================

Flask server that provides:
- SSE streaming of real-time IMU orientation data
- REST API for calibration and configuration
- Static file serving for the web UI

Usage:
    uv run python vis/imu/server.py --host 0.0.0.0 --port 8080
"""

import argparse
import base64
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from flask import Flask, Response, jsonify, request, send_from_directory

# Add project root to path for imports
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from src.sensors.imu_fusion_bno055 import (
    IMUFusionBNO055, IMUConfigBNO055, IMUCalibrationBNO055, HAS_SMBUS
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__, static_folder='static')

# Global IMU instance
imu: Optional[IMUFusionBNO055] = None
imu_config: Optional[IMUConfigBNO055] = None


def init_imu(bus: int = 1, address: int = 0x28) -> bool:
    """Initialize the IMU with given I2C settings."""
    global imu, imu_config
    
    if not HAS_SMBUS:
        logger.error("smbus2 not installed - IMU not available")
        return False
    
    # Stop existing IMU if running
    if imu is not None:
        imu.stop()
    
    imu_config = IMUConfigBNO055(
        i2c_bus=bus,
        i2c_address=address,
        update_rate_hz=100  # Run at max rate
    )
    
    imu = IMUFusionBNO055(imu_config)
    
    if not imu.start():
        logger.error("Failed to start IMU")
        imu = None
        return False
    
    logger.info(f"IMU started on bus {bus} at address 0x{address:02X}")
    return True


# =============================================================================
# Static file routes
# =============================================================================

@app.route('/')
def index():
    """Serve the main HTML page."""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/static/<path:filename>')
def static_files(filename):
    """Serve static files."""
    return send_from_directory(app.static_folder, filename)


# =============================================================================
# SSE Streaming
# =============================================================================

@app.route('/stream')
def stream():
    """
    Server-Sent Events endpoint for real-time IMU data.
    
    Streams JSON data at ~20Hz with orientation, rates, and calibration status.
    """
    def generate():
        last_send = 0
        interval = 0.05  # 20Hz
        
        while True:
            now = time.time()
            
            if now - last_send >= interval:
                last_send = now
                
                if imu is None:
                    data = {
                        "connected": False,
                        "error": "IMU not initialized"
                    }
                else:
                    imu_data = imu.get_data()
                    cal_status = imu.get_calibration_status()
                    stats = imu.stats
                    
                    data = {
                        "connected": True,
                        "timestamp": imu_data.timestamp,
                        "heading": round(imu_data.heading, 2),
                        "pitch": round(imu_data.pitch, 2),
                        "roll": round(imu_data.roll, 2),
                        "yaw_rate": round(imu_data.yaw_rate, 2),
                        "pitch_rate": round(imu_data.pitch_rate, 2),
                        "roll_rate": round(imu_data.roll_rate, 2),
                        "accel_x": round(imu_data.accel_x, 3),
                        "accel_y": round(imu_data.accel_y, 3),
                        "accel_z": round(imu_data.accel_z, 3),
                        "cal_sys": cal_status["sys"],
                        "cal_gyro": cal_status["gyro"],
                        "cal_accel": cal_status["accel"],
                        "cal_mag": cal_status["mag"],
                        "is_calibrated": cal_status["fully_calibrated"],
                        "valid": imu_data.valid,
                        "age_ms": round(imu_data.age_ms, 1),
                        "message_count": stats["message_count"],
                        "error_count": stats["error_count"]
                    }
                
                yield f"data: {json.dumps(data)}\n\n"
            
            time.sleep(0.01)  # Small sleep to prevent busy-waiting
    
    return Response(
        generate(),
        mimetype='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no'  # Disable nginx buffering
        }
    )


# =============================================================================
# REST API
# =============================================================================

@app.route('/api/status')
def get_status():
    """Get current IMU status and configuration."""
    if imu is None:
        return jsonify({
            "connected": False,
            "error": "IMU not initialized",
            "smbus_available": HAS_SMBUS
        })
    
    cal_status = imu.get_calibration_status()
    stats = imu.stats
    
    return jsonify({
        "connected": True,
        "smbus_available": HAS_SMBUS,
        "config": {
            "i2c_bus": imu_config.i2c_bus,
            "i2c_address": f"0x{imu_config.i2c_address:02X}",
            "update_rate_hz": imu_config.update_rate_hz
        },
        "calibration": cal_status,
        "stats": stats
    })


@app.route('/api/calibrate/status')
def get_calibration_status():
    """Get detailed calibration status."""
    if imu is None:
        return jsonify({"error": "IMU not initialized"}), 503
    
    cal_status = imu.get_calibration_status()
    
    # Add calibration hints based on current status
    hints = []
    if cal_status["gyro"] < 3:
        hints.append("Gyroscope: Keep sensor still for a few seconds")
    if cal_status["accel"] < 3:
        hints.append("Accelerometer: Place sensor in 6 different positions (each axis up/down)")
    if cal_status["mag"] < 3:
        hints.append("Magnetometer: Move sensor in figure-8 pattern")
    if cal_status["sys"] < 3 and cal_status["gyro"] == 3 and cal_status["accel"] == 3 and cal_status["mag"] == 3:
        hints.append("System: Wait for fusion algorithm to stabilize")
    
    return jsonify({
        "status": cal_status,
        "hints": hints,
        "fully_calibrated": cal_status["fully_calibrated"]
    })


@app.route('/api/calibrate/save', methods=['GET'])
def save_calibration():
    """
    Download current calibration as JSON file.
    
    Returns calibration data that can be saved locally and uploaded later.
    """
    if imu is None:
        return jsonify({"error": "IMU not initialized"}), 503
    
    try:
        # Read calibration directly from sensor
        if imu._driver is None:
            return jsonify({"error": "IMU driver not available"}), 503
        
        cal_bytes = imu._driver.get_calibration()
        cal_status = imu.get_calibration_status()
        
        # Create calibration data structure
        cal_data = {
            "calibration_bytes": base64.b64encode(cal_bytes).decode('ascii'),
            "calibration_status": cal_status,
            "saved_at": datetime.now().isoformat(),
            "device": {
                "i2c_bus": imu_config.i2c_bus,
                "i2c_address": f"0x{imu_config.i2c_address:02X}"
            }
        }
        
        # Return as downloadable JSON file
        response = Response(
            json.dumps(cal_data, indent=2),
            mimetype='application/json',
            headers={
                'Content-Disposition': 'attachment; filename=bno055_calibration.json'
            }
        )
        return response
        
    except Exception as e:
        logger.exception("Failed to save calibration")
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/calibrate/load', methods=['POST'])
def load_calibration():
    """
    Upload and apply calibration data.
    
    Accepts JSON calibration data (from save) and applies it to the sensor.
    """
    if imu is None:
        return jsonify({"error": "IMU not initialized"}), 503
    
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({"error": "No calibration data provided"}), 400
        
        if 'calibration_bytes' not in data:
            return jsonify({"error": "Invalid calibration data format"}), 400
        
        # Decode calibration bytes
        cal_bytes = base64.b64decode(data['calibration_bytes'])
        
        if len(cal_bytes) != 22:
            return jsonify({"error": f"Invalid calibration data length: {len(cal_bytes)} (expected 22)"}), 400
        
        # Apply calibration to sensor
        if imu._driver is None:
            return jsonify({"error": "IMU driver not available"}), 503
        
        success = imu._driver.set_calibration(cal_bytes)
        
        if success:
            logger.info("Calibration data applied successfully")
            return jsonify({
                "success": True, 
                "message": "Calibration applied successfully",
                "saved_at": data.get("saved_at", "unknown")
            })
        else:
            return jsonify({"success": False, "error": "Failed to apply calibration to sensor"}), 500
            
    except json.JSONDecodeError:
        return jsonify({"error": "Invalid JSON data"}), 400
    except Exception as e:
        logger.exception("Failed to load calibration")
        return jsonify({"success": False, "error": "Failed to load calibration data"}), 500


@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration."""
    if imu_config is None:
        return jsonify({
            "i2c_bus": 1,
            "i2c_address": "0x28",
            "update_rate_hz": 100
        })
    
    return jsonify({
        "i2c_bus": imu_config.i2c_bus,
        "i2c_address": f"0x{imu_config.i2c_address:02X}",
        "update_rate_hz": imu_config.update_rate_hz
    })


@app.route('/api/config', methods=['POST'])
def set_config():
    """Update configuration and restart IMU."""
    data = request.get_json()
    
    bus = data.get('i2c_bus', 1)
    address = data.get('i2c_address', 0x28)
    
    # Handle hex string or int
    if isinstance(address, str):
        address = int(address, 0)
    
    try:
        success = init_imu(bus, address)
        if success:
            return jsonify({"success": True, "message": f"IMU configured on bus {bus} at 0x{address:02X}"})
        else:
            return jsonify({"success": False, "error": "Failed to initialize IMU"}), 500
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/restart', methods=['POST'])
def restart_imu():
    """Restart the IMU."""
    if imu_config is None:
        return jsonify({"error": "No configuration available"}), 400
    
    try:
        success = init_imu(imu_config.i2c_bus, imu_config.i2c_address)
        if success:
            return jsonify({"success": True, "message": "IMU restarted"})
        else:
            return jsonify({"success": False, "error": "Failed to restart IMU"}), 500
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='BNO055 IMU Web Testing Tool')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Host to bind to (default: 0.0.0.0)')
    parser.add_argument('--port', '-p', type=int, default=8080,
                        help='Port to run server on (default: 8080)')
    parser.add_argument('--bus', '-b', type=int, default=1,
                        help='I2C bus number (default: 1)')
    parser.add_argument('--address', '-a', type=lambda x: int(x, 0), default=0x28,
                        help='I2C address (default: 0x28)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode')
    parser.add_argument('--no-imu', action='store_true',
                        help='Start without IMU (for UI testing)')
    
    args = parser.parse_args()
    
    if not args.no_imu:
        if not HAS_SMBUS:
            logger.warning("smbus2 not available - running in demo mode")
        else:
            if not init_imu(args.bus, args.address):
                logger.warning("Failed to initialize IMU - running without sensor")
    
    logger.info(f"Starting server at http://{args.host}:{args.port}")
    logger.info(f"Open in browser: http://localhost:{args.port}" if args.host == '0.0.0.0' else f"Open in browser: http://{args.host}:{args.port}")
    
    try:
        app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)
    finally:
        if imu is not None:
            imu.stop()
            logger.info("IMU stopped")


if __name__ == '__main__':
    main()
