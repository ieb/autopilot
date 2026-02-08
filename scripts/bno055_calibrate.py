#!/usr/bin/env python3
"""
BNO055 Calibration Script
=========================

Interactive calibration tool for the BNO055 IMU.

Usage:
    python scripts/bno055_calibrate.py
    
    # Save calibration to file
    python scripts/bno055_calibrate.py --save /etc/autopilot/bno055_cal.json
    
    # Set timeout
    python scripts/bno055_calibrate.py --timeout 120
"""

import argparse
import sys
import time
import signal

# Add parent directory to path for imports
sys.path.insert(0, '.')

try:
    from src.sensors.imu_fusion_bno055 import (
        IMUFusionBNO055, IMUConfigBNO055, IMUCalibrationBNO055, HAS_SMBUS
    )
except ImportError as e:
    print(f"ERROR: Cannot import IMU module: {e}")
    print("Run from project root: python scripts/bno055_calibrate.py")
    sys.exit(1)

if not HAS_SMBUS:
    print("ERROR: smbus2 not installed. Run: pip install smbus2")
    sys.exit(1)


# Global flag for clean shutdown
running = True


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    global running
    running = False
    print("\n\nCalibration cancelled.")


def format_bar(value: int, max_val: int = 3, width: int = 20) -> str:
    """Create a progress bar for calibration status."""
    filled = int((value / max_val) * width)
    bar = "█" * filled + "░" * (width - filled)
    return f"[{bar}] {value}/{max_val}"


def print_instructions():
    """Print calibration instructions."""
    print()
    print("=" * 60)
    print("BNO055 CALIBRATION PROCEDURE")
    print("=" * 60)
    print()
    print("The BNO055 calibrates automatically. Follow these steps:")
    print()
    print("1. GYROSCOPE (fastest)")
    print("   → Keep the sensor completely still for 2-3 seconds")
    print()
    print("2. ACCELEROMETER")
    print("   → Place sensor in 6 orientations:")
    print("     - Flat on table (both sides)")
    print("     - On each edge (4 positions)")
    print("   → Hold still in each position for 2-3 seconds")
    print()
    print("3. MAGNETOMETER")
    print("   → Wave sensor in a figure-8 pattern")
    print("   → Cover all orientations (like drawing a 3D 8)")
    print("   → Keep away from metal and magnets")
    print()
    print("4. SYSTEM")
    print("   → Calibrates automatically when others are done")
    print()
    print("=" * 60)
    print()


def main():
    parser = argparse.ArgumentParser(description="Calibrate BNO055 IMU")
    parser.add_argument("--bus", "-b", type=int, default=1,
                       help="I2C bus number (default: 1)")
    parser.add_argument("--address", "-a", type=lambda x: int(x, 0), default=0x28,
                       help="I2C address (default: 0x28)")
    parser.add_argument("--timeout", "-t", type=int, default=60,
                       help="Calibration timeout in seconds (default: 60)")
    parser.add_argument("--save", "-s", type=str, default=None,
                       help="Save calibration to file when complete")
    parser.add_argument("--quiet", "-q", action="store_true",
                       help="Minimal output")
    
    args = parser.parse_args()
    
    # Setup signal handler for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    
    if not args.quiet:
        print_instructions()
    
    # Create IMU config
    config = IMUConfigBNO055(
        i2c_bus=args.bus,
        i2c_address=args.address,
        calibration_file=args.save
    )
    
    print(f"Connecting to BNO055 on I2C bus {args.bus} at 0x{args.address:02X}...")
    
    imu = IMUFusionBNO055(config)
    
    if not imu.start():
        print("ERROR: Failed to start IMU")
        sys.exit(1)
    
    print("BNO055 connected!")
    print()
    
    # Initial calibration status
    status = imu.get_calibration_status()
    if status['fully_calibrated']:
        print("✓ Sensor is already fully calibrated!")
        if args.save:
            print(f"\nSaving calibration to {args.save}...")
            imu.save_calibration()
            print("Calibration saved!")
        imu.stop()
        return
    
    print("Starting calibration monitoring...")
    print("Press Ctrl+C to cancel")
    print()
    
    start_time = time.time()
    last_status = None
    
    # Progress tracking
    best_sys = 0
    best_gyr = 0
    best_acc = 0
    best_mag = 0
    
    while running:
        elapsed = time.time() - start_time
        
        if elapsed > args.timeout:
            print(f"\n⚠ Calibration timed out after {args.timeout}s")
            print("Try again with more motion/orientations.")
            break
        
        status = imu.get_calibration_status()
        
        # Track best values
        best_sys = max(best_sys, status['sys'])
        best_gyr = max(best_gyr, status['gyro'])
        best_acc = max(best_acc, status['accel'])
        best_mag = max(best_mag, status['mag'])
        
        # Only update display if status changed or every second
        if (status != last_status or int(elapsed) != int(elapsed - 0.1)):
            last_status = status
            
            # Status icons
            icon_sys = "✓" if status['sys'] == 3 else " "
            icon_gyr = "✓" if status['gyro'] == 3 else " "
            icon_acc = "✓" if status['accel'] == 3 else " "
            icon_mag = "✓" if status['mag'] == 3 else " "
            
            # Build status display
            lines = [
                f"Time: {elapsed:5.1f}s / {args.timeout}s",
                "",
                f" {icon_gyr} Gyroscope:     {format_bar(status['gyro'])}",
                f" {icon_acc} Accelerometer: {format_bar(status['accel'])}",
                f" {icon_mag} Magnetometer:  {format_bar(status['mag'])}",
                f" {icon_sys} System:        {format_bar(status['sys'])}",
                "",
            ]
            
            # Add hints based on what's not calibrated
            hints = []
            if status['gyro'] < 3:
                hints.append("Keep sensor still...")
            if status['accel'] < 3:
                hints.append("Try different orientations...")
            if status['mag'] < 3:
                hints.append("Wave in figure-8 pattern...")
            
            if hints:
                lines.append(f"→ {hints[0]}")
            else:
                lines.append("All sensors calibrating well!")
            
            # Move cursor up and redraw
            num_lines = len(lines) + 1
            print(f"\033[{num_lines}A", end="")
            for line in lines:
                print(f"{line:<60}")
            print()
        
        # Check if fully calibrated
        if status['fully_calibrated']:
            print()
            print("=" * 60)
            print("✓ CALIBRATION COMPLETE!")
            print("=" * 60)
            print(f"\nCalibration achieved in {elapsed:.1f} seconds")
            
            if args.save:
                print(f"\nSaving calibration to {args.save}...")
                try:
                    imu.save_calibration()
                    print("✓ Calibration saved successfully!")
                except Exception as e:
                    print(f"✗ Failed to save: {e}")
            else:
                print("\nUse --save <path> to save calibration for next boot.")
            
            break
        
        time.sleep(0.1)
    
    imu.stop()
    
    if not status.get('fully_calibrated', False):
        print()
        print("Final calibration status:")
        print(f"  System:        {best_sys}/3")
        print(f"  Gyroscope:     {best_gyr}/3")
        print(f"  Accelerometer: {best_acc}/3")
        print(f"  Magnetometer:  {best_mag}/3")


if __name__ == "__main__":
    main()
