#!/usr/bin/env python3
"""
BNO055 Read Script
==================

Continuously read and display orientation data from the BNO055 IMU.

Usage:
    python scripts/bno055_read.py
    
    # Custom update rate
    python scripts/bno055_read.py --rate 50
    
    # Show raw sensor data
    python scripts/bno055_read.py --raw
"""

import argparse
import sys
import time
import signal

# Add parent directory to path for imports
sys.path.insert(0, '.')

try:
    from src.sensors.imu_fusion_bno055 import (
        IMUFusionBNO055, IMUConfigBNO055, HAS_SMBUS
    )
except ImportError as e:
    print(f"ERROR: Cannot import IMU module: {e}")
    print("Run from project root: python scripts/bno055_read.py")
    sys.exit(1)

if not HAS_SMBUS:
    print("ERROR: smbus2 not installed. Run: pip install smbus2")
    sys.exit(1)


# Global flag for clean shutdown
running = True
start_time = 0
sample_count = 0


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    global running
    running = False
    print("\nShutting down...")


def format_angle(angle: float) -> str:
    """Format angle with sign and padding."""
    return f"{angle:+7.2f}°"


def format_cal(cal: int) -> str:
    """Format calibration status with visual indicator."""
    bars = "█" * cal + "░" * (3 - cal)
    return f"{bars} {cal}/3"


def main():
    parser = argparse.ArgumentParser(description="Read BNO055 orientation data")
    parser.add_argument("--bus", "-b", type=int, default=1,
                       help="I2C bus number (default: 1)")
    parser.add_argument("--address", "-a", type=lambda x: int(x, 0), default=0x28,
                       help="I2C address (default: 0x28)")
    parser.add_argument("--rate", "-r", type=int, default=10,
                       help="Display update rate in Hz (default: 10)")
    parser.add_argument("--raw", action="store_true",
                       help="Show raw sensor data (accel, gyro, mag)")
    parser.add_argument("--csv", action="store_true",
                       help="Output as CSV format")
    
    args = parser.parse_args()
    
    # Setup signal handler for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create IMU config
    config = IMUConfigBNO055(
        i2c_bus=args.bus,
        i2c_address=args.address,
        update_rate_hz=100  # Run sensor at max rate
    )
    
    print(f"Connecting to BNO055 on I2C bus {args.bus} at 0x{args.address:02X}...")
    
    imu = IMUFusionBNO055(config)
    
    if not imu.start():
        print("ERROR: Failed to start IMU")
        sys.exit(1)
    
    print("BNO055 started successfully!")
    print()
    
    # Wait for first data
    time.sleep(0.1)
    
    # Print header
    if args.csv:
        if args.raw:
            print("timestamp,heading,pitch,roll,yaw_rate,ax,ay,az,gx,gy,gz,mx,my,mz,cal_sys,cal_gyr,cal_acc,cal_mag")
        else:
            print("timestamp,heading,pitch,roll,yaw_rate,cal_sys,cal_gyr,cal_acc,cal_mag")
    else:
        print("Press Ctrl+C to stop")
        print()
        print("=" * 70)
    
    global start_time, sample_count
    
    update_interval = 1.0 / args.rate
    last_update = 0
    start_time = time.time()
    sample_count = 0
    
    while running:
        now = time.time()
        
        if now - last_update >= update_interval:
            last_update = now
            
            data = imu.get_data()
            
            if data is None:
                print("Waiting for data...")
                continue
            
            sample_count += 1
            
            if args.csv:
                # CSV output
                if args.raw:
                    print(f"{data.timestamp:.3f},{data.heading:.2f},{data.pitch:.2f},"
                          f"{data.roll:.2f},{data.yaw_rate:.2f},"
                          f"{data.accel_x:.3f},{data.accel_y:.3f},{data.accel_z:.3f},"
                          f"{data.gyro_x:.3f},{data.gyro_y:.3f},{data.gyro_z:.3f},"
                          f"{data.mag_x:.1f},{data.mag_y:.1f},{data.mag_z:.1f},"
                          f"{data.cal_sys},{data.cal_gyro},{data.cal_accel},{data.cal_mag}")
                else:
                    print(f"{data.timestamp:.3f},{data.heading:.2f},{data.pitch:.2f},"
                          f"{data.roll:.2f},{data.yaw_rate:.2f},"
                          f"{data.cal_sys},{data.cal_gyro},{data.cal_accel},{data.cal_mag}")
            else:
                # Human-readable output
                lines = []
                
                # Orientation
                lines.append(f"Heading: {format_angle(data.heading):>10}  "
                            f"Pitch: {format_angle(data.pitch):>10}  "
                            f"Roll: {format_angle(data.roll):>10}")
                lines.append(f"Yaw Rate: {data.yaw_rate:+7.2f}°/s")
                
                if args.raw:
                    # Raw sensor data
                    lines.append("")
                    lines.append(f"Accel: X={data.accel_x:+7.3f}  Y={data.accel_y:+7.3f}  "
                                f"Z={data.accel_z:+7.3f} m/s²")
                    lines.append(f"Gyro:  X={data.gyro_x:+7.3f}  Y={data.gyro_y:+7.3f}  "
                                f"Z={data.gyro_z:+7.3f} °/s")
                    lines.append(f"Mag:   X={data.mag_x:+7.1f}  Y={data.mag_y:+7.1f}  "
                                f"Z={data.mag_z:+7.1f} µT")
                
                # Calibration status
                lines.append("")
                cal_status = "CALIBRATED ✓" if data.is_calibrated else "Calibrating..."
                lines.append(f"Calibration: {cal_status}")
                lines.append(f"  Sys: {format_cal(data.cal_sys)}  "
                            f"Gyr: {format_cal(data.cal_gyro)}  "
                            f"Acc: {format_cal(data.cal_accel)}  "
                            f"Mag: {format_cal(data.cal_mag)}")
                
                # Data quality
                age_ms = (time.time() - data.timestamp) * 1000
                quality = "✓ Fresh" if age_ms < 50 else f"⚠ Stale ({age_ms:.0f}ms)"
                runtime = time.time() - start_time
                rate = sample_count / runtime if runtime > 0 else 0
                lines.append("")
                lines.append(f"Data: {quality}  |  Rate: {rate:.1f} Hz")
                
                # Clear screen and print (move cursor up)
                num_lines = len(lines) + 2
                print(f"\033[{num_lines}A", end="")  # Move up
                print("=" * 70)
                for line in lines:
                    print(f"{line:<70}")
                print("=" * 70)
        
        time.sleep(0.001)  # Small sleep to prevent busy-waiting
    
    # Cleanup
    imu.stop()
    print("\nIMU stopped.")
    
    # Print stats
    runtime = time.time() - start_time
    avg_rate = sample_count / runtime if runtime > 0 else 0
    stats = imu.stats
    print(f"\nSession Statistics:")
    print(f"  Runtime: {runtime:.1f}s")
    print(f"  Samples: {sample_count}")
    print(f"  Avg Rate: {avg_rate:.1f} Hz")
    print(f"  Messages: {stats['message_count']}")
    print(f"  Errors: {stats['error_count']}")


if __name__ == "__main__":
    main()
