#!/usr/bin/env python3
"""
BNO055 Benchmark Script
=======================

Measure I2C performance and data rates from the BNO055 IMU.

Usage:
    python scripts/bno055_benchmark.py
    
    # Run for specific duration
    python scripts/bno055_benchmark.py --duration 30
"""

import argparse
import sys
import time
import statistics

# Add parent directory to path for imports
sys.path.insert(0, '.')

try:
    from src.sensors.imu_fusion_bno055 import (
        IMUFusionBNO055, IMUConfigBNO055, HAS_SMBUS
    )
except ImportError as e:
    print(f"ERROR: Cannot import IMU module: {e}")
    print("Run from project root: python scripts/bno055_benchmark.py")
    sys.exit(1)

if not HAS_SMBUS:
    print("ERROR: smbus2 not installed. Run: pip install smbus2")
    sys.exit(1)


def benchmark_raw_i2c(bus_num: int, address: int, iterations: int = 1000):
    """Benchmark raw I2C read performance."""
    import smbus2
    
    print(f"\n{'='*60}")
    print("RAW I2C BENCHMARK")
    print(f"{'='*60}")
    
    bus = smbus2.SMBus(bus_num)
    
    # Benchmark single byte read
    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        bus.read_byte_data(address, 0x00)  # Read chip ID
        times.append(time.perf_counter() - start)
    
    avg_us = statistics.mean(times) * 1e6
    std_us = statistics.stdev(times) * 1e6 if len(times) > 1 else 0
    min_us = min(times) * 1e6
    max_us = max(times) * 1e6
    
    print(f"\nSingle byte read ({iterations} iterations):")
    print(f"  Average: {avg_us:.1f} µs")
    print(f"  Std Dev: {std_us:.1f} µs")
    print(f"  Min:     {min_us:.1f} µs")
    print(f"  Max:     {max_us:.1f} µs")
    print(f"  Max Rate: {1e6/avg_us:.0f} reads/sec")
    
    # Benchmark block read (Euler angles - 6 bytes)
    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        bus.read_i2c_block_data(address, 0x1A, 6)  # Euler angles
        times.append(time.perf_counter() - start)
    
    avg_us = statistics.mean(times) * 1e6
    std_us = statistics.stdev(times) * 1e6 if len(times) > 1 else 0
    
    print(f"\nBlock read (6 bytes, {iterations} iterations):")
    print(f"  Average: {avg_us:.1f} µs")
    print(f"  Std Dev: {std_us:.1f} µs")
    print(f"  Max Rate: {1e6/avg_us:.0f} reads/sec")
    
    # Benchmark full data read (all sensor data - 45 bytes)
    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        bus.read_i2c_block_data(address, 0x08, 32)  # First block
        bus.read_i2c_block_data(address, 0x28, 13)  # Second block
        times.append(time.perf_counter() - start)
    
    avg_us = statistics.mean(times) * 1e6
    std_us = statistics.stdev(times) * 1e6 if len(times) > 1 else 0
    
    print(f"\nFull sensor read (45 bytes, {iterations} iterations):")
    print(f"  Average: {avg_us:.1f} µs")
    print(f"  Std Dev: {std_us:.1f} µs")
    print(f"  Max Rate: {1e6/avg_us:.0f} reads/sec")
    
    bus.close()


def benchmark_imu_driver(config: IMUConfigBNO055, duration: float = 10.0):
    """Benchmark the IMU driver's data rate."""
    print(f"\n{'='*60}")
    print("IMU DRIVER BENCHMARK")
    print(f"{'='*60}")
    
    imu = IMUFusionBNO055(config)
    
    if not imu.start():
        print("ERROR: Failed to start IMU")
        return
    
    print(f"\nRunning for {duration:.0f} seconds...")
    print("Collecting samples...")
    
    # Collect timing data
    sample_times = []
    read_latencies = []
    last_sample_time = None
    
    start_time = time.time()
    sample_count = 0
    
    while time.time() - start_time < duration:
        data = imu.get_data()
        now = time.time()
        
        if data is not None:
            sample_count += 1
            
            # Track inter-sample time
            if last_sample_time is not None:
                sample_times.append(now - last_sample_time)
            last_sample_time = now
            
            # Track data latency
            latency = now - data.timestamp
            read_latencies.append(latency)
        
        time.sleep(0.001)  # Small sleep to not spin
    
    imu.stop()
    
    # Calculate statistics
    elapsed = time.time() - start_time
    
    print(f"\nResults:")
    print(f"  Duration:    {elapsed:.1f}s")
    print(f"  Samples:     {sample_count}")
    print(f"  Sample Rate: {sample_count/elapsed:.1f} Hz")
    
    if sample_times:
        avg_interval = statistics.mean(sample_times) * 1000
        std_interval = statistics.stdev(sample_times) * 1000 if len(sample_times) > 1 else 0
        min_interval = min(sample_times) * 1000
        max_interval = max(sample_times) * 1000
        
        print(f"\nInter-sample timing:")
        print(f"  Average: {avg_interval:.2f} ms")
        print(f"  Std Dev: {std_interval:.2f} ms")
        print(f"  Min:     {min_interval:.2f} ms")
        print(f"  Max:     {max_interval:.2f} ms")
        print(f"  Jitter:  ±{(max_interval-min_interval)/2:.2f} ms")
    
    if read_latencies:
        avg_latency = statistics.mean(read_latencies) * 1000
        max_latency = max(read_latencies) * 1000
        p99_latency = sorted(read_latencies)[int(len(read_latencies)*0.99)] * 1000
        
        print(f"\nData latency (timestamp to read):")
        print(f"  Average: {avg_latency:.2f} ms")
        print(f"  P99:     {p99_latency:.2f} ms")
        print(f"  Max:     {max_latency:.2f} ms")
    
    # Driver stats
    stats = imu.stats
    print(f"\nDriver statistics:")
    print(f"  Messages: {stats['message_count']}")
    print(f"  Errors: {stats['error_count']}")


def benchmark_orientation_stability(config: IMUConfigBNO055, duration: float = 10.0):
    """Benchmark orientation output stability when stationary."""
    print(f"\n{'='*60}")
    print("ORIENTATION STABILITY BENCHMARK")
    print(f"{'='*60}")
    print("\nKeep the sensor COMPLETELY STILL during this test.")
    print("Press Enter to start...")
    input()
    
    imu = IMUFusionBNO055(config)
    
    if not imu.start():
        print("ERROR: Failed to start IMU")
        return
    
    # Wait for initial data
    time.sleep(0.5)
    
    headings = []
    pitches = []
    rolls = []
    yaw_rates = []
    
    print(f"Collecting data for {duration:.0f} seconds...")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        data = imu.get_data()
        if data is not None:
            headings.append(data.heading)
            pitches.append(data.pitch)
            rolls.append(data.roll)
            yaw_rates.append(data.yaw_rate)
        time.sleep(0.01)
    
    imu.stop()
    
    print(f"\nStability Results ({len(headings)} samples):")
    
    def analyze(name, values):
        if not values:
            return
        mean = statistics.mean(values)
        std = statistics.stdev(values) if len(values) > 1 else 0
        min_v = min(values)
        max_v = max(values)
        range_v = max_v - min_v
        
        print(f"\n  {name}:")
        print(f"    Mean:    {mean:+8.3f}")
        print(f"    Std Dev: {std:8.3f}")
        print(f"    Range:   {range_v:8.3f} ({min_v:+.3f} to {max_v:+.3f})")
    
    analyze("Heading (°)", headings)
    analyze("Pitch (°)", pitches)
    analyze("Roll (°)", rolls)
    analyze("Yaw Rate (°/s)", yaw_rates)
    
    # Overall assessment
    heading_std = statistics.stdev(headings) if len(headings) > 1 else 0
    print(f"\nAssessment:")
    if heading_std < 0.1:
        print("  ✓ Excellent stability (heading std < 0.1°)")
    elif heading_std < 0.5:
        print("  ✓ Good stability (heading std < 0.5°)")
    elif heading_std < 1.0:
        print("  ⚠ Moderate stability (heading std < 1.0°)")
    else:
        print("  ✗ Poor stability (heading std >= 1.0°)")
        print("    → Check calibration and magnetic interference")


def main():
    parser = argparse.ArgumentParser(description="Benchmark BNO055 IMU performance")
    parser.add_argument("--bus", "-b", type=int, default=1,
                       help="I2C bus number (default: 1)")
    parser.add_argument("--address", "-a", type=lambda x: int(x, 0), default=0x28,
                       help="I2C address (default: 0x28)")
    parser.add_argument("--duration", "-d", type=float, default=10.0,
                       help="Benchmark duration in seconds (default: 10)")
    parser.add_argument("--skip-raw", action="store_true",
                       help="Skip raw I2C benchmark")
    parser.add_argument("--skip-driver", action="store_true",
                       help="Skip driver benchmark")
    parser.add_argument("--skip-stability", action="store_true",
                       help="Skip stability benchmark")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("BNO055 PERFORMANCE BENCHMARK")
    print("=" * 60)
    print(f"\nI2C Bus: {args.bus}")
    print(f"Address: 0x{args.address:02X}")
    print(f"Duration: {args.duration}s")
    
    config = IMUConfigBNO055(
        i2c_bus=args.bus,
        i2c_address=args.address
    )
    
    # Run benchmarks
    if not args.skip_raw:
        benchmark_raw_i2c(args.bus, args.address)
    
    if not args.skip_driver:
        benchmark_imu_driver(config, args.duration)
    
    if not args.skip_stability:
        benchmark_orientation_stability(config, args.duration)
    
    print(f"\n{'='*60}")
    print("BENCHMARK COMPLETE")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
