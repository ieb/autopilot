#!/usr/bin/env python3
"""
BNO055 Detection Script
=======================

Simple script to detect and verify the BNO055 IMU is connected via I2C.

Usage:
    python scripts/bno055_detect.py
    
    # With alternate address
    python scripts/bno055_detect.py --address 0x29
"""

import argparse
import sys

try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False
    print("ERROR: smbus2 not installed. Run: pip install smbus2")
    sys.exit(1)


# BNO055 register definitions
BNO055_CHIP_ID_REG = 0x00
BNO055_ACC_ID_REG = 0x01
BNO055_MAG_ID_REG = 0x02
BNO055_GYR_ID_REG = 0x03
BNO055_SW_REV_LSB = 0x04
BNO055_SW_REV_MSB = 0x05
BNO055_CALIB_STAT = 0x35
BNO055_ST_RESULT = 0x36
BNO055_SYS_STATUS = 0x39
BNO055_SYS_ERROR = 0x3A

# Expected chip IDs
EXPECTED_CHIP_ID = 0xA0
EXPECTED_ACC_ID = 0xFB
EXPECTED_MAG_ID = 0x32
EXPECTED_GYR_ID = 0x0F


def detect_bno055(bus_num: int = 1, address: int = 0x28) -> bool:
    """
    Detect BNO055 on I2C bus.
    
    Args:
        bus_num: I2C bus number (default 1)
        address: I2C address (default 0x28)
        
    Returns:
        True if BNO055 found and verified
    """
    print(f"Detecting BNO055 on I2C bus {bus_num} at address 0x{address:02X}...")
    print()
    
    try:
        bus = smbus2.SMBus(bus_num)
    except FileNotFoundError:
        print(f"ERROR: I2C bus {bus_num} not found.")
        print("Make sure I2C is enabled: sudo raspi-config -> Interface Options -> I2C")
        return False
    except PermissionError:
        print(f"ERROR: Permission denied for I2C bus {bus_num}.")
        print("Run with sudo or add user to i2c group: sudo usermod -aG i2c $USER")
        return False
    
    try:
        # Read chip ID
        chip_id = bus.read_byte_data(address, BNO055_CHIP_ID_REG)
        acc_id = bus.read_byte_data(address, BNO055_ACC_ID_REG)
        mag_id = bus.read_byte_data(address, BNO055_MAG_ID_REG)
        gyr_id = bus.read_byte_data(address, BNO055_GYR_ID_REG)
        
        # Read software revision
        sw_lsb = bus.read_byte_data(address, BNO055_SW_REV_LSB)
        sw_msb = bus.read_byte_data(address, BNO055_SW_REV_MSB)
        sw_rev = (sw_msb << 8) | sw_lsb
        
        # Read system status
        sys_status = bus.read_byte_data(address, BNO055_SYS_STATUS)
        sys_error = bus.read_byte_data(address, BNO055_SYS_ERROR)
        st_result = bus.read_byte_data(address, BNO055_ST_RESULT)
        
        # Read calibration status
        calib_stat = bus.read_byte_data(address, BNO055_CALIB_STAT)
        cal_sys = (calib_stat >> 6) & 0x03
        cal_gyr = (calib_stat >> 4) & 0x03
        cal_acc = (calib_stat >> 2) & 0x03
        cal_mag = calib_stat & 0x03
        
        bus.close()
        
        # Display results
        print("=" * 50)
        print("BNO055 DETECTED")
        print("=" * 50)
        print()
        
        print("Chip IDs:")
        status_chip = "✓" if chip_id == EXPECTED_CHIP_ID else "✗"
        status_acc = "✓" if acc_id == EXPECTED_ACC_ID else "✗"
        status_mag = "✓" if mag_id == EXPECTED_MAG_ID else "✗"
        status_gyr = "✓" if gyr_id == EXPECTED_GYR_ID else "✗"
        
        print(f"  {status_chip} Chip ID:    0x{chip_id:02X} (expected 0x{EXPECTED_CHIP_ID:02X})")
        print(f"  {status_acc} Accel ID:   0x{acc_id:02X} (expected 0x{EXPECTED_ACC_ID:02X})")
        print(f"  {status_mag} Mag ID:     0x{mag_id:02X} (expected 0x{EXPECTED_MAG_ID:02X})")
        print(f"  {status_gyr} Gyro ID:    0x{gyr_id:02X} (expected 0x{EXPECTED_GYR_ID:02X})")
        print(f"  Software Rev: {sw_rev}")
        print()
        
        print("System Status:")
        status_names = {
            0: "Idle",
            1: "System Error",
            2: "Initializing Peripherals",
            3: "System Initialization",
            4: "Executing Self-Test",
            5: "Sensor Fusion Running",
            6: "System Running (no fusion)"
        }
        print(f"  Status: {status_names.get(sys_status, 'Unknown')} ({sys_status})")
        print(f"  Error Code: {sys_error}")
        print(f"  Self-Test: {'PASS' if st_result == 0x0F else 'FAIL'} (0x{st_result:02X})")
        print()
        
        print("Calibration Status (0-3, 3=calibrated):")
        print(f"  System:        {cal_sys}/3 {'✓' if cal_sys == 3 else ''}")
        print(f"  Gyroscope:     {cal_gyr}/3 {'✓' if cal_gyr == 3 else ''}")
        print(f"  Accelerometer: {cal_acc}/3 {'✓' if cal_acc == 3 else ''}")
        print(f"  Magnetometer:  {cal_mag}/3 {'✓' if cal_mag == 3 else ''}")
        print()
        
        # Verify all IDs match
        if (chip_id == EXPECTED_CHIP_ID and acc_id == EXPECTED_ACC_ID and
            mag_id == EXPECTED_MAG_ID and gyr_id == EXPECTED_GYR_ID):
            print("✓ All chip IDs verified - BNO055 is working correctly!")
            return True
        else:
            print("✗ Some chip IDs don't match - sensor may be faulty")
            return False
            
    except OSError as e:
        print(f"ERROR: I2C communication failed: {e}")
        print()
        print("Troubleshooting:")
        print("1. Check wiring connections")
        print("2. Verify I2C is enabled: sudo i2cdetect -y 1")
        print("3. Try alternate address: --address 0x29")
        return False


def scan_i2c(bus_num: int = 1):
    """Scan I2C bus for devices."""
    print(f"Scanning I2C bus {bus_num}...")
    print()
    
    try:
        bus = smbus2.SMBus(bus_num)
    except Exception as e:
        print(f"ERROR: Cannot open I2C bus: {e}")
        return
    
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    
    bus.close()
    
    if found:
        print(f"Found {len(found)} device(s):")
        for addr in found:
            note = ""
            if addr == 0x28:
                note = " <- BNO055 (default address)"
            elif addr == 0x29:
                note = " <- BNO055 (alternate address)"
            print(f"  0x{addr:02X}{note}")
    else:
        print("No I2C devices found!")
        print()
        print("Check:")
        print("1. I2C is enabled: sudo raspi-config")
        print("2. Wiring is correct (SDA, SCL, VCC, GND)")
        print("3. Device is powered")


def main():
    parser = argparse.ArgumentParser(description="Detect BNO055 IMU on I2C")
    parser.add_argument("--bus", "-b", type=int, default=1,
                       help="I2C bus number (default: 1)")
    parser.add_argument("--address", "-a", type=lambda x: int(x, 0), default=0x28,
                       help="I2C address (default: 0x28)")
    parser.add_argument("--scan", "-s", action="store_true",
                       help="Scan I2C bus for all devices")
    
    args = parser.parse_args()
    
    if args.scan:
        scan_i2c(args.bus)
        print()
    
    success = detect_bno055(args.bus, args.address)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
