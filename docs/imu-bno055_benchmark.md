# Benchmark script output

* OS: PiOS 32 bit
* Hardware: Pi 1 B+


        ============================================================
        BNO055 PERFORMANCE BENCHMARK
        ============================================================

        I2C Bus: 1
        Address: 0x28
        Duration: 10.0s

        ============================================================
        RAW I2C BENCHMARK
        ============================================================

        Single byte read (1000 iterations):
        Average: 1017.2 µs
        Std Dev: 241.7 µs
        Min:     790.0 µs
        Max:     2657.0 µs
        Max Rate: 983 reads/sec

        Block read (6 bytes, 1000 iterations):
        Average: 1750.5 µs
        Std Dev: 463.1 µs
        Max Rate: 571 reads/sec

        Full sensor read (45 bytes, 1000 iterations):
        Average: 7560.2 µs
        Std Dev: 1034.1 µs
        Max Rate: 132 reads/sec

        ============================================================
        IMU DRIVER BENCHMARK
        ============================================================

        Running for 10 seconds...
        Collecting samples...

        Results:
        Duration:    10.0s
        Samples:     5436
        Sample Rate: 543.2 Hz

        Inter-sample timing:
        Average: 1.84 ms
        Std Dev: 0.41 ms
        Min:     1.43 ms
        Max:     8.02 ms
        Jitter:  ±3.29 ms

        Data latency (timestamp to read):
        Average: 1954329224.49 ms
        P99:     12.78 ms
        Max:     1770622271587.88 ms

        Driver statistics:
        Messages: 873
        Errors: 0

        ============================================================
        ORIENTATION STABILITY BENCHMARK
        ============================================================

        Keep the sensor COMPLETELY STILL during this test.
        Press Enter to start...

        Collecting data for 10 seconds...

        Stability Results (926 samples):

        Heading (°):
            Mean:    +185.312
            Std Dev:    0.000
            Range:      0.000 (+185.312 to +185.312)

        Pitch (°):
            Mean:    -148.438
            Std Dev:    0.000
            Range:      0.000 (-148.438 to -148.438)

        Roll (°):
            Mean:      -1.401
            Std Dev:    0.455
            Range:      8.000 (-9.375 to -1.375)

        Yaw Rate (°/s):
            Mean:      +0.002
            Std Dev:    0.538
            Range:     16.125 (-8.062 to +8.062)

        Assessment:
        ✓ Excellent stability (heading std < 0.1°)

        ============================================================
        BENCHMARK COMPLETE
        ============================================================
        ieb@pilot1:~/usbdisk/autopilot $ 
