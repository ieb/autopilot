# ESP32 Autopilot



# Open Questions

* The ESP32 has a notoriously bad ADC expecially at the top and bottom of the range. This is more a question. Is the ESP32 ADC good enough and linear enough for a) the motor current reading and b) the rudder possition. I expect the rudder pot to be setup to read from perhaps 1V to 2V avoiding the 0-1V and 2-3.3V hence avoiding the worst of the ESP32 non linearities.
* Nothing calls the calibration methods for the rudder. The ui should setup for center, port and starboard rudder readings, and then normalise the ranges to ensure linearality and save the rudder settings.
* n2k.cpp:parse_heading. This should be the primary source of heading. When it is updated updated, the IMU headding reading should be saved so that between N2K Heading updates the change in IMU heading can correct the N2K Headding. This will ensure that teh pilot does require the IMU to be perfectly aligned at all times. It is only needed to be stable and linear for a few seconds between N2K Heading updates. imu.cpp will also have to be changes so that every time there is a new imu reading, the H2K headding is adjusted.
