// Stub — printSensorDetails() not used in firmware.
// The real implementation uses Serial which requires Arduino.h.

#include "Adafruit_Sensor.h"
#include <stdio.h>

void Adafruit_Sensor::printSensorDetails(void) {
    // No-op in native build
}
