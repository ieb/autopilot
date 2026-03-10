// Global TwoWire instance for HAL-level simulation.
// Must be defined exactly once.

#ifdef HAL_SIM

#include "Wire.h"

TwoWire Wire;

#endif // HAL_SIM
