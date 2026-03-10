// N2K library support for HAL-level native build.
// N2kTimer.cpp already provides N2kMillis64() and N2kMillis() for non-Arduino
// platforms using our extern "C" millis(). No additional support needed here.
// This file is kept as a placeholder for any future N2K-specific support code.

#ifdef HAL_SIM
// Nothing needed — N2kTimer.cpp handles timing via extern "C" millis()
#endif
