#ifdef HAL_SIM

#include "NMEA2000_sim.h"

// Global instance
tNMEA2000_sim NMEA2000;

void tNMEA2000_sim::inject_message(const tN2kMsg& msg) {
    // The NMEA2000 library's SendMsg internally encodes messages to CAN frames
    // via CANSendFrame. To inject messages for reception, we need to encode
    // them the same way the library does for single-frame PGNs.
    //
    // For single-frame messages (<=8 data bytes), the CAN frame format is:
    //   ID bits: priority(3) | reserved(1) | data_page(1) | PGN(16) | source(8)
    //   For PDU2 (PGN >= 0xF000): PGN includes destination in PS field
    //
    // We use the library's own encoding by temporarily sending via the TX path
    // and capturing the result, then moving it to the RX queue.

    // Simple approach: directly construct CAN frames for single-frame messages
    if (msg.DataLen <= 8) {
        SimCANFrame frame;
        // Build 29-bit CAN ID for NMEA2000
        // Format: priority(3) | reserved(1)=0 | data_page(1) | PGN_high(8) | PGN_low/dest(8) | source(8)
        unsigned long pgn = msg.PGN;
        unsigned char priority = msg.Priority;
        unsigned char source = msg.Source;

        unsigned long id;
        if (pgn < 0xF000) {
            // PDU1: destination-specific
            id = ((unsigned long)priority << 26) |
                 ((pgn & 0x1FF00) << 8) |
                 ((unsigned long)msg.Destination << 8) |
                 source;
        } else {
            // PDU2: broadcast
            id = ((unsigned long)priority << 26) |
                 (pgn << 8) |
                 source;
        }

        frame.id = id;
        frame.len = msg.DataLen;
        memcpy(frame.buf, msg.Data, msg.DataLen);
        inject_frame(frame);
    }
    // Multi-frame (fast-packet / ISO TP) not needed for our PGNs
}

#endif // HAL_SIM
