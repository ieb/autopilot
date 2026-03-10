// NMEA2000 simulation driver — in-memory CAN queues instead of real hardware.
// Subclasses tNMEA2000 and implements the 3 required virtual methods.

#ifndef NMEA2000_SIM_H
#define NMEA2000_SIM_H

#include <NMEA2000.h>
#include <N2kMessages.h>
#include <queue>
#include <mutex>

struct SimCANFrame {
    unsigned long id;
    unsigned char len;
    unsigned char buf[8];
};

class tNMEA2000_sim : public tNMEA2000 {
public:
    tNMEA2000_sim() : tNMEA2000() {}

    // Inject a raw CAN frame into the RX queue (called by SimEnvironment)
    void inject_frame(const SimCANFrame& frame) {
        std::lock_guard<std::mutex> lock(rx_mutex);
        rx_queue.push(frame);
    }

    // Inject a complete N2K message by encoding it to CAN frames.
    // For single-frame messages, this serializes and injects one frame.
    void inject_message(const tN2kMsg& msg);

    // Get last transmitted frame (for inspection/logging)
    bool get_tx_frame(SimCANFrame& frame) {
        std::lock_guard<std::mutex> lock(tx_mutex);
        if (tx_queue.empty()) return false;
        frame = tx_queue.front();
        tx_queue.pop();
        return true;
    }

protected:
    bool CANOpen() override { return true; }

    bool CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) override {
        std::lock_guard<std::mutex> lock(rx_mutex);
        if (rx_queue.empty()) return false;
        SimCANFrame frame = rx_queue.front();
        rx_queue.pop();
        id = frame.id;
        len = frame.len;
        memcpy(buf, frame.buf, frame.len);
        return true;
    }

    bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent = true) override {
        (void)wait_sent;
        SimCANFrame frame;
        frame.id = id;
        frame.len = len;
        memcpy(frame.buf, buf, len);
        std::lock_guard<std::mutex> lock(tx_mutex);
        tx_queue.push(frame);
        return true;
    }

private:
    std::queue<SimCANFrame> rx_queue;
    std::queue<SimCANFrame> tx_queue;
    std::mutex rx_mutex;
    std::mutex tx_mutex;
};

// Global instance — referenced by n2k.cpp under HAL_SIM
extern tNMEA2000_sim NMEA2000;

#endif // NMEA2000_SIM_H
