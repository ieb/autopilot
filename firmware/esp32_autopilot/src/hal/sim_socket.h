// SimSocket — TCP socket server for external simulator communication.
// Allows a Python program (using yacht_dynamics.py) to drive the HAL sim
// by sending CAN frames and IMU data over a framed TCP protocol.
//
// Wire protocol: [type:1][length:2 LE][payload:length]
//   Type 0x01: Python→ESP CAN frame (4B CAN ID LE + 1B DLC + up to 8B data)
//   Type 0x02: ESP→Python CAN frame (same format)
//   Type 0x03: Python→ESP IMU data (6 x int16 LE: heading,roll,pitch,gx,gy,gz)

#ifndef SIM_SOCKET_H
#define SIM_SOCKET_H

#include <atomic>
#include <mutex>
#include <stdint.h>

// Forward declarations
class tNMEA2000_sim;
class TwoWire;

// Snapshot of socket diagnostics, readable from any thread.
// Updated atomically by SimSocket after each received message.
struct SimSocketStats {
    bool connected = false;

    // Total message counts
    uint32_t can_rx_count = 0;
    uint32_t imu_rx_count = 0;
    uint32_t can_tx_count = 0;

    // Per-PGN receive counts
    uint32_t pgn_wind_count = 0;    // PGN 130306
    uint32_t pgn_stw_count = 0;     // PGN 128259
    uint32_t pgn_cog_sog_count = 0; // PGN 129026
    uint32_t pgn_heading_count = 0; // PGN 127250
    uint32_t pgn_other_count = 0;   // unrecognised PGNs

    // Last IMU values (degrees / dps, already scaled from raw)
    float imu_heading = 0;
    float imu_roll = 0;
    float imu_pitch = 0;
    float imu_gyro_x = 0;
    float imu_gyro_y = 0;
    float imu_gyro_z = 0;
};

class SimSocket {
public:
    SimSocket(tNMEA2000_sim& nmea, TwoWire& wire);
    ~SimSocket();

    // Start listening on the given port. Returns true on success.
    bool start(int port = 9876);

    // Drain TX frames from NMEA2000 and send to connected client.
    // Call from the main loop.
    void flush_tx_frames();

    // True if an external client is connected.
    bool is_connected() const { return client_fd.load() >= 0; }

    // Thread-safe snapshot of current stats.
    SimSocketStats get_stats() const;

private:
    void accept_thread();
    void reader_thread(int fd);
    void handle_can_rx(const uint8_t* payload, uint16_t len);
    void handle_imu(const uint8_t* payload, uint16_t len);
    bool send_message(uint8_t type, const uint8_t* data, uint16_t len);

    tNMEA2000_sim& nmea;
    TwoWire& wire;
    int server_fd = -1;
    std::atomic<int> client_fd{-1};
    std::mutex write_mutex;

    // Stats (protected by stats_mutex)
    mutable std::mutex stats_mutex;
    SimSocketStats stats;
};

// Global pointer set by hal_sim_main so sim_web.cpp can access stats.
// NULL when not in HAL_SIM mode or before socket is created.
extern SimSocket* g_sim_socket;

#endif // SIM_SOCKET_H
