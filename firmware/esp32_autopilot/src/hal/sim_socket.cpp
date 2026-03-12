#ifdef HAL_SIM

#include "sim_socket.h"
#include "NMEA2000_sim.h"
#include "Wire.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <errno.h>

// macOS uses SO_NOSIGPIPE instead of MSG_NOSIGNAL
#ifdef __APPLE__
#define MSG_NOSIGNAL 0
#endif

// Message types
static constexpr uint8_t MSG_CAN_RX  = 0x01;  // Python→ESP: CAN frame
static constexpr uint8_t MSG_CAN_TX  = 0x02;  // ESP→Python: CAN frame
static constexpr uint8_t MSG_IMU     = 0x03;  // Python→ESP: IMU registers

// Global pointer for sim_web.cpp access
SimSocket* g_sim_socket = nullptr;

SimSocket::SimSocket(tNMEA2000_sim& nmea, TwoWire& wire)
    : nmea(nmea), wire(wire) {}

SimSocket::~SimSocket() {
    int cfd = client_fd.load();
    if (cfd >= 0) close(cfd);
    if (server_fd >= 0) close(server_fd);
}

bool SimSocket::start(int port) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        printf("[SOCKET] Failed to create socket: %s\n", strerror(errno));
        return false;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("[SOCKET] Bind failed on port %d: %s\n", port, strerror(errno));
        close(server_fd);
        server_fd = -1;
        return false;
    }

    if (listen(server_fd, 1) < 0) {
        printf("[SOCKET] Listen failed: %s\n", strerror(errno));
        close(server_fd);
        server_fd = -1;
        return false;
    }

    printf("[SOCKET] Listening on port %d for external simulator\n", port);

    // Launch accept thread (detached — runs for lifetime of process)
    std::thread(&SimSocket::accept_thread, this).detach();
    return true;
}

void SimSocket::accept_thread() {
    while (server_fd >= 0) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (fd < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // Close any existing client
        int old_fd = client_fd.exchange(fd);
        if (old_fd >= 0) {
            close(old_fd);
        }

        // Reset stats for new session
        {
            std::lock_guard<std::mutex> lock(stats_mutex);
            stats = SimSocketStats();
            stats.connected = true;
        }

#ifdef __APPLE__
        int nosigpipe = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));
#endif

        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
        printf("\n[SOCKET] External simulator connected from %s:%d\n",
               ip_str, ntohs(client_addr.sin_port));

        // Launch reader thread for this client
        std::thread(&SimSocket::reader_thread, this, fd).detach();
    }
}

void SimSocket::reader_thread(int fd) {
    uint8_t header[3];  // type(1) + length(2 LE)

    while (true) {
        // Read header
        ssize_t n = 0;
        while (n < 3) {
            ssize_t r = recv(fd, header + n, 3 - n, 0);
            if (r <= 0) goto disconnected;
            n += r;
        }

        uint8_t type = header[0];
        uint16_t length = header[1] | (header[2] << 8);

        if (length > 256) {
            printf("[SOCKET] Invalid message length %u, disconnecting\n", length);
            goto disconnected;
        }

        // Read payload
        uint8_t payload[256];
        n = 0;
        while (n < length) {
            ssize_t r = recv(fd, payload + n, length - n, 0);
            if (r <= 0) goto disconnected;
            n += r;
        }

        // Dispatch
        switch (type) {
            case MSG_CAN_RX:
                handle_can_rx(payload, length);
                break;
            case MSG_IMU:
                handle_imu(payload, length);
                break;
            default:
                printf("[SOCKET] Unknown message type 0x%02x\n", type);
                break;
        }
    }

disconnected:
    // Only clear client_fd if it's still our fd (not replaced by a new client)
    int expected = fd;
    if (client_fd.compare_exchange_strong(expected, -1)) {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.connected = false;
    }
    close(fd);
    printf("\n[SOCKET] External simulator disconnected\n");
}

static unsigned long extract_pgn(unsigned long can_id) {
    unsigned char pdu_format = (can_id >> 16) & 0xFF;
    if (pdu_format < 240) {
        return (can_id >> 8) & 0x1FF00UL;
    }
    return (can_id >> 8) & 0x1FFFFUL;
}

void SimSocket::handle_can_rx(const uint8_t* payload, uint16_t len) {
    // Payload: 4B CAN ID (LE) + 1B DLC + up to 8B data
    if (len < 5) return;

    SimCANFrame frame;
    frame.id = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
    frame.len = payload[4];
    if (frame.len > 8) frame.len = 8;
    if (len < 5u + frame.len) return;

    memcpy(frame.buf, payload + 5, frame.len);

    unsigned long pgn = extract_pgn(frame.id);

    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.can_rx_count++;

        if (stats.can_rx_count == 1) {
            printf("\n[SOCKET] First CAN frame received: PGN %lu, %d bytes\n",
                   pgn, frame.len);
        }

        switch (pgn) {
            case 130306: stats.pgn_wind_count++; break;
            case 128259: stats.pgn_stw_count++; break;
            case 129026: stats.pgn_cog_sog_count++; break;
            case 127250: stats.pgn_heading_count++; break;
            default:     stats.pgn_other_count++; break;
        }
    }

    nmea.inject_frame(frame);
}

void SimSocket::handle_imu(const uint8_t* payload, uint16_t len) {
    // Payload: 6 x int16 LE: heading, roll, pitch, gyro_x, gyro_y, gyro_z
    // BNO055 raw format: 1 LSB = 1/16 degree or 1/16 dps
    if (len < 12) return;

    int16_t heading = (int16_t)(payload[0] | (payload[1] << 8));
    int16_t roll    = (int16_t)(payload[2] | (payload[3] << 8));
    int16_t pitch   = (int16_t)(payload[4] | (payload[5] << 8));
    int16_t gyro_x  = (int16_t)(payload[6] | (payload[7] << 8));
    int16_t gyro_y  = (int16_t)(payload[8] | (payload[9] << 8));
    int16_t gyro_z  = (int16_t)(payload[10] | (payload[11] << 8));

    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.imu_rx_count++;

        if (stats.imu_rx_count == 1) {
            printf("\n[SOCKET] First IMU data received: hdg=%.1f° roll=%.1f° pitch=%.1f° gyro_z=%.1f°/s\n",
                   heading / 16.0f, roll / 16.0f, pitch / 16.0f, gyro_z / 16.0f);
        }

        stats.imu_heading = heading / 16.0f;
        stats.imu_roll    = roll / 16.0f;
        stats.imu_pitch   = pitch / 16.0f;
        stats.imu_gyro_x  = gyro_x / 16.0f;
        stats.imu_gyro_y  = gyro_y / 16.0f;
        stats.imu_gyro_z  = gyro_z / 16.0f;
    }

    wire.set_register_16(0x1A, heading);  // EULER_H
    wire.set_register_16(0x1C, roll);     // EULER_R
    wire.set_register_16(0x1E, pitch);    // EULER_P
    wire.set_register_16(0x14, gyro_x);   // GYRO_X
    wire.set_register_16(0x16, gyro_y);   // GYRO_Y
    wire.set_register_16(0x18, gyro_z);   // GYRO_Z
}

void SimSocket::flush_tx_frames() {
    int fd = client_fd.load();
    if (fd < 0) return;

    SimCANFrame frame;
    while (nmea.get_tx_frame(frame)) {
        // Build payload: 4B CAN ID (LE) + 1B DLC + data
        uint8_t payload[13];
        payload[0] = frame.id & 0xFF;
        payload[1] = (frame.id >> 8) & 0xFF;
        payload[2] = (frame.id >> 16) & 0xFF;
        payload[3] = (frame.id >> 24) & 0xFF;
        payload[4] = frame.len;
        memcpy(payload + 5, frame.buf, frame.len);

        {
            std::lock_guard<std::mutex> lock(stats_mutex);
            stats.can_tx_count++;
        }
        send_message(MSG_CAN_TX, payload, 5 + frame.len);
    }
}

SimSocketStats SimSocket::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex);
    return stats;
}

bool SimSocket::send_message(uint8_t type, const uint8_t* data, uint16_t len) {
    int fd = client_fd.load();
    if (fd < 0) return false;

    uint8_t header[3];
    header[0] = type;
    header[1] = len & 0xFF;
    header[2] = (len >> 8) & 0xFF;

    std::lock_guard<std::mutex> lock(write_mutex);

    // Send header + payload
    ssize_t n = send(fd, header, 3, MSG_NOSIGNAL);
    if (n <= 0) return false;
    n = send(fd, data, len, MSG_NOSIGNAL);
    return n > 0;
}

#endif // HAL_SIM
