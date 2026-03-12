/**
 * HAL-level simulation entry point.
 * Same cooperative loop as the ESP32 firmware, but using HAL stubs
 * so the REAL n2k.cpp, imu.cpp, and actuator.cpp run unchanged.
 *
 * Usage: ./program [--port 8080] [--tws 12] [--twd 225]
 */

#ifdef HAL_SIM

#include "config.h"
#include "app_state.h"
#include "imu.h"
#include "n2k.h"
#include "actuator.h"
#include "pilot_manager.h"
#include "seatalk.h"
#include "web.h"

#include "NMEA2000_sim.h"
#include "Wire.h"
#include "sim_environment.h"
#include "sim_socket.h"

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <thread>
#include <chrono>

// --- Timing ---

namespace sim_hal {
    inline auto& start_time() {
        static auto t0 = std::chrono::steady_clock::now();
        return t0;
    }
}

static uint32_t sim_millis() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - sim_hal::start_time());
    return static_cast<uint32_t>(elapsed.count());
}

// Provide millis() for the entire program (N2kDef.h extern, pilot_base.h, etc.)
extern "C" uint32_t millis() {
    return sim_millis();
}

// Forward declarations for sim_web.cpp
void sim_web_set_port(int port);

static volatile bool running = true;

static void signal_handler(int) {
    running = false;
}

static const char* mode_name(PilotMode m) {
    switch (m) {
        case MODE_STANDBY:  return "STANDBY";
        case MODE_COMPASS:  return "COMPASS";
        case MODE_WIND_AWA: return "AWA";
        case MODE_WIND_TWA: return "TWA";
        case MODE_VMG_UP:   return "VMG_UP";
        case MODE_VMG_DOWN: return "VMG_DN";
        default: return "???";
    }
}

int main(int argc, char* argv[]) {
    // Parse CLI args
    float tws = 12.0f;
    float twd = 225.0f;
    int port = 8080;
    int socket_port = 9876;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--tws") == 0 && i + 1 < argc) tws = atof(argv[++i]);
        else if (strcmp(argv[i], "--twd") == 0 && i + 1 < argc) twd = atof(argv[++i]);
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--socket-port") == 0 && i + 1 < argc) socket_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [--port 8080] [--tws 12] [--twd 225] [--socket-port 9876]\n", argv[0]);
            printf("  --port         HTTP server port (default: 8080)\n");
            printf("  --tws          True wind speed in knots (default: 12)\n");
            printf("  --twd          True wind direction in degrees (default: 225)\n");
            printf("  --socket-port  External simulator TCP port (default: 9876)\n");
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== Pogo 1250 Autopilot HAL Simulator ===\n");
    printf("TWS=%.1f kn  TWD=%.0f°  Port=%d  Socket=%d\n", tws, twd, port, socket_port);
    printf("Real firmware code: n2k.cpp, imu.cpp, actuator.cpp\n\n");

    // Create SimEnvironment — it feeds data into the HAL layer
    // NMEA2000 global instance is in NMEA2000_sim.cpp
    // Wire global instance is in Wire.cpp
    SimEnvironment sim_env(NMEA2000, Wire);
    sim_env.set_wind(tws, twd);

    // Create SimSocket for external simulator communication
    SimSocket sim_socket(NMEA2000, Wire);
    g_sim_socket = &sim_socket;
    sim_socket.start(socket_port);

    // Configure sim web
    sim_web_set_port(port);

    // Initialize (same as ESP32 setup())
    AppState state;
    memset(&state, 0, sizeof(state));
    state.pilot_mode = MODE_STANDBY;
    state.pilot_type = PILOT_PD;

    // Seed the environment — run a few cycles to populate N2K and IMU data
    for (int i = 0; i < 10; i++) {
        sim_env.update(0.01f);
    }

    imu_init();
    n2k_init();
    actuator_init();
    pilot_manager_init();
    web_init();

    printf("\nReady. Open http://localhost:%d/ in your browser.\n", port);
    printf("Press Ctrl+C to stop.\n\n");

    // Cooperative loop (same structure as ESP32 loop())
    uint32_t last_imu_ms = 0;
    uint32_t last_pilot_ms = 0;
    uint32_t last_actuator_ms = 0;
    uint32_t last_n2k_send_ms = 0;
    uint32_t last_print_ms = 0;

    while (running) {
        uint32_t now = sim_millis();

        // External simulator: flush TX frames and update mode
        sim_socket.flush_tx_frames();
        sim_env.set_external_active(sim_socket.is_connected());

        // N2K parse — every loop (processes injected CAN frames)
        n2k_update(state);

        // Web and p70 commands — every loop
        web_apply_commands(state);
        seatalk_apply_commands(state);

        // IMU at 20Hz
        if (now - last_imu_ms >= IMU_INTERVAL_MS) {
            float dt = (now - last_imu_ms) / 1000.0f;
            last_imu_ms = now;

            // Update sim environment (feeds data into HAL)
            sim_env.update(dt);

            // Real imu.cpp reads BNO055 registers via mock Wire
            imu_read(state);
        }

        // Pilot at 5Hz
        if (now - last_pilot_ms >= PILOT_INTERVAL_MS) {
            last_pilot_ms = now;
            pilot_manager_update(state);
        }

        // Actuator at 50Hz — real actuator.cpp reads ADC, drives PWM
        if (now - last_actuator_ms >= ACTUATOR_INTERVAL_MS) {
            last_actuator_ms = now;
            actuator_read_sensors(state);
            // Override current/voltage with simulated values (actuator.cpp has no real ADC)
            state.motor_current = sim_env.get_motor_current();
            state.supply_voltage = 12.8f;
            // Recompute 5s EMA with correct current (actuator.cpp ran it with 0)
            const float ema_alpha = (ACTUATOR_INTERVAL_MS / 1000.0f) / 5.0f;
            state.motor_current_avg += ema_alpha * (state.motor_current - state.motor_current_avg);
            actuator_update(state);
        }

        // N2K send at 5Hz (real n2k.cpp sends PGNs via sim CAN)
        if (now - last_n2k_send_ms >= N2K_SEND_INTERVAL_MS) {
            last_n2k_send_ms = now;
            n2k_send(state);
        }

        // Console status at 2Hz
        if (now - last_print_ms >= 500) {
            last_print_ms = now;
            if (sim_env.is_external_active()) {
                auto ss = sim_socket.get_stats();
                printf("\r[%6.1fs] EXT %s tgt=%.1f° hdg=%.1f° awa=%.0f° twa=%.0f° stw=%.1f kn rud=%.1f°/%.1f° rx:%u imu:%u tx:%u ",
                       now / 1000.0f,
                       mode_name(state.pilot_mode),
                       state.target_value,
                       state.heading,
                       state.awa,
                       state.twa,
                       state.stw,
                       state.rudder_target * 25.0f,
                       state.rudder_actual * 25.0f,
                       ss.can_rx_count,
                       ss.imu_rx_count,
                       ss.can_tx_count);
            } else {
                printf("\r[%6.1fs] %s tgt=%.1f° hdg=%.1f° awa=%.0f° stw=%.1f kn rud=%.1f°/%.1f° ",
                       now / 1000.0f,
                       mode_name(state.pilot_mode),
                       state.target_value,
                       state.heading,
                       state.awa,
                       state.stw,
                       state.rudder_target * 25.0f,
                       state.rudder_actual * 25.0f);
            }
            fflush(stdout);
        }

        // Sleep 1ms to avoid busy-spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    printf("\nShutting down.\n");
    return 0;
}

#endif // HAL_SIM
