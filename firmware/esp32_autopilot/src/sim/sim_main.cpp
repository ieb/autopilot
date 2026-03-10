#ifdef SIM_BUILD

/**
 * Native simulation entry point.
 * Runs the same cooperative loop as the ESP32 firmware
 * with fake hardware and a real HTTP server.
 *
 * Usage: ./program [--port 8080] [--tws 12] [--twd 225]
 */

#include "config.h"
#include "app_state.h"
#include "imu.h"
#include "n2k.h"
#include "actuator.h"
#include "pilot_manager.h"
#include "web.h"
#include "sim_hal.h"

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <thread>
#include <chrono>

// Forward declarations for sim config functions
void sim_n2k_set_wind(float tws, float twd);
void sim_web_set_port(int port);
void sim_imu_update_dynamics(float rudder_actual_norm, float dt);

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

// Override millis() for the sim — pilot_base.h declares extern "C" millis()
// under SIM_BUILD. N2kDef.h also declares extern "C" millis() when !ARDUINO.
// We provide the real implementation here with matching linkage.
extern "C" uint32_t millis() {
    return sim_millis();
}

int main(int argc, char* argv[]) {
    // Parse CLI args
    float tws = 12.0f;
    float twd = 225.0f;
    int port = 8080;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--tws") == 0 && i + 1 < argc) tws = atof(argv[++i]);
        else if (strcmp(argv[i], "--twd") == 0 && i + 1 < argc) twd = atof(argv[++i]);
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [--port 8080] [--tws 12] [--twd 225]\n", argv[0]);
            printf("  --port  HTTP server port (default: 8080)\n");
            printf("  --tws   True wind speed in knots (default: 12)\n");
            printf("  --twd   True wind direction in degrees (default: 225)\n");
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== Pogo 1250 Autopilot Simulator ===\n");
    printf("TWS=%.1f kn  TWD=%.0f°  Port=%d\n\n", tws, twd, port);

    // Configure sim modules
    sim_n2k_set_wind(tws, twd);
    sim_web_set_port(port);

    // Initialize (same as ESP32 setup())
    AppState state;
    memset(&state, 0, sizeof(state));
    state.pilot_mode = MODE_STANDBY;
    state.pilot_type = PILOT_PD;

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

        // N2K parse — every loop
        n2k_update(state);

        // Web commands — every loop
        web_apply_commands(state);

        // IMU at 20Hz
        if (now - last_imu_ms >= IMU_INTERVAL_MS) {
            float dt = (now - last_imu_ms) / 1000.0f;
            last_imu_ms = now;

            // Update dynamics (heading responds to rudder)
            sim_imu_update_dynamics(state.rudder_actual, dt);
            imu_read(state);
        }

        // Pilot at 5Hz
        if (now - last_pilot_ms >= PILOT_INTERVAL_MS) {
            last_pilot_ms = now;
            pilot_manager_update(state);
        }

        // Actuator at 50Hz
        if (now - last_actuator_ms >= ACTUATOR_INTERVAL_MS) {
            last_actuator_ms = now;
            actuator_read_sensors(state);
            actuator_update(state);
        }

        // N2K send at 5Hz (no-op in sim)
        if (now - last_n2k_send_ms >= N2K_SEND_INTERVAL_MS) {
            last_n2k_send_ms = now;
            n2k_send(state);
        }

        // Console status at 2Hz
        if (now - last_print_ms >= 500) {
            last_print_ms = now;
            printf("\r[%6.1fs] %s tgt=%.1f° hdg=%.1f° awa=%.0f° stw=%.1f kn rud=%.1f°/%.1f° ",
                   now / 1000.0f,
                   mode_name(state.pilot_mode),
                   state.target_value,
                   state.heading,
                   state.awa,
                   state.stw,
                   state.rudder_target * 25.0f,
                   state.rudder_actual * 25.0f);
            fflush(stdout);
        }

        // Sleep 1ms to avoid busy-spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    printf("\nShutting down.\n");
    return 0;
}

#endif // SIM_BUILD
