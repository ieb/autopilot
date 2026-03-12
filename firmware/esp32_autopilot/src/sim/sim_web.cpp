#ifdef SIM_BUILD

#include "web.h"
#include "config.h"
#include "pilot_manager.h"
#include "actuator.h"
#include "polar.h"
#include "sim_hal.h"

#include "httplib.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <math.h>

#ifdef HAL_SIM
#include "sim_socket.h"
#endif

static httplib::Server svr;
static std::thread server_thread;
static std::mutex state_mutex;

// Shared state pointer
static AppState* g_sim_state = nullptr;

// Pending commands (same pattern as ESP32 web.cpp)
static volatile bool cmd_mode_change = false;
static volatile PilotMode cmd_mode = MODE_STANDBY;
static volatile float cmd_target = 0.0f;
static volatile PilotType cmd_pilot_type = PILOT_PD;
static volatile bool cmd_type_change = false;
static volatile bool cmd_gains_change = false;
static volatile float cmd_kp = DEFAULT_KP;
static volatile float cmd_ki = DEFAULT_KI;
static volatile float cmd_kd = DEFAULT_KD;

static int sim_port = 8080;

// SSE client tracking
static std::atomic<bool> sse_client_connected{false};

void sim_web_set_port(int port) {
    sim_port = port;
}

// Build combined JSON for SSE (status + gains + calibration + socket)
static std::string build_sse_json(const AppState& s) {
    char buf[2048];
    float polar_pct = 0.0f;
    if (s.tws > 1.0f && s.stw > 0.1f) {
        polar_pct = polar_get_performance_ratio(fabsf(s.twa), s.tws, s.stw) * 100.0f;
    }

    float g_kp, g_ki, g_kd;
    pilot_manager_get_gains(g_kp, g_ki, g_kd);
    float confidence = pilot_manager_get_adaptive_confidence();
    float g_kr = 0.0f, g_kd_plant = 0.0f;
    pilot_manager_get_plant_params(g_kr, g_kd_plant);

    uint32_t center, port_mv, stbd;
    actuator_get_calibration(center, port_mv, stbd);

    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"heading\":%.1f,\"pitch\":%.1f,\"roll\":%.1f,\"yaw_rate\":%.2f,"
        "\"awa\":%.1f,\"aws\":%.1f,\"twa\":%.1f,\"tws\":%.1f,"
        "\"stw\":%.1f,\"sog\":%.1f,\"cog\":%.1f,"
        "\"rudder_actual\":%.3f,\"rudder_target\":%.3f,"
        "\"pilot_mode\":%d,\"pilot_type\":%d,"
        "\"target_value\":%.1f,\"fault_code\":%d,"
        "\"clutch_engaged\":%s,\"clutch_requested\":%s,"
        "\"motor_current\":%.2f,\"motor_current_avg\":%.2f,\"supply_voltage\":%.1f,"
        "\"polar_pct\":%.0f,"
        "\"gains_kp\":%.3f,\"gains_ki\":%.3f,\"gains_kd\":%.3f,"
        "\"adaptive_confidence\":%.3f,"
        "\"plant_kr\":%.4f,\"plant_kd\":%.4f,"
        "\"cal_center\":%u,\"cal_port\":%u,\"cal_stbd\":%u,\"cal_live\":%u",
        s.heading, s.pitch, s.roll, s.yaw_rate,
        s.awa, s.aws, s.twa, s.tws,
        s.stw, s.sog, s.cog,
        s.rudder_actual, s.rudder_target,
        (int)s.pilot_mode, (int)s.pilot_type,
        s.target_value, (int)s.fault_code,
        s.clutch_engaged ? "true" : "false",
        s.clutch_requested ? "true" : "false",
        s.motor_current, s.motor_current_avg, s.supply_voltage,
        polar_pct,
        g_kp, g_ki, g_kd,
        confidence,
        g_kr, g_kd_plant,
        center, port_mv, stbd, actuator_read_raw_mv()
    );

#ifdef HAL_SIM
    // Append socket stats if available
    if (g_sim_socket) {
        auto ss = g_sim_socket->get_stats();
        n += snprintf(buf + n, sizeof(buf) - n,
            ","
            "\"sock_connected\":%s,"
            "\"sock_can_rx\":%u,\"sock_imu_rx\":%u,\"sock_can_tx\":%u,"
            "\"sock_pgn_wind\":%u,\"sock_pgn_stw\":%u,\"sock_pgn_cog_sog\":%u,"
            "\"sock_pgn_heading\":%u,\"sock_pgn_other\":%u,"
            "\"sock_imu_heading\":%.1f,\"sock_imu_roll\":%.1f,\"sock_imu_pitch\":%.1f,"
            "\"sock_imu_gyro_x\":%.1f,\"sock_imu_gyro_y\":%.1f,\"sock_imu_gyro_z\":%.1f",
            ss.connected ? "true" : "false",
            ss.can_rx_count, ss.imu_rx_count, ss.can_tx_count,
            ss.pgn_wind_count, ss.pgn_stw_count, ss.pgn_cog_sog_count,
            ss.pgn_heading_count, ss.pgn_other_count,
            ss.imu_heading, ss.imu_roll, ss.imu_pitch,
            ss.imu_gyro_x, ss.imu_gyro_y, ss.imu_gyro_z
        );
    }
#endif

    snprintf(buf + n, sizeof(buf) - n, "}");
    return buf;
}

// Minimal JSON parser for {\"key\": value} — handles the simple API payloads
static float json_get_float(const std::string& body, const char* key, float def) {
    std::string needle = std::string("\"") + key + "\"";
    auto pos = body.find(needle);
    if (pos == std::string::npos) return def;
    pos = body.find(':', pos);
    if (pos == std::string::npos) return def;
    return strtof(body.c_str() + pos + 1, nullptr);
}

static int json_get_int(const std::string& body, const char* key, int def) {
    return (int)json_get_float(body, key, (float)def);
}

static void setup_routes(const std::string& data_dir) {
    // Serve index.html
    svr.Get("/", [data_dir](const httplib::Request&, httplib::Response& res) {
        std::ifstream f(data_dir + "/index.html");
        if (f.good()) {
            std::string content((std::istreambuf_iterator<char>(f)),
                                std::istreambuf_iterator<char>());
            res.set_content(content, "text/html");
        } else {
            res.status = 404;
            res.set_content("index.html not found", "text/plain");
        }
    });

    // SSE endpoint — streams all state at 5 Hz
    svr.Get("/api/events", [](const httplib::Request&, httplib::Response& res) {
        sse_client_connected = true;
        res.set_chunked_content_provider(
            "text/event-stream",
            [](size_t /*offset*/, httplib::DataSink& sink) -> bool {
                while (true) {
                    if (!g_sim_state) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        continue;
                    }

                    std::string json;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        json = build_sse_json(*g_sim_state);
                    }

                    std::string sse = "event: state\ndata: " + json + "\n\n";
                    if (!sink.is_writable()) {
                        sse_client_connected = false;
                        return false;
                    }
                    sink.write(sse.c_str(), sse.size());

                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                return true;
            },
            [](bool /*success*/) {
                sse_client_connected = false;
            }
        );
    });

    // POST /api/gains
    svr.Post("/api/gains", [](const httplib::Request& req, httplib::Response& res) {
        cmd_kp = json_get_float(req.body, "kp", DEFAULT_KP);
        cmd_ki = json_get_float(req.body, "ki", DEFAULT_KI);
        cmd_kd = json_get_float(req.body, "kd", DEFAULT_KD);
        cmd_gains_change = true;
        res.set_content("{\"ok\":true}", "application/json");
    });

    // POST /api/mode
    svr.Post("/api/mode", [](const httplib::Request& req, httplib::Response& res) {
        cmd_mode = (PilotMode)json_get_int(req.body, "mode", 0);
        cmd_target = json_get_float(req.body, "target", 0.0f);
        cmd_mode_change = true;
        res.set_content("{\"ok\":true}", "application/json");
    });

    // POST /api/pilot
    svr.Post("/api/pilot", [](const httplib::Request& req, httplib::Response& res) {
        cmd_pilot_type = (PilotType)json_get_int(req.body, "type", 0);
        cmd_type_change = true;
        res.set_content("{\"ok\":true}", "application/json");
    });

    // Calibration endpoints
    svr.Post("/api/calibrate/center", [](const httplib::Request&, httplib::Response& res) {
        actuator_calibrate_center();
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Post("/api/calibrate/port", [](const httplib::Request&, httplib::Response& res) {
        actuator_calibrate_port();
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Post("/api/calibrate/stbd", [](const httplib::Request&, httplib::Response& res) {
        actuator_calibrate_stbd();
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Post("/api/calibrate/linearise", [](const httplib::Request&, httplib::Response& res) {
        actuator_calibration_linearise();
        res.set_content("{\"ok\":true}", "application/json");
    });

    svr.Get("/api/calibrate", [](const httplib::Request&, httplib::Response& res) {
        uint32_t center, port, stbd;
        actuator_get_calibration(center, port, stbd);
        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"center_mv\":%u,\"port_mv\":%u,\"stbd_mv\":%u,\"live_mv\":%u}",
            center, port, stbd, actuator_read_raw_mv());
        res.set_content(buf, "application/json");
    });
}

void web_init() {
    // Find data directory (relative to CWD or binary location)
    std::string data_dir = "data";

    setup_routes(data_dir);

    // Start HTTP server in background thread
    server_thread = std::thread([&]() {
        printf("[SIM] Web server starting on http://localhost:%d/\n", sim_port);
        svr.listen("0.0.0.0", sim_port);
    });
    server_thread.detach();

    // Give server a moment to bind
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    printf("[SIM] Web UI at http://localhost:%d/\n", sim_port);
}

void web_apply_commands(AppState& state) {
    g_sim_state = &state;

    if (cmd_mode_change) {
        cmd_mode_change = false;
        state.pilot_mode = cmd_mode;
        state.target_value = cmd_target;
        state.clutch_requested = (cmd_mode != MODE_STANDBY);
        state.last_pilot_ms = sim_millis();  // prevent watchdog on engage
        if (cmd_mode == MODE_VMG_UP || cmd_mode == MODE_VMG_DOWN) {
            pilot_manager_latch_tack(state.twa);
        }
        pilot_manager_set_mode(cmd_mode, cmd_target);
    }

    if (cmd_type_change) {
        cmd_type_change = false;
        state.pilot_type = cmd_pilot_type;
        pilot_manager_set_type(cmd_pilot_type);
    }

    if (cmd_gains_change) {
        cmd_gains_change = false;
        pilot_manager_set_gains(cmd_kp, cmd_ki, cmd_kd);
    }
}

#endif // SIM_BUILD
