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
#include <string>
#include <math.h>

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

void sim_web_set_port(int port) {
    sim_port = port;
}

// Simple JSON builder (no ArduinoJson on native)
static std::string json_status(const AppState& s) {
    char buf[1024];
    float polar_pct = 0.0f;
    if (s.tws > 1.0f && s.stw > 0.1f) {
        polar_pct = polar_get_performance_ratio(fabsf(s.twa), s.tws, s.stw) * 100.0f;
    }
    snprintf(buf, sizeof(buf),
        "{"
        "\"heading\":%.1f,\"pitch\":%.1f,\"roll\":%.1f,\"yaw_rate\":%.2f,"
        "\"awa\":%.1f,\"aws\":%.1f,\"twa\":%.1f,\"tws\":%.1f,"
        "\"stw\":%.1f,\"sog\":%.1f,\"cog\":%.1f,"
        "\"rudder_actual\":%.3f,\"rudder_target\":%.3f,"
        "\"pilot_mode\":%d,\"pilot_type\":%d,"
        "\"target_value\":%.1f,\"fault_code\":%d,"
        "\"clutch_engaged\":%s,"
        "\"motor_current\":%.1f,\"supply_voltage\":%.1f,"
        "\"polar_pct\":%.0f"
        "}",
        s.heading, s.pitch, s.roll, s.yaw_rate,
        s.awa, s.aws, s.twa, s.tws,
        s.stw, s.sog, s.cog,
        s.rudder_actual, s.rudder_target,
        (int)s.pilot_mode, (int)s.pilot_type,
        s.target_value, (int)s.fault_code,
        s.clutch_engaged ? "true" : "false",
        s.motor_current, s.supply_voltage,
        polar_pct
    );
    return buf;
}

// Minimal JSON parser for {"key": value} — handles the simple API payloads
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

    // GET /api/status
    svr.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        if (!g_sim_state) {
            res.status = 500;
            res.set_content("{\"error\":\"not ready\"}", "application/json");
            return;
        }
        std::lock_guard<std::mutex> lock(state_mutex);
        res.set_content(json_status(*g_sim_state), "application/json");
    });

    // GET /api/gains
    svr.Get("/api/gains", [](const httplib::Request&, httplib::Response& res) {
        float kp, ki, kd;
        pilot_manager_get_gains(kp, ki, kd);
        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"kp\":%.2f,\"ki\":%.3f,\"kd\":%.2f,\"pilot_type\":%d}",
            kp, ki, kd, (int)pilot_manager_get_type());
        res.set_content(buf, "application/json");
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
