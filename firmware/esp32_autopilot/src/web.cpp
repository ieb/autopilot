#ifndef NATIVE_BUILD

#include "web.h"
#include "config.h"
#include "pilot_manager.h"
#include "actuator.h"
#include "polar.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

static AsyncWebServer server(80);

// Shared state pointer for web callbacks
static volatile AppState* g_web_state = nullptr;

// Pending commands from web (applied in main loop)
static volatile bool cmd_pending = false;
static volatile PilotMode cmd_mode = MODE_STANDBY;
static volatile float cmd_target = 0.0f;
static volatile PilotType cmd_pilot_type = PILOT_PD;
static volatile bool cmd_mode_change = false;
static volatile bool cmd_type_change = false;
static volatile bool cmd_gains_change = false;
static volatile float cmd_kp = DEFAULT_KP;
static volatile float cmd_ki = DEFAULT_KI;
static volatile float cmd_kd = DEFAULT_KD;

static const char* AP_SSID = "PogoAutopilot";
static const char* AP_PASS = "pogo1250";

void web_init() {
    // Start as WiFi Access Point
    WiFi.softAP(AP_SSID, AP_PASS);

    // Initialize SPIFFS for static files
    SPIFFS.begin(true);

    // GET / — serve dashboard
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // GET /api/status — JSON status
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (!g_web_state) {
            request->send(500, "application/json", "{\"error\":\"not ready\"}");
            return;
        }

        const volatile AppState& s = *g_web_state;
        JsonDocument doc;
        doc["heading"] = s.heading;
        doc["pitch"] = s.pitch;
        doc["roll"] = s.roll;
        doc["yaw_rate"] = s.yaw_rate;
        doc["awa"] = s.awa;
        doc["aws"] = s.aws;
        doc["twa"] = s.twa;
        doc["tws"] = s.tws;
        doc["stw"] = s.stw;
        doc["sog"] = s.sog;
        doc["cog"] = s.cog;
        doc["rudder_actual"] = s.rudder_actual;
        doc["rudder_target"] = s.rudder_target;
        doc["pilot_mode"] = (int)s.pilot_mode;
        doc["pilot_type"] = (int)s.pilot_type;
        doc["target_value"] = s.target_value;
        doc["fault_code"] = s.fault_code;
        doc["clutch_engaged"] = s.clutch_engaged;
        doc["motor_current"] = s.motor_current;
        doc["supply_voltage"] = s.supply_voltage;

        // Performance ratio if sailing
        if (s.tws > 1.0f && s.stw > 0.1f) {
            doc["polar_pct"] = polar_get_performance_ratio(
                fabsf(s.twa), s.tws, s.stw) * 100.0f;
        }

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // GET /api/gains — current pilot gains
    server.on("/api/gains", HTTP_GET, [](AsyncWebServerRequest* request) {
        float kp, ki, kd;
        pilot_manager_get_gains(kp, ki, kd);

        JsonDocument doc;
        doc["kp"] = kp;
        doc["ki"] = ki;
        doc["kd"] = kd;
        doc["pilot_type"] = (int)pilot_manager_get_type();

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // POST /api/gains — set gains
    server.on("/api/gains", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"error\":\"bad json\"}");
                return;
            }
            cmd_kp = doc["kp"] | DEFAULT_KP;
            cmd_ki = doc["ki"] | DEFAULT_KI;
            cmd_kd = doc["kd"] | DEFAULT_KD;
            cmd_gains_change = true;
            request->send(200, "application/json", "{\"ok\":true}");
        }
    );

    // POST /api/mode — set pilot mode + target
    server.on("/api/mode", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"error\":\"bad json\"}");
                return;
            }
            cmd_mode = (PilotMode)(doc["mode"] | 0);
            cmd_target = doc["target"] | 0.0f;
            cmd_mode_change = true;
            request->send(200, "application/json", "{\"ok\":true}");
        }
    );

    // POST /api/pilot — select pilot type
    server.on("/api/pilot", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"error\":\"bad json\"}");
                return;
            }
            cmd_pilot_type = (PilotType)(doc["type"] | 0);
            cmd_type_change = true;
            request->send(200, "application/json", "{\"ok\":true}");
        }
    );

    // POST /api/calibrate/center — set rudder center position
    server.on("/api/calibrate/center", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibrate_center();
        request->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /api/calibrate/port — set rudder port limit
    server.on("/api/calibrate/port", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibrate_port();
        request->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /api/calibrate/stbd — set rudder starboard limit
    server.on("/api/calibrate/stbd", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibrate_stbd();
        request->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /api/calibrate/linearise — equalise port/stbd ranges
    server.on("/api/calibrate/linearise", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibration_linearise();
        request->send(200, "application/json", "{\"ok\":true}");
    });

    // GET /api/calibrate — current calibration values + live ADC reading
    server.on("/api/calibrate", HTTP_GET, [](AsyncWebServerRequest* request) {
        uint32_t center, port, stbd;
        actuator_get_calibration(center, port, stbd);

        JsonDocument doc;
        doc["center_mv"] = center;
        doc["port_mv"] = port;
        doc["stbd_mv"] = stbd;
        doc["live_mv"] = actuator_read_raw_mv();

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    server.begin();
}

void web_apply_commands(AppState& state) {
    g_web_state = &state;

    if (cmd_mode_change) {
        cmd_mode_change = false;
        state.pilot_mode = cmd_mode;
        state.target_value = cmd_target;
        state.clutch_requested = (cmd_mode != MODE_STANDBY);
        state.last_pilot_ms = millis();  // prevent watchdog on engage
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

#endif // NATIVE_BUILD
