#ifndef NATIVE_BUILD

#include "web.h"
#include "config.h"
#include "pilot_manager.h"
#include "actuator.h"
#include "polar.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncEventSource.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

static AsyncWebServer server(80);
static AsyncEventSource events("/api/events");

// Shared state pointer for web callbacks
static volatile AppState* g_web_state = nullptr;

// Pending commands from web (applied in main loop)
static volatile bool cmd_mode_change = false;
static volatile PilotMode cmd_mode = MODE_STANDBY;
static volatile float cmd_target = 0.0f;
static volatile PilotType cmd_pilot_type = PILOT_PD;
static volatile bool cmd_type_change = false;
static volatile bool cmd_gains_change = false;
static volatile float cmd_kp = DEFAULT_KP;
static volatile float cmd_ki = DEFAULT_KI;
static volatile float cmd_kd = DEFAULT_KD;

static const char* AP_SSID = "PogoAutopilot";
static const char* AP_PASS = "pogo1250";

// SSE push rate
static uint32_t last_sse_ms = 0;
static const uint32_t SSE_INTERVAL_MS = 200;  // 5 Hz

void web_init() {
    // Start as WiFi Access Point
    WiFi.softAP(AP_SSID, AP_PASS);

    // Initialize SPIFFS for static files
    SPIFFS.begin(true);

    // GET / — serve dashboard
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // SSE event source
    events.onConnect([](AsyncEventSourceClient* client) {
        client->send("connected", "hello", millis(), 1000);
    });
    server.addHandler(&events);

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

    // Calibration endpoints
    server.on("/api/calibrate/center", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibrate_center();
        request->send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/api/calibrate/port", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibrate_port();
        request->send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/api/calibrate/stbd", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibrate_stbd();
        request->send(200, "application/json", "{\"ok\":true}");
    });
    server.on("/api/calibrate/linearise", HTTP_POST, [](AsyncWebServerRequest* request) {
        actuator_calibration_linearise();
        request->send(200, "application/json", "{\"ok\":true}");
    });

    server.begin();
}

// Build combined JSON for SSE push
static void build_sse_json(char* buf, size_t len, const AppState& s) {
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

    snprintf(buf, len,
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
        "\"cal_center\":%u,\"cal_port\":%u,\"cal_stbd\":%u,\"cal_live\":%u"
        "}",
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
}

void web_apply_commands(AppState& state) {
    g_web_state = &state;

    if (cmd_mode_change) {
        cmd_mode_change = false;
        state.pilot_mode = cmd_mode;
        state.target_value = cmd_target;
        state.clutch_requested = (cmd_mode != MODE_STANDBY);
        state.last_pilot_ms = millis();  // prevent watchdog on engage
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

    // Push SSE at 5 Hz
    uint32_t now = millis();
    if (events.count() > 0 && now - last_sse_ms >= SSE_INTERVAL_MS) {
        last_sse_ms = now;
        char buf[1024];
        build_sse_json(buf, sizeof(buf), state);
        events.send(buf, "state", now);
    }
}

#endif // NATIVE_BUILD
