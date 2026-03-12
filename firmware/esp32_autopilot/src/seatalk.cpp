#if !defined(NATIVE_BUILD) || defined(HAL_SIM)

#include "seatalk.h"
#include "config.h"
#include "pilot_manager.h"
#include <math.h>

#ifdef HAL_SIM
extern "C" uint32_t millis();
#else
#include <Arduino.h>
#endif

#include <N2kMsg.h>

// Defined in n2k.cpp
extern void n2k_send_msg(tN2kMsg& msg);

// Raymarine manufacturer header (little-endian)
#define RAYMARINE_MFR_LO  0x3B
#define RAYMARINE_MFR_HI  0x9F  // includes reserved bits + industry code Marine

// Keystroke codes from PGN 126720
#define KEY_PLUS_1         0x07F8
#define KEY_MINUS_1        0x05FA
#define KEY_PLUS_10        0x08F7
#define KEY_MINUS_10       0x06F9
#define KEY_TACK_STBD      0x22DD
#define KEY_TACK_PORT      0x21DE

// Pilot mode bytes (from PGN 126208 / PGN 65379)
#define SEATALK_MODE_STANDBY     0x00
#define SEATALK_MODE_AUTO        0x40
#define SEATALK_SUBMODE_STANDBY  0x00
#define SEATALK_SUBMODE_WIND     0x01
#define SEATALK_SUBMODE_TRACK    0x01
#define SEATALK_MODE_TRACK       0x80
#define SEATALK_MODE_NODRIFT     0x81

// State
static uint32_t p70_last_heartbeat_ms = 0;
static bool p70_detected = false;

// TX timing (all driven from seatalk_send called at N2K_SEND_RATE_HZ)
static uint32_t last_pilot_mode_ms = 0;
static uint32_t last_pilot_heading_ms = 0;
static uint32_t last_pilot_locked_ms = 0;
static uint32_t last_pilot_wind_ms = 0;
static uint8_t heading_sid = 0;

// Pending command from p70 keystrokes
static volatile bool cmd_heading_adjust = false;
static volatile float cmd_heading_delta = 0.0f;
static volatile bool cmd_mode_change = false;
static volatile PilotMode cmd_new_mode = MODE_STANDBY;
static volatile float cmd_new_target = 0.0f;

// ============================================================================
// Helper: check Raymarine manufacturer header
// ============================================================================

static bool is_raymarine(const unsigned char* data, int len) {
    if (len < 2) return false;
    return data[0] == RAYMARINE_MFR_LO && data[1] == RAYMARINE_MFR_HI;
}

// ============================================================================
// RX: Parse PGN 126720 — Seatalk1 Keystroke Commands
// ============================================================================

static bool parse_keystroke(const unsigned char* data, int len, AppState& state) {
    // Keystroke message is 22 bytes:
    // 3b 9f f0 81 86 21 [CMD_HI] [CMD_LO] ...
    if (len < 8) return false;

    // Verify Raymarine header
    if (data[0] != RAYMARINE_MFR_LO || data[1] != RAYMARINE_MFR_HI) return false;

    // Check for keystroke signature: byte 2=0xF0, bytes 3-5=0x81 0x86 0x21
    if (data[2] != 0xF0 || data[3] != 0x81 || data[4] != 0x86 || data[5] != 0x21)
        return false;

    uint16_t key = ((uint16_t)data[6] << 8) | data[7];

    switch (key) {
        case KEY_PLUS_1:
            cmd_heading_delta = 1.0f;
            cmd_heading_adjust = true;
            break;
        case KEY_MINUS_1:
            cmd_heading_delta = -1.0f;
            cmd_heading_adjust = true;
            break;
        case KEY_PLUS_10:
            cmd_heading_delta = 10.0f;
            cmd_heading_adjust = true;
            break;
        case KEY_MINUS_10:
            cmd_heading_delta = -10.0f;
            cmd_heading_adjust = true;
            break;
        case KEY_TACK_STBD:
        case KEY_TACK_PORT:
            if (state.pilot_mode == MODE_WIND_AWA || state.pilot_mode == MODE_WIND_TWA) {
                // Tack: negate the wind target angle (AWA/TWA sign flip)
                cmd_new_target = -state.target_value;
                cmd_new_mode = state.pilot_mode;
                cmd_mode_change = true;
            } else if (state.pilot_mode == MODE_VMG_UP || state.pilot_mode == MODE_VMG_DOWN) {
                // Tack in VMG: flip the latched tack side
                pilot_manager_tack();
            }
            break;
        default:
            return false;
    }
    return true;
}

// ============================================================================
// RX: Parse PGN 126208 — NMEA Command Group (Mode Change)
// ============================================================================

static bool parse_mode_command(const unsigned char* data, int len, AppState& state) {
    // Mode change command (17 bytes):
    // 01 63 ff 00 f8 04 01 3b 07 03 04 04 [MODE] [SUBMODE] 05 ff ff
    //
    // Heading set command (14 bytes):
    // 01 50 ff 00 f8 03 01 3b 07 03 04 06 [LO] [HI]

    if (len < 14) return false;

    // Function code must be 0x01 (Command)
    if (data[0] != 0x01) return false;

    // Check for mode change: writing to PGN 65379 (bytes 1-2 = 0x63, 0xFF)
    if (data[1] == 0x63 && data[2] == 0xFF && len >= 14) {
        uint8_t mode = data[12];
        uint8_t submode = (len > 13) ? data[13] : 0;

        if (mode == SEATALK_MODE_STANDBY && submode == SEATALK_SUBMODE_STANDBY) {
            cmd_new_mode = MODE_STANDBY;
            cmd_new_target = 0.0f;
            cmd_mode_change = true;
        } else if (mode == SEATALK_MODE_AUTO && submode == SEATALK_SUBMODE_STANDBY) {
            // Auto (compass) — lock current heading
            cmd_new_mode = MODE_COMPASS;
            cmd_new_target = state.heading;
            cmd_mode_change = true;
        } else if (mode == SEATALK_MODE_STANDBY && submode == SEATALK_SUBMODE_WIND) {
            // Wind mode — lock current AWA
            cmd_new_mode = MODE_WIND_AWA;
            cmd_new_target = state.awa;
            cmd_mode_change = true;
        } else if (mode == SEATALK_MODE_TRACK && submode == SEATALK_SUBMODE_TRACK) {
            // Track mode — not currently supported, treat as compass
            cmd_new_mode = MODE_COMPASS;
            cmd_new_target = state.heading;
            cmd_mode_change = true;
        }
        return true;
    }

    // Check for heading set: writing to PGN 65360 (bytes 1-2 = 0x50, 0xFF)
    if (data[1] == 0x50 && data[2] == 0xFF && len >= 14) {
        uint16_t heading_raw = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
        float heading_rad = heading_raw / 10000.0f;
        float heading_deg = heading_rad * (180.0f / M_PI);

        // Wrap to 0-360
        while (heading_deg < 0) heading_deg += 360.0f;
        while (heading_deg >= 360.0f) heading_deg -= 360.0f;

        if (state.pilot_mode == MODE_COMPASS) {
            cmd_new_mode = MODE_COMPASS;
            cmd_new_target = heading_deg;
            cmd_mode_change = true;
        }
        return true;
    }

    return false;
}

// ============================================================================
// RX: Parse PGN 65374 — Keypad Heartbeat
// ============================================================================

static bool parse_heartbeat(const unsigned char* data, int len) {
    if (len < 2) return false;
    if (!is_raymarine(data, len)) return false;

    p70_last_heartbeat_ms = millis();
    p70_detected = true;
    return true;
}

// ============================================================================
// Public API
// ============================================================================

void seatalk_init() {
    p70_detected = false;
    p70_last_heartbeat_ms = 0;
}

bool seatalk_handle_msg(const unsigned char* data, int data_len, unsigned long pgn,
                        unsigned char source, AppState& state) {
    (void)source;  // available if needed for filtering
    switch (pgn) {
        case 126720:
            return parse_keystroke(data, data_len, state);
        case 126208:
            return parse_mode_command(data, data_len, state);
        case 65374:
            return parse_heartbeat(data, data_len);
        case 65371:
            // Keypad message — note p70 is alive
            if (is_raymarine(data, data_len)) {
                p70_last_heartbeat_ms = millis();
                p70_detected = true;
            }
            return true;
        default:
            return false;
    }
}

bool seatalk_p70_present() {
    if (!p70_detected) return false;
    // Consider p70 gone if no heartbeat for 5 seconds
    return (millis() - p70_last_heartbeat_ms) < 5000;
}

uint32_t seatalk_p70_last_heartbeat() {
    return p70_last_heartbeat_ms;
}

// ============================================================================
// Apply pending commands from p70 (call from web_apply_commands context)
// ============================================================================

void seatalk_apply_commands(AppState& state) {
    if (cmd_heading_adjust) {
        cmd_heading_adjust = false;
        if (state.pilot_mode == MODE_COMPASS) {
            float new_target = state.target_value + cmd_heading_delta;
            // Wrap to 0-360
            while (new_target < 0) new_target += 360.0f;
            while (new_target >= 360.0f) new_target -= 360.0f;
            state.target_value = new_target;
        } else if (state.pilot_mode == MODE_WIND_AWA || state.pilot_mode == MODE_WIND_TWA) {
            state.target_value += cmd_heading_delta;
        }
    }

    if (cmd_mode_change) {
        cmd_mode_change = false;
        state.pilot_mode = cmd_new_mode;
        state.target_value = cmd_new_target;
        state.clutch_requested = (cmd_new_mode != MODE_STANDBY);
        state.last_pilot_ms = millis();
        if (cmd_new_mode == MODE_VMG_UP || cmd_new_mode == MODE_VMG_DOWN) {
            pilot_manager_latch_tack(state.twa);
        }
        pilot_manager_set_mode(cmd_new_mode, cmd_new_target);
    }
}

// ============================================================================
// TX: Send Seatalk PGNs
// ============================================================================

static void send_pilot_mode(const AppState& state) {
    tN2kMsg msg;
    msg.Init(7, 65379, 28);  // priority 7, our source address 28
    msg.AddByte(RAYMARINE_MFR_LO);
    msg.AddByte(RAYMARINE_MFR_HI);
    msg.AddByte(0x01);  // proprietary field

    // Map our pilot mode to Seatalk mode/submode
    uint8_t mode = SEATALK_MODE_STANDBY;
    uint8_t submode = SEATALK_SUBMODE_STANDBY;

    switch (state.pilot_mode) {
        case MODE_COMPASS:
            mode = SEATALK_MODE_AUTO;
            submode = SEATALK_SUBMODE_STANDBY;
            break;
        case MODE_WIND_AWA:
        case MODE_WIND_TWA:
        case MODE_VMG_UP:
        case MODE_VMG_DOWN:
            mode = SEATALK_MODE_STANDBY;
            submode = SEATALK_SUBMODE_WIND;
            break;
        default:
            break;
    }

    msg.AddByte(mode);
    msg.AddByte(0x00);  // mode high byte
    msg.AddByte(submode);
    msg.AddByte(0x00);  // submode high byte
    msg.AddByte(0x00);  // pilot mode data
    msg.AddByte(0xFF);  // reserved

    n2k_send_msg(msg);
}

static void send_pilot_heading(const AppState& state) {
    tN2kMsg msg;
    msg.Init(7, 65359, 28);
    msg.AddByte(RAYMARINE_MFR_LO);
    msg.AddByte(RAYMARINE_MFR_HI);
    msg.AddByte(heading_sid++);
    msg.AddByte(0xFF);  // heading true N/A
    msg.AddByte(0xFF);

    // Heading magnetic in radians * 10000
    float heading_rad = state.heading * (M_PI / 180.0f);
    uint16_t heading_raw = (uint16_t)(heading_rad * 10000.0f);
    msg.AddByte(heading_raw & 0xFF);
    msg.AddByte((heading_raw >> 8) & 0xFF);
    msg.AddByte(0xFF);  // reserved

    n2k_send_msg(msg);
}

static void send_pilot_locked_heading(const AppState& state) {
    if (state.pilot_mode == MODE_STANDBY) return;
    if (state.pilot_mode != MODE_COMPASS) return;  // only for compass mode

    tN2kMsg msg;
    msg.Init(7, 65360, 28);
    msg.AddByte(RAYMARINE_MFR_LO);
    msg.AddByte(RAYMARINE_MFR_HI);
    msg.AddByte(heading_sid);

    // Target heading true: N/A
    msg.AddByte(0xFF);
    msg.AddByte(0xFF);

    // Target heading magnetic in radians * 10000
    float target_rad = state.target_value * (M_PI / 180.0f);
    uint16_t target_raw = (uint16_t)(target_rad * 10000.0f);
    msg.AddByte(target_raw & 0xFF);
    msg.AddByte((target_raw >> 8) & 0xFF);
    msg.AddByte(0xFF);  // reserved

    n2k_send_msg(msg);
}

static void send_pilot_wind_datum(const AppState& state) {
    // Only send in wind modes
    if (state.pilot_mode != MODE_WIND_AWA && state.pilot_mode != MODE_WIND_TWA &&
        state.pilot_mode != MODE_VMG_UP && state.pilot_mode != MODE_VMG_DOWN)
        return;

    tN2kMsg msg;
    msg.Init(7, 65345, 28);
    msg.AddByte(RAYMARINE_MFR_LO);
    msg.AddByte(RAYMARINE_MFR_HI);

    // Wind datum (target wind angle) in radians * 10000
    // Convert signed degrees to 0-360 range first
    float wind_deg = state.target_value;
    if (wind_deg < 0) wind_deg += 360.0f;
    float wind_rad = wind_deg * (M_PI / 180.0f);
    uint16_t wind_raw = (uint16_t)(wind_rad * 10000.0f);
    msg.AddByte(wind_raw & 0xFF);
    msg.AddByte((wind_raw >> 8) & 0xFF);

    // Rolling average wind angle (use current AWA)
    float awa_deg = state.awa;
    if (awa_deg < 0) awa_deg += 360.0f;
    float awa_rad = awa_deg * (M_PI / 180.0f);
    uint16_t awa_raw = (uint16_t)(awa_rad * 10000.0f);
    msg.AddByte(awa_raw & 0xFF);
    msg.AddByte((awa_raw >> 8) & 0xFF);

    msg.AddByte(0xFF);  // reserved
    msg.AddByte(0xFF);

    n2k_send_msg(msg);
}

void seatalk_send(const AppState& state) {
    uint32_t now = millis();

    // PGN 65379 Pilot Mode — 1 Hz
    if (now - last_pilot_mode_ms >= 1000) {
        last_pilot_mode_ms = now;
        send_pilot_mode(state);
    }

    // PGN 65359 Pilot Heading — 5 Hz (every call at N2K_SEND_RATE_HZ=5)
    if (now - last_pilot_heading_ms >= 200) {
        last_pilot_heading_ms = now;
        send_pilot_heading(state);
    }

    // PGN 65360 Locked Heading — 2 Hz
    if (now - last_pilot_locked_ms >= 500) {
        last_pilot_locked_ms = now;
        send_pilot_locked_heading(state);
    }

    // PGN 65345 Wind Datum — 2 Hz
    if (now - last_pilot_wind_ms >= 500) {
        last_pilot_wind_ms = now;
        send_pilot_wind_datum(state);
    }
}

#endif // !NATIVE_BUILD || HAL_SIM
