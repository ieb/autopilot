#if !defined(NATIVE_BUILD) || defined(HAL_SIM)

#include "n2k.h"
#include "seatalk.h"
#include "config.h"
#include <math.h>

#ifdef HAL_SIM
#include "hal/NMEA2000_sim.h"
// NMEA2000 instance defined in NMEA2000_sim.cpp
#else
#include <NMEA2000_esp32.h>
static tNMEA2000_esp32 NMEA2000((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX);
#endif

#include <N2kMessages.h>

// Forward declarations
static void handle_msg(const tN2kMsg& msg);
static void parse_wind(const tN2kMsg& msg, AppState& state);
static void parse_stw(const tN2kMsg& msg, AppState& state);
static void parse_cog_sog(const tN2kMsg& msg, AppState& state);
static void parse_heading(const tN2kMsg& msg, AppState& state);

// Pointer to current state for message handler callback
static AppState* g_state = nullptr;

// Message handler wrapper (ttlappalainen callback signature)
static void msg_handler(const tN2kMsg& msg) {
    if (g_state) handle_msg(msg);
}

void n2k_init() {
    NMEA2000.SetProductInformation(
        "00001", 100, "Pogo Autopilot", "1.0", "1.0"
    );
    NMEA2000.SetDeviceInformation(1, 150, 40);  // Steering gear
    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode);
    NMEA2000.SetMsgHandler(msg_handler);

    // Transmit PGNs: standard + Seatalk proprietary
    const unsigned long txPGNs[] = {
        127245UL, 127237UL,           // Rudder, Heading/Track Control
        65379UL, 65359UL, 65360UL, 65345UL,  // Seatalk Pilot Mode/Heading/Locked/Wind
        126720UL, 126208UL,           // Pilot status heartbeat + p70 poll responses + ACK
        0
    };
    NMEA2000.ExtendTransmitMessages(txPGNs);

    NMEA2000.Open();

    seatalk_init();
}

void n2k_update(AppState& state) {
    g_state = &state;
    NMEA2000.ParseMessages();
}

// Public: send a message via the NMEA2000 instance (used by seatalk.cpp)
void n2k_send_msg(tN2kMsg& msg) {
    NMEA2000.SendMsg(msg);
}

void n2k_send(const AppState& state) {
    // PGN 127245 — Rudder
    tN2kMsg rudder_msg;
    SetN2kRudder(rudder_msg,
                 DegToRad(state.rudder_actual * 25.0),  // actual position
                 0,                                       // instance
                 N2kRDO_MoveToStarboard,                 // direction order
                 DegToRad(state.rudder_target * 25.0));  // commanded position
    NMEA2000.SendMsg(rudder_msg);

    // PGN 127237 — Heading/Track Control
    tN2kMsg htc_msg;
    tN2kOnOff rudder_limit = N2kOnOff_Off;
    tN2kOnOff off_heading = N2kOnOff_Off;
    tN2kOnOff off_track = N2kOnOff_Off;
    tN2kOnOff override_flag = N2kOnOff_Off;

    tN2kSteeringMode steer_mode;
    tN2kTurnMode turn_mode = N2kTM_RudderLimitControlled;

    switch (state.pilot_mode) {
        case MODE_COMPASS:   steer_mode = N2kSM_HeadingControl; break;
        case MODE_WIND_AWA:
        case MODE_WIND_TWA:
        case MODE_VMG_UP:
        case MODE_VMG_DOWN:  steer_mode = N2kSM_HeadingControl; break;
        default:             steer_mode = N2kSM_Unavailable; break;
    }

    tN2kHeadingReference ref = N2khr_magnetic;

    SetN2kHeadingTrackControl(
        htc_msg,
        rudder_limit,
        off_heading,
        off_track,
        override_flag,
        steer_mode,
        turn_mode,
        ref,
        N2kRDO_MoveToStarboard,
        DegToRad(state.rudder_actual * 25.0),  // commanded rudder angle
        DegToRad(state.target_value),           // heading to steer
        N2kDoubleNA,                            // track
        N2kDoubleNA,                            // rudder limit
        N2kDoubleNA,                            // off heading limit
        N2kDoubleNA,                            // radius of turn
        N2kDoubleNA,                            // rate of turn
        N2kDoubleNA,                            // off track limit
        DegToRad(state.heading)                 // vessel heading
    );
    NMEA2000.SendMsg(htc_msg);

    // Seatalk proprietary PGNs for p70 display
    seatalk_send(state);
}

static void handle_msg(const tN2kMsg& msg) {
    // Try Seatalk handler first for proprietary PGNs
    if (seatalk_handle_msg(msg.Data, msg.DataLen, msg.PGN, msg.Source, *g_state))
        return;

    switch (msg.PGN) {
        case 130306: parse_wind(msg, *g_state); break;
        case 128259: parse_stw(msg, *g_state); break;
        case 129026: parse_cog_sog(msg, *g_state); break;
        case 127250: parse_heading(msg, *g_state); break;
    }
}

static void parse_wind(const tN2kMsg& msg, AppState& state) {
    unsigned char sid;
    double wind_speed, wind_angle;
    tN2kWindReference ref;

    if (ParseN2kWindSpeed(msg, sid, wind_speed, wind_angle, ref)) {
        float speed_kts = wind_speed / 0.514444;  // m/s to kts
        float angle_deg = RadToDeg(wind_angle);

        if (ref == N2kWind_Apparent) {
            // Convert 0-360 to signed: positive = starboard
            if (angle_deg > 180.0f) angle_deg -= 360.0f;
            state.awa = angle_deg;
            state.aws = speed_kts;

            // Compute true wind from apparent wind + STW (wind triangle)
            // Real N2K buses only carry apparent wind — no True_boat PGN.
            // V_true = V_apparent - V_boat
            if (state.stw > 0.1f) {
                float awa_rad = angle_deg * (M_PI / 180.0f);
                float wx = speed_kts * cosf(awa_rad) - state.stw;
                float wy = speed_kts * sinf(awa_rad);
                state.tws = sqrtf(wx * wx + wy * wy);
                state.twa = atan2f(wy, wx) * (180.0f / M_PI);
            }
        } else if (ref == N2kWind_True_boat) {
            // Accept true wind if a processor is sending it (fallback)
            if (angle_deg > 180.0f) angle_deg -= 360.0f;
            state.twa = angle_deg;
            state.tws = speed_kts;
        }
        state.n2k_wind_ms = millis();
    }
}

static void parse_stw(const tN2kMsg& msg, AppState& state) {
    unsigned char sid;
    double stw, ground_ref;
    tN2kSpeedWaterReferenceType swrt;

    if (ParseN2kBoatSpeed(msg, sid, stw, ground_ref, swrt)) {
        state.stw = stw / 0.514444;  // m/s to kts
        state.n2k_speed_ms = millis();
    }
}

static void parse_cog_sog(const tN2kMsg& msg, AppState& state) {
    unsigned char sid;
    tN2kHeadingReference ref;
    double cog, sog;

    if (ParseN2kCOGSOGRapid(msg, sid, ref, cog, sog)) {
        state.cog = RadToDeg(cog);
        state.sog = sog / 0.514444;  // m/s to kts
        state.n2k_cog_ms = millis();
    }
}

static void parse_heading(const tN2kMsg& msg, AppState& state) {
    unsigned char sid;
    double heading, deviation, variation;
    tN2kHeadingReference ref;

    if (ParseN2kHeading(msg, sid, heading, deviation, variation, ref)) {
        state.n2k_heading = RadToDeg(heading);
        // Snapshot raw IMU heading so imu_read() can compute delta correctly.
        // Must use raw IMU, not fused heading — otherwise when both sources
        // report the same value, the delta is doubled.
        state.imu_heading_at_n2k = state.imu_raw_heading;
        state.n2k_heading_valid = true;
        state.n2k_heading_ms = millis();
    }
}

#endif // !NATIVE_BUILD || HAL_SIM
