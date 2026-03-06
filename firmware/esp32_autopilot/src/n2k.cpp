#ifndef NATIVE_BUILD

#include "n2k.h"
#include "config.h"

#include <NMEA2000_esp32.h>
#include <N2kMessages.h>

static tNMEA2000_esp32 NMEA2000((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX);

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

    // Transmit PGNs
    const unsigned long txPGNs[] = {127245UL, 127237UL, 0};
    NMEA2000.ExtendTransmitMessages(txPGNs);

    NMEA2000.Open();
}

void n2k_update(AppState& state) {
    g_state = &state;
    NMEA2000.ParseMessages();
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
}

static void handle_msg(const tN2kMsg& msg) {
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
        } else if (ref == N2kWind_True_boat) {
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
        // BNO055 heading is primary; N2K heading used as timestamp only
        state.n2k_heading_ms = millis();
    }
}

#endif // NATIVE_BUILD
