# Raymarine p70 Autopilot Head Integration

## System Architecture

The Raymarine Evolution autopilot system:
- **p70/p70s** — Control head (user interface, buttons, display)
- **EV-1** — Sensor core (heading/attitude, course computer) — source address 205 (0xCD)
- **ACU-200/ACU-400** — Actuator control unit (drives rudder) — source address 182 (0xB6)

All communicate over **SeaTalkNG** (electrically NMEA 2000, different connectors).
Manufacturer code: **1851** (0x073B). Industry code: **Marine (4)**.

Our ESP32 autopilot replaces the EV-1 + ACU combination. The p70 remains as the
control head and display.

## PGN Message Reference

### Messages the p70 Transmits

#### PGN 126720 (0x1EF00) — Seatalk1 Keystroke Commands
Fast-packet, 22 bytes. Sent when +1/-1/+10/-10 buttons are pressed.

```
Bytes: 3b 9f f0 81 86 21 [CMD_HI] [CMD_LO] ff ff ff ff ff c1 c2 cd 66 80 d3 42 b1 c8
```

Keystroke codes (bytes 6-7):

| Code (hi,lo) | Hex    | Function |
|--------------|--------|----------|
| 0x07, 0xF8   | 0x07F8 | +1 degree |
| 0x05, 0xFA   | 0x05FA | -1 degree |
| 0x08, 0xF7   | 0x08F7 | +10 degrees |
| 0x06, 0xF9   | 0x06F9 | -10 degrees |
| 0x22, 0xDD   | 0x22DD | Tack starboard (+1 and +10 together) |
| 0x21, 0xDE   | 0x21DE | Tack port (-1 and -10 together) |

Identification: bytes 0-1 = 0x3B9F (Raymarine manufacturer LE), byte 2 = 0xF0,
bytes 3-5 = 0x81 0x86 0x21.

#### PGN 126208 (0x1ED00) — NMEA Command Group Function (Mode Changes)
Fast-packet. Used to set pilot mode and target heading.

**Set Pilot Mode** (17 bytes, writes to PGN 65379):
```
01 63 ff 00 f8 04 01 3b 07 03 04 04 [MODE] [SUBMODE] 05 ff ff
```

| MODE | SUBMODE | Meaning |
|------|---------|---------|
| 0x00 | 0x00    | Standby |
| 0x40 | 0x00    | Auto (compass) |
| 0x00 | 0x01    | Wind/Vane |
| 0x80 | 0x01    | Track |
| 0x81 | 0x01    | No Drift (COG referenced) |

**Set Target Heading** (14 bytes, writes to PGN 65360):
```
01 50 ff 00 f8 03 01 3b 07 03 04 06 [LO] [HI]
```
Bytes 12-13: heading in radians * 10000 as uint16 LE.
Example: 180° = 3.14159 rad * 10000 = 31416 = 0x7AB8 → bytes 0xB8, 0x7A

#### PGN 65374 (0xFF5E) — Keypad Heartbeat
8 bytes, sent periodically (~1 Hz) by the p70.

```
Bytes: [mfr_lo] [mfr_hi|industry] [prop_id] [variant] [status] [reserved x3]
```

The ACU/EV-1 expects this heartbeat. If it stops, the system may raise an alarm
and revert to standby. Our firmware should monitor for this to detect p70 presence.

#### PGN 65371 (0xFF5B) — Keypad Message
8 bytes, physical button press/release events.

```
Bytes: [mfr] [mfr|ind] [prop_id] [first_key] [second_key] [key_states] [encoder_pos] [reserved]
```

#### PGN 65361 (0xFF51) — Silence Alarm
8 bytes, sent to acknowledge/silence alarms.

```
Bytes: e7 64 [alarm_code] [alarm_group] 00 00 00 00
```

### Messages the p70 Receives (Displays)

#### PGN 65379 (0xFF63) — Seatalk: Pilot Mode
8 bytes. Broadcast by the course computer to indicate current pilot state.

```
Bytes: [mfr_lo] [mfr_hi|ind] 01 [pilot_mode_lo] [pilot_mode_hi] [sub_mode_lo] [sub_mode_hi] [pilot_mode_data] [reserved]
```

Pilot mode values (byte 3):

| Value | Mode |
|-------|------|
| 0x00  | Standby |
| 0x40  | Auto (compass heading) |
| 0x00 (sub=0x01) | Wind/Vane |
| 0x80 (sub=0x01) | Track |
| 0x81 (sub=0x01) | No Drift |

Our firmware must transmit this so the p70 displays the correct mode.

#### PGN 65359 (0xFF4F) — Seatalk: Pilot Heading
8 bytes, ~10 Hz. Current vessel heading as seen by the pilot.

```
Bytes: [mfr_lo] [mfr_hi|ind] [SID] ff ff [heading_mag_lo] [heading_mag_hi] ff
```

Heading magnetic: uint16 LE, radians * 10000.

#### PGN 65360 (0xFF50) — Seatalk: Pilot Locked Heading
8 bytes. Target/commanded heading (only sent when pilot is engaged).

```
Bytes: [mfr_lo] [mfr_hi|ind] [SID] [heading_true_lo] [heading_true_hi] [heading_mag_lo] [heading_mag_hi] ff
```

#### PGN 65345 (0xFF41) — Seatalk: Pilot Wind Datum
8 bytes. Locked wind angle when in wind mode.

```
Bytes: [mfr_lo] [mfr_hi|ind] [wind_datum_lo] [wind_datum_hi] [rolling_avg_lo] [rolling_avg_hi] [reserved x2]
```

Wind datum: uint16 LE, radians * 10000.

#### PGN 65288 (0xFF08) — Seatalk: Alarm
8 bytes. System alarms displayed on the p70.

```
Bytes: [mfr_lo] [mfr_hi|ind] [SID] [alarm_status] [alarm_id] [alarm_group] [priority_lo] [priority_hi]
```

### Standard PGNs Also Used

| PGN | Name | Direction | Notes |
|-----|------|-----------|-------|
| 127250 | Vessel Heading | p70 ← instruments | Already handled (RX) |
| 127245 | Rudder | p70 ← ACU/us | Already handled (TX) |
| 127237 | Heading/Track Control | p70 ← us | Already handled (TX) |
| 130306 | Wind Data | p70 ← instruments | Already handled (RX) |
| 129283 | Cross Track Error | p70 ← plotter | Needed for Track mode |
| 129284 | Navigation Data | p70 ← plotter | Needed for Track mode |

## Raymarine Manufacturer Header

All proprietary PGNs (65xxx range) start with the Raymarine manufacturer header:

```cpp
// Little-endian: 0x3B9F = manufacturer 1851 | industry Marine (4)
// Byte 0: 0x3B (lower 8 bits of mfr code)
// Byte 1: 0x9F = (upper 3 bits of mfr: 100) | (reserved: 11) | (industry: 100)
//        = 0b10011111 = 0x9F
#define RAYMARINE_MFR_LO  0x3B
#define RAYMARINE_MFR_HI  0x9F  // includes industry code
```

## Data from N2K Logs

From `/Users/boston/ieb/autopilot/n2klogs/`:

| PGN | Found | Count | Notes |
|-----|-------|-------|-------|
| 65379 (Pilot Mode) | Yes | 134 | All standby (mode=0) |
| 65359 (Pilot Heading) | Yes | 1402 | ~10 Hz from EV-1 |
| 65360 (Locked Heading) | No | — | Pilot was in standby |
| 65345 (Wind Datum) | No | — | Never in wind mode |
| 65371 (Keypad Msg) | No | — | No p70 on bus during capture |
| 65374 (Keypad Heartbeat) | No | — | No p70 on bus during capture |
| 65288 (Alarm) | Yes | 4 | From src 1 and 2 |
| 126720 (Seatalk1) | Yes | 129k | Bulk of traffic |
| 126208 (Command) | Yes | 878 | Inter-device requests |
| 127237 (HTC) | Yes | 2174 | From ACU (src 182) |

## Implementation Plan

### RX (receive from p70)
1. Parse PGN 126720 for keystroke commands → adjust target heading ±1/±10°
2. Parse PGN 126208 for mode change commands → set pilot mode
3. Monitor PGN 65374 for p70 heartbeat → track p70 presence

### TX (send to p70 for display)
1. PGN 65379 at ~1 Hz — current pilot mode
2. PGN 65359 at ~10 Hz — current heading
3. PGN 65360 at ~2 Hz — target heading (when engaged)
4. PGN 65345 at ~2 Hz — wind datum (when in wind mode)

### References
- [ESP32-Evo-Remote-Pilot-NMEA2000](https://github.com/AK-Homberger/ESP32-Evo-Remote-Pilot-NMEA2000)
- [RaymarineAutoPilot](https://github.com/MENIER/RaymarineAutoPilot)
- [matztam/raymarine-evo-pilot-remote](https://github.com/matztam/raymarine-evo-pilot-remote)
- [canboat PGN database](https://canboat.github.io/canboat/canboat.html)
