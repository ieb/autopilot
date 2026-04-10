# p70 → EV-1 Control Protocol via PGN 126208

Decoded from CAN bus captures of a Raymarine p70 (source 0) commanding an EV-1 (source 205) via PGN 126208 Command Group. The p70 does NOT use PGN 126720 keystroke codes — all commands are structured PGN 126208 messages.

Source data: `n2klogs/logged/*.bin` (2026-03-14 dock recording, ~1.4M frames).

## PGN 126208 Command Group overview

PGN 126208 is a standard NMEA 2000 meta-PGN with three function codes:

| FC | Name | Direction | Purpose |
|----|------|-----------|---------|
| 0x00 | Request | p70 → EV-1 | Poll for status (PGN 126720 telemetry) |
| 0x01 | Command | p70 → EV-1 | Set mode, heading, wind datum |
| 0x02 | Acknowledge | EV-1 → p70 | Confirm receipt of command |

All commands from the p70 are sent via FC=0x01, targeting a specific proprietary PGN. The EV-1 replies with FC=0x02 within 2–34ms.

## Common byte structure

All p70 commands share this prefix structure:

```
Byte  Field                   Value/Meaning
────  ─────                   ─────────────
[0]   Function Code           0x01 (Command)
[1]   Target PGN byte 0      PGN & 0xFF
[2]   Target PGN byte 1      (PGN >> 8) & 0xFF
[3]   Target PGN byte 2      (PGN >> 16) & 0xFF
[4]   Priority/Reserved       0xF8
[5]   Number of parameters    varies (3 or 4)
[6]   Param 1 index           0x01
[7]   Param 1 value: mfr lo   0x3B  ← Raymarine manufacturer code (0x073B = 1851)
[8]   Param 1 value: mfr hi   0x07
[9]   Param 2 index           0x03
[10]  Param 2 value: ???      0x04
[11]  Param 3 index           0x04 or 0x06
[12+] Param 3 value           command-specific payload (see below)
```

Bytes [6]–[10] are always `01 3b 07 03 04` — this is the Raymarine manufacturer authentication header. The actual command data starts at byte [11].

## Angle encoding

All angles (heading, wind datum) are encoded as:

```
uint16 LE = radians * 10000
```

To encode: `value = int(math.radians(degrees) * 10000)` stored little-endian.
To decode: `degrees = math.degrees(uint16_le / 10000.0)`

Range: 0–360 degrees maps to 0–62832 (0x0000–0xF5A0).

---

## CMD → PGN 65379: Pilot Mode

Sets the autopilot mode (Standby, Auto/Compass, Wind AWA, Wind TWA, Track).

### Byte layout (17 bytes)

```
Byte  Field                   Notes
────  ─────                   ─────
[0]   FC                      0x01
[1-3] Target PGN              0x63 0xFF 0x00  (65379 LE, 3 bytes)
[4]   Priority                0xF8
[5]   Num params              0x04
[6-8] Param 1 (mfr)          0x01 0x3B 0x07
[9-10] Param 2               0x03 0x04
[11]  Param 3 index           0x04
[12]  Mode                    see table below
[13]  Mode qualifier          see table below
[14]  Param 4 index           0x05
[15]  Submode hi              see table below
[16]  Submode lo              see table below
```

### Mode byte [12] and qualifier byte [13]

| Mode | [12] | [13] | Description |
|------|------|------|-------------|
| STANDBY | 0x00 | 0x00 | Disengage autopilot |
| AUTO (Compass) | 0x40 | 0x00 | Steer to locked heading |
| WIND | 0x00 | 0x01 | Steer to wind angle (AWA or TWA) |
| TRACK | 0x80 | 0x01 | Steer to GPS track/waypoint |
| NO DRIFT | 0x81 | 0x00 | Compass + drift compensation (not observed in logs) |

### Submode bytes [15]–[16] (wind mode only)

| [15] | [16] | Meaning |
|------|------|---------|
| 0xFF | 0xFF | Keep current submode / default (used on initial wind mode entry) |
| 0x03 | 0x00 | AWA — Apparent Wind Angle |
| 0x04 | 0x00 | TWA — True Wind Angle |

For non-wind modes (STANDBY, AUTO, TRACK), bytes [15]–[16] are always `0xFF 0xFF`.

### Observed sequences

Entering wind mode from standby:
```
STANDBY → WIND [0xFF 0xFF]     enters wind, keeps last submode (AWA or TWA)
```

Switching between AWA and TWA while already in wind mode:
```
WIND [0xFF 0xFF] → WIND [0x04 0x00]    switch to TWA
WIND [0x04 0x00] → WIND [0x03 0x00]    switch to AWA
WIND [0x03 0x00] → WIND [0x04 0x00]    switch back to TWA
```

The p70 sends a new 65379 command each time the user toggles between AWA and TWA. The EV-1 ACKs each one.

### Example: enter Auto (Compass) mode

```
01 63 ff 00 f8 04 01 3b 07 03 04 04 40 00 05 ff ff
```

### Example: enter Wind AWA mode

```
01 63 ff 00 f8 04 01 3b 07 03 04 04 00 01 05 03 00
```

### Example: Standby

```
01 63 ff 00 f8 04 01 3b 07 03 04 04 00 00 05 ff ff
```

---

## CMD → PGN 65360: Locked Heading

Sets the target heading in Auto (Compass) mode. The p70 computes the absolute heading and sends it — not a delta.

### Byte layout (14 bytes)

```
Byte  Field                   Notes
────  ─────                   ─────
[0]   FC                      0x01
[1-3] Target PGN              0x50 0xFF 0x00  (65360 LE, 3 bytes)
[4]   Priority                0xF8
[5]   Num params              0x03
[6-8] Param 1 (mfr)          0x01 0x3B 0x07
[9-10] Param 2               0x03 0x04
[11]  Param 3 index           0x06
[12]  Heading lo              uint16 LE = radians * 10000
[13]  Heading hi
```

Note: param 3 index is `0x06` (not `0x04` as in mode/wind commands).

### Usage

Only sent during Auto (Compass) mode. When the user presses +1 or +10 on the p70, it reads the current heading, adds the offset, and sends the new absolute heading.

No heading commands were observed during wind mode — wind mode uses PGN 65345 exclusively.

### Example: set heading to 12.2 degrees

```
heading_rad_x10000 = int(math.radians(12.2) * 10000) = 0x084E

01 50 ff 00 f8 03 01 3b 07 03 04 06 4e 08
```

### Example: set heading to 41.2 degrees

```
heading_rad_x10000 = int(math.radians(41.2) * 10000) = 0x1C12

01 50 ff 00 f8 03 01 3b 07 03 04 06 12 1c
```

---

## CMD → PGN 65345: Wind Datum

Sets the target wind angle in Wind mode (AWA or TWA). Like heading, the p70 sends absolute angles, not deltas.

### Byte layout (14 bytes)

```
Byte  Field                   Notes
────  ─────                   ─────
[0]   FC                      0x01
[1-3] Target PGN              0x41 0xFF 0x00  (65345 LE, 3 bytes)
[4]   Priority                0xF8
[5]   Num params              0x03
[6-8] Param 1 (mfr)          0x01 0x3B 0x07
[9-10] Param 2               0x03 0x04
[11]  Param 3 index           0x04
[12]  Wind angle lo           uint16 LE = radians * 10000
[13]  Wind angle hi
```

### Usage

Only sent during Wind mode. The p70 sends absolute wind angle targets. In the logs, typical adjustments were +1 degree or +10 degree increments (the p70 computes the new absolute value).

The same PGN is used for both AWA and TWA targets — the current submode (set via PGN 65379) determines interpretation.

### Example: set wind datum to 300.9 degrees

```
wind_rad_x10000 = int(math.radians(300.9) * 10000) = 0xCD2D

01 41 ff 00 f8 03 01 3b 07 03 04 04 2d cd
```

---

## Acknowledgement format (EV-1 → p70)

The EV-1 sends a PGN 126208 FC=0x02 ACK for every command, within 2–34ms.

### Byte layout (8–9 bytes)

```
Byte  Field                   Notes
────  ─────                   ─────
[0]   FC                      0x02 (Acknowledge)
[1-3] Target PGN              same PGN as the command being ACKed (LE, 3 bytes)
[4]   PGN error code          0x00 = OK
[5]   Num params from cmd     matches the command's param count
[6]   Num param errors        0x00 (no errors)
[7+]  Param error codes       0x00 per param, or 0xF0 (padding)
```

PGN error codes:
- 0x00 = OK
- 0x01 = PGN not supported
- 0x02 = PGN temporarily unavailable
- 0x03 = Access denied
- 0x04 = Request not supported

All 86 ACKs observed in the logs had error code 0x00 (OK).

### ACK examples

```
ACK for 65379 (mode):        02 63 ff 00 00 04 00 00
ACK for 65360 (heading):     02 50 ff 00 00 03 00 f0
ACK for 65345 (wind datum):  02 41 ff 00 00 03 00 f0
```

---

## Implementation checklist

To accept p70 commands (replacing the EV-1 role):

1. **Listen for PGN 126208 FC=0x01** from source 0 (p70)
2. **Parse the target PGN** from bytes [1]–[3]
3. **Validate Raymarine header** bytes [7]–[8] = `0x3B 0x07` (manufacturer 1851)
4. **Dispatch by target PGN**:
   - 65379 → mode change (read bytes [12]–[13] for mode, [15]–[16] for wind submode)
   - 65360 → heading set (decode uint16 LE from bytes [12]–[13])
   - 65345 → wind datum set (decode uint16 LE from bytes [12]–[13])
5. **Send ACK** (PGN 126208 FC=0x02) within ~34ms, matching the target PGN and param count

To command the pilot (taking the p70 role):

1. **Build the 126208 command** using the byte layouts above
2. **Send as fast-packet** on PGN 126208
3. **Wait for ACK** (FC=0x02) — check PGN error code is 0x00

### Mode change sequence

```
Enter Auto:     CMD 65379 [mode=0x40, qual=0x00, sub=0xFF 0xFF]
Enter Wind AWA: CMD 65379 [mode=0x00, qual=0x01, sub=0x03 0x00]
Enter Wind TWA: CMD 65379 [mode=0x00, qual=0x01, sub=0x04 0x00]
Enter Wind:     CMD 65379 [mode=0x00, qual=0x01, sub=0xFF 0xFF]  (keep last submode)
Enter Track:    CMD 65379 [mode=0x80, qual=0x01, sub=0xFF 0xFF]
Standby:        CMD 65379 [mode=0x00, qual=0x00, sub=0xFF 0xFF]
```

### Adjusting targets

```
In Auto mode:   CMD 65360 [heading = radians * 10000, uint16 LE]
In Wind mode:   CMD 65345 [wind_angle = radians * 10000, uint16 LE]
```

---

## What the p70 also needs from the pilot

Beyond accepting commands, the p70 polls for status and expects certain broadcasts. See `docs/bus_analysis_20260215.txt` for full details. Summary:

| Requirement | PGN | Rate | Notes |
|-------------|-----|------|-------|
| Pilot status heartbeat | 126720 sig `f081ae02` | ~1 Hz | Byte[6] = mode (0x00/0x01/0x03) |
| Response to polling | 126720 sigs `6c1a`/`6c23` | On request | Constant payloads observed at dock |
| Pilot mode broadcast | 65379 | ~1 Hz | Current mode/submode |
| Pilot heading broadcast | 65359 | ~10 Hz | Current heading reference |
| Locked heading broadcast | 65360 | ~10 Hz | Target heading |
| Wind datum broadcast | 65345 | ~10 Hz | Target wind angle |
