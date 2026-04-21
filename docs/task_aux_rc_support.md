# Task: AUX RC Support via MSP2_INAV_SET_AUX_RC

## Background

INAV PR #11482 (by b14ckyy) introduces `MSP2_INAV_SET_AUX_RC` (0x2230), a new bandwidth-efficient MSP command for updating auxiliary RC channels (CH13–CH32) without affecting the primary stick channels. This document captures the current BulletGCSS implementation, what the new command enables, and the constraints that must be resolved before a migration plan can be finalized.

## Current Implementation

BulletGCSS uses `MSP_SET_RAW_RC` (code 200, MSPv1) to activate flight modes remotely. The mechanism works by replacing the entire RC channel array on every 160 ms telemetry cycle.

### Flow

1. **`msp_get_mode_ranges()`** — runs once at startup. Reads `MSP_MODE_RANGES` (34) from the FC to discover which RC channel and PWM range activates each commandable flight mode (RTH, AltHold, Cruise, WP, Beeper, PosHold). Stores results in `cmdModes[].mode->range`.

2. **`msp_get_override_channels()`** — runs once after mode ranges are known. Reads the `msp_override_channels` FC setting, ORs in any channels needed by BulletGCSS, and writes the updated mask back to the FC. This is required by INAV to whitelist which channels `MSP_SET_RAW_RC` is allowed to override.

3. **`msp_get_rc()`** — runs every 160 ms cycle. Reads current RC channel values from the FC via `MSP_RC` (105) into `rcChannels[]`. When MSP RC Override is active, the FC echoes back the previously sent override values rather than the real radio values.

4. **`msp_send_rc_override()`** — runs every 160 ms cycle. Builds a modified copy of `rcChannels[]`, patches in the active mode values (or safe neutral values for inactive modes), and sends the full array via `MSP_SET_RAW_RC`.

### Complexity Required by This Approach

- **Full channel array every cycle** — `MSP_SET_RAW_RC` always starts at CH1 and covers all N channels. The firmware must send stick values (CH1–CH4) unchanged alongside the patched aux channels.
- **`msp_get_rc()` on every cycle** — needed solely to get current stick positions to pass through untouched.
- **Gap-finder (`findSafeOffValue()`)** — when deactivating a mode, the firmware cannot simply stop sending a channel. Because all channels are always sent, it must explicitly write a neutral value. If another mode shares the same channel, the neutral value must fall in a gap between all activation ranges on that channel to avoid inadvertently triggering another mode.
- **`MSP RC OVERRIDE` flight mode required** — INAV only applies `MSP_SET_RAW_RC` overrides when this flight mode is active on the FC.
- **`msp_override_channels` setting** — must be configured in INAV RAM to whitelist channels; BulletGCSS auto-configures this at startup.

### Relevant Source Locations

| Item | File | Lines |
|---|---|---|
| `rcChannels[]` / `rcChannelCount` | `ESP32-Modem.cpp` | 170–173 |
| `cmdModes[]` table | `ESP32-Modem.cpp` | 198–206 |
| `msp_get_mode_ranges()` | `ESP32-Modem.cpp` | 1517–1572 |
| `msp_get_override_channels()` | `ESP32-Modem.cpp` | 1621–1668 |
| `msp_get_rc()` | `ESP32-Modem.cpp` | 1670–1694 |
| `findSafeOffValue()` | `ESP32-Modem.cpp` | 1723–~ |
| `msp_send_rc_override()` | `ESP32-Modem.cpp` | 1779–1852 |
| `MSP_SET_RAW_RC` constant | `msp_library.h` | 52 |

## What MSP2_INAV_SET_AUX_RC Enables

The new command targets only CH13–CH32 as a post-RX overlay applied after MSP RC Override but before failsafe. Key differences:

- **No `MSP RC OVERRIDE` flight mode required** — the overlay is always applied regardless of active flight modes.
- **No `msp_override_channels` configuration** — that setting is specific to `MSP_SET_RAW_RC` and does not apply here.
- **No need to read RC values first** — the command never touches CH1–CH12, so stick pass-through is not a concern.
- **No gap-finder needed for shared channels** — sending value `0` for a channel means "no update / skip", though a safe neutral value still needs to be sent when explicitly deactivating a mode.
- **Bandwidth reduction ~65%** — the payload covers only the target aux channels in configurable resolution (2/4/8/16-bit per channel).

## Key Constraint

The new command only covers **CH13–CH32** (aux index 8 and above). The most common INAV setup maps flight mode switches to **CH5–CH12**. If a user's mode ranges are on CH5–CH12, `MSP2_INAV_SET_AUX_RC` cannot reach them and `MSP_SET_RAW_RC` would still be required for those channels.

This is the central question that must be resolved before implementation:
- **Hybrid approach**: use `MSP2_INAV_SET_AUX_RC` for CH13+ and keep `MSP_SET_RAW_RC` for CH5–CH12.
- **Full migration**: drop `MSP_SET_RAW_RC` entirely and require users to assign flight mode switches to CH13+.
- **Keep as-is**: continue using `MSP_SET_RAW_RC` until INAV extends the new command's channel range.

## Proposed New Approach: Self-Configured Dedicated Channel

### Problem with the Current Detection Approach

The current implementation reads the user's existing mode range configuration and relies on those ranges being present. If the user has not configured a range for a given mode (e.g. no RTH switch set up in the INAV Configurator), BulletGCSS cannot trigger that mode at all. This creates a hard dependency on the user's AUX channel layout and means BulletGCSS capability varies unpredictably between aircraft.

### Solution: BulletGCSS Owns Its Channel

Instead of discovering and reusing the user's configuration, BulletGCSS firmware will:

1. **Scan all existing mode ranges** — read all 40 condition slots via `MSP_MODE_RANGES` (34) and record which aux channels are already in use (have any valid activation range).
2. **Find a free aux channel** — starting from aux channel 20 (CH25, 0-based RC index 24), find the first channel that has no existing mode range mapped to it. This keeps BulletGCSS out of the range users typically use for manual switches (CH5–CH24).
3. **Claim 6 free condition slots** — identify 6 unused entries in the 40-slot condition table (entries where `startStep >= endStep` are considered empty).
4. **Write the BulletGCSS range map** — use `MSP_SET_MODE_RANGE` (35) to write one condition per commandable mode into the 6 claimed slots, all targeting the chosen dedicated channel.
5. **Control the channel via `MSP2_INAV_SET_AUX_RC`** — on every cycle, send the single channel's PWM value to activate the desired mode (or the neutral range to activate none).

This approach makes BulletGCSS fully self-contained: no manual INAV Configurator setup is needed for remote mode control.

### Dedicated Channel Layout

The dedicated channel is divided into 10 equal ranges of 100 µs each, covering 1000–2000 µs. INAV steps are 25 µs each (step 0 = 900 µs, step 4 = 1000 µs).

| Range index | PWM (µs) | Steps (start–end) | Mode | Condition slot usage |
|---|---|---|---|---|
| 0 | 1000–1100 | 4–8 | *(neutral — no mode active)* | — no entry written — |
| 1 | 1100–1200 | 8–12 | RTH | 1 slot |
| 2 | 1200–1300 | 12–16 | AltHold | 1 slot |
| 3 | 1300–1400 | 16–20 | Cruise | 1 slot |
| 4 | 1400–1500 | 20–24 | WP Mission | 1 slot |
| 5 | 1500–1600 | 24–28 | Beeper | 1 slot |
| 6 | 1600–1700 | 28–32 | PosHold | 1 slot |
| 7 | 1700–1800 | 32–36 | *(spare)* | — no entry written — |
| 8 | 1800–1900 | 36–40 | *(spare)* | — no entry written — |
| 9 | 1900–2000 | 40–44 | *(spare)* | — no entry written — |

The neutral range (1000–1100 µs, center 1050 µs) has no condition entry. When no mode should be active, BulletGCSS sets the channel to 1050 µs; the FC sees no activation condition for that value and keeps all modes off.

On each cycle, BulletGCSS sets the dedicated channel to the center PWM of the active mode's range, or to 1050 µs if no mode is active. Only one mode can be active at a time on this channel, which is acceptable because INAV navigation modes (RTH, Cruise, WP, AltHold) are mutually exclusive by design.

### `MSP_SET_MODE_RANGE` Payload

5 bytes per call: `[slotIndex (u8), permanentId (u8), auxChannelIndex (u8), startStep (u8), endStep (u8)]`

- `slotIndex`: 0–39 (one of the 5 free slots identified in step 3 above)
- `permanentId`: mode permanent ID (RTH=10, AltHold=3, Cruise=53, WP=28, Beeper=13, PosHold=11; see `msp_library.h`)
- `auxChannelIndex`: 0-based aux index of the chosen dedicated channel (e.g. 20 for CH25)
- `startStep` / `endStep`: from the table above

The 3 spare ranges (1700–2000 µs) are reserved for future use and **no condition entries are written for them**. Only 6 slots are needed — one per commandable mode.

**EEPROM write policy**: these writes go to INAV RAM only — `MSP_EEPROM_WRITE` is intentionally not called. This avoids permanently modifying the user's saved FC configuration. Whether INAV applies in-RAM mode range changes immediately without a save is to be confirmed by testing. If it does not, an EEPROM write will be required and the reconnect detection logic below becomes even more important.

**Note on concurrent modes**: only one mode can be active at a time on the dedicated channel, which is acceptable because INAV navigation modes (RTH, Cruise, WP, AltHold) are mutually exclusive by design. Beeper is independent but shares the channel; simultaneous Beeper + nav mode is not possible with this single-channel design and is accepted as a known limitation.

### Startup Sequence: Detect Before Configure

The ESP32 has a watchdog routine that can reboot the board mid-session. On reconnect to the FC, the mode ranges written in the previous session may still be present in INAV RAM (if no FC reboot occurred). Writing a second dedicated channel on every reconnect would exhaust the available condition slots over time and collide with the user's own configuration.

Therefore, the startup sequence must **detect an existing BulletGCSS channel before writing a new one**:

1. **Read all 40 mode condition slots** via `MSP_MODE_RANGES`.
2. **Scan for an existing BulletGCSS channel** — check whether all 6 commandable modes (RTH, AltHold, Cruise, WP, Beeper, PosHold) already have condition entries on the **same** aux channel with the **exact expected step values** from the layout table above. If all 6 are found on the same channel with correct steps, the channel is already configured; skip to step 6.
3. **Find a free aux channel** ≥ 20 (CH25+) — a channel with no existing condition entries.
4. **Find 6 free condition slots** — entries where `startStep >= endStep` (INAV's representation of an empty/disabled condition).
5. **Write 6 mode range entries** via `MSP_SET_MODE_RANGE`, one per commandable mode, all targeting the chosen channel.
6. **Store the dedicated `auxChannelIndex`** and per-mode center PWM values for use by the send loop.

This detection ensures that an ESP32 reboot followed by FC reconnect reuses the existing channel rather than creating a duplicate. An INAV reboot is a catastrophic event outside BulletGCSS's responsibility; in that case the ranges are gone and BulletGCSS will simply write them fresh (the scan in step 2 will find nothing, so a new channel is claimed).

### Send Loop Change

`msp_send_rc_override()` is replaced by a simpler function:

- Determine the active mode (if any) from `cmdModes[].mode->active`
- Look up the center PWM for that mode from the layout table (or 1050 µs for neutral)
- Send a single-channel `MSP2_INAV_SET_AUX_RC` packet targeting the dedicated channel index

Because `MSP2_INAV_SET_AUX_RC` does not require `MSP RC OVERRIDE` flight mode, and only one channel value is ever sent, the following can all be removed:
- `msp_get_rc()` (no longer needed — stick values are not touched)
- `rcChannels[]` / `rcChannelCount` arrays
- `findSafeOffValue()` (no shared-channel conflict possible on a BulletGCSS-owned channel)
- `mspOverrideChannelsMask` / `msp_get_override_channels()` / `msp_set_setting_u32()`
- The `mspRcOverride` guard in the send loop (no longer a prerequisite)
