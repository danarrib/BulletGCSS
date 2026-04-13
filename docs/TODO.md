# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities. Items are grouped by category and re-enumerated after each cleanup pass.

---

## New Commands

### C3. Set Altitude (target altitude for Altitude Hold / Cruise mode)

Allow the operator to command the aircraft to climb or descend to a specific altitude while Altitude Hold or Cruise mode is active.

**Status:** `MSP2_INAV_SET_ALT_TARGET` (code **0x2215 / 8725**) merged into INAV, available on **INAV 10.0.0 or newer**. Our current firmware handler sends the wrong payload and must be updated.

**Payload mismatch:** Our implementation sends a bare `int32_t` (4 bytes). The actual INAV command expects two fields:
1. `altitudeDatum` — `uint8_t` — altitude reference (`NAV_WP_TAKEOFF_DATUM`, MSL, or terrain; MSL requires valid GPS origin, terrain is rejected)
2. `altitudeTarget` — `int32_t` — target altitude in centimetres relative to the chosen datum

The command is rejected by INAV unless altitude control is active (NAV or ALTHOLD mode), not landing/emergency landing, altitude estimation is valid, and the datum is supported.

**What needs to be done:**
- **Firmware (`ESP32-Modem.cpp`):** update `cmd:setalt` handler to send a 5-byte payload: `uint8_t datum` + `int32_t altCm`. Use takeoff datum (`NAV_WP_TAKEOFF_DATUM = 0`) as the default; optionally accept a `datum` field in the downlink message.
- **Firmware (`msp_library.h`):** verify or add the `MSP2_INAV_SET_ALT_TARGET` constant (0x2215).
- **UI:** gate the Set Altitude row behind INAV >= 10.0.0 (separate from the existing `extCmdsSupported` check which covers >= 9.0.2).
- **Protocol (`BulletGCSS_protocol.md`):** document the updated `cmd:setalt,alt:<cm>` downlink message.

---

## Completed Items

- **U1** — Fix main menu in landscape orientation ✅ — resolved by Bootstrap 5 migration (offcanvas sidebar scrolls naturally).
- **U2** — Migrate UI to Bootstrap 5 ✅ — completed in v1.7; `basicui.html` now runs on Bootstrap 5.3.8.
- **F1** — Multi-aircraft monitoring ✅ — completed in v1.6.
- **F2** — Waypoint mission planner ✅ — completed in v1.6.
- **C1** — Set Course (Cruise Mode heading) ✅ — uses `MSP2_INAV_SET_CRUISE_HEADING` (0x2223). Firmware and UI implemented.
- **C2** — Jump to Waypoint ✅ — uses `MSP2_INAV_SET_WP_INDEX`. Firmware and UI implemented.
