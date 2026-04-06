# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities. Items are grouped by category and re-enumerated after each cleanup pass.

---

## New Commands

### C1. Set Course command (Cruise Mode heading)
Send a target heading to the aircraft while in Cruise Mode, so the operator can redirect the flight without using the RC transmitter.

**INAV MSP command:** `MSP_SET_HEAD` (code **211**) — payload: one `U16` value in centidegrees (0–36000). Already implemented in INAV; no firmware changes needed on the INAV side.

**What needs to be done:**
- **Firmware (`ESP32-Modem.cpp`):** add a new `cmd:setheading` handler that reads the `heading` field from the downlink message and sends `MSP_SET_HEAD` to the FC.
- **UI (`bsPageScripts.js` / commands panel):** add a heading input field (0–359°) and a Send button to the Commands panel.
- **Protocol (`BulletGCSS_protocol.md`):** document the new `cmd:setheading,heading:<centidegrees>` downlink message.

---

### C2. Jump to Waypoint (skip to WP N during active mission)

No dedicated MSP command exists in INAV for this. The controlling variable is `posControl.activeWaypointIndex` (`int8_t` inside the global `posControl` struct, defined in `navigation_private.h:494`). It is managed entirely by the navigation state machine and is not directly writable via MSP.

**Approach: contribute `MSP_SET_WP_INDEX` upstream to INAV**

A mid-flight waypoint manipulation workaround (overwriting the current WP slot with a temporary JUMP action, then restoring) was investigated and is technically feasible — but it goes against a core premise of Bullet GCSS: **INAV should be the one flying the aircraft**. Manipulating mission data mid-flight from the ESP32 is fragile and puts mission-management logic in the wrong place.

The right solution is a new `MSP_SET_WP_INDEX` command contributed to INAV that directly writes `posControl.activeWaypointIndex` and triggers the navigation state machine to activate the target waypoint cleanly. The INAV navigation code already has all the logic needed — it just needs an MSP entry point.

**INAV contribution scope:**
- Add a new MSP command (e.g. `MSP_SET_WP_INDEX`, code TBD) to `fc_msp.c` that accepts a `uint8_t` waypoint index, validates it against `posControl.waypointCount`, sets `posControl.activeWaypointIndex`, and fires the appropriate FSM event to activate the new waypoint.
- Open a PR to the INAV repository.

**Bullet GCSS scope (after INAV merges):**
- **Firmware (`ESP32-Modem.cpp`):** add `cmd:jumpwp` handler that sends `MSP_SET_WP_INDEX` to the FC.
- **UI:** add waypoint selector to the Commands panel.
- **Protocol (`BulletGCSS_protocol.md`):** document `cmd:jumpwp,wp:<n>`.

---

### C3. Set Altitude (target altitude for Altitude Hold / Cruise mode)

Allow the operator to command the aircraft to climb or descend to a specific altitude while Altitude Hold or Cruise mode is active.

INAV does not expose the target altitude via MSP. Internally, the target is stored in `posControl.desiredState.pos.z` (float, centimetres, local frame relative to home). It is set exclusively by `updateClimbRateToAltitudeController(desiredClimbRate, targetAltitude, mode)` in `navigation.c`. The relevant mode is `ROC_TO_ALT_TARGET` — this is the same call already used by waypoint navigation at `navigation.c:3496`:

```c
updateClimbRateToAltitudeController(0, pos->z, ROC_TO_ALT_TARGET);
```

This pattern works identically for both multicopters (throttle stick manages climb) and fixed-wing aircraft (pitch stick manages climb) — both share the same variable and function, just with different actuator outputs downstream.

**Approach: contribute `MSP_SET_ALTITUDE` upstream to INAV**

A new MSP command that writes `posControl.desiredState.pos.z` and calls `updateClimbRateToAltitudeController(0, targetAlt, ROC_TO_ALT_TARGET)` would let the ground station command a target altitude cleanly. INAV then handles the climb or descent exactly as it would for a waypoint target — no mid-flight RC channel manipulation needed.

**INAV contribution scope:**
- Add a new MSP command (e.g. `MSP_SET_ALTITUDE`) to `fc_msp.c` that accepts a signed 32-bit integer payload (target altitude in centimetres, relative to home), validates that Altitude Hold or a dependent mode (Cruise, Waypoint) is active, and calls `updateClimbRateToAltitudeController(0, targetAlt, ROC_TO_ALT_TARGET)`.
- Open a PR to the INAV repository.

**Bullet GCSS scope (after INAV merges):**
- **Firmware (`ESP32-Modem.cpp`):** add `cmd:setalt` handler that reads the `alt` field from the downlink message and sends the new MSP command to the FC.
- **UI:** add an altitude input field and Send button to the Commands panel (visible when Altitude Hold or Cruise mode is active).
- **Protocol (`BulletGCSS_protocol.md`):** document the new `cmd:setalt,alt:<centimetres>` downlink message.

---

## Completed Items

- **U1** — Fix main menu in landscape orientation ✅ — resolved by Bootstrap 5 migration (offcanvas sidebar scrolls naturally).
- **U2** — Migrate UI to Bootstrap 5 ✅ — completed in v1.7; `basicui.html` now runs on Bootstrap 5.3.8.
- **F1** — Multi-aircraft monitoring ✅ — completed in v1.6.
- **F2** — Waypoint mission planner ✅ — completed in v1.6.
