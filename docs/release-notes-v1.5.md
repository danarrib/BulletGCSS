# Bullet GCSS v1.5 — Release Notes

## Overview

v1.5 is a significant feature release focused on **remote command capability**, **map interactivity**, and **firmware reliability**. It introduces three new inflight commands (Set Altitude, Set Course, Jump to Waypoint), a reworked interactive map with a modern vector renderer, and several firmware fixes that improve connectivity stability in real-world deployments.

---

## New Features

### Remote Flight Commands

Three new commands are now available in the Commands panel, enabling more capable remote control of the UAV during flight:

- **Set Altitude** — Sends a target altitude (in cm, relative to home) to the flight controller via MSP. Only enabled when Altitude Hold mode is active on the FC.
- **Set Course** — Sets the cruise/course-hold heading in centidegrees. Only enabled when Cruise mode is active on the FC.
- **Jump to Waypoint** — Commands the flight controller to jump to a specific waypoint number in the loaded mission. Only enabled when WP Mission mode is active on the FC.

These commands require INAV firmware version **9.0.2 or later** (the minimum version that supports the underlying MSP write commands). The UI automatically detects the FC firmware version via telemetry and enables or disables these buttons accordingly.

### Click-to-Jump on Map

When WP Mission mode is active, clicking any waypoint marker on the map now opens a confirmation prompt and sends a Jump to Waypoint command for that waypoint. This makes mission management during flight much more intuitive — no need to manually type waypoint numbers.

### Flight Mode State Telemetry

The firmware now reports whether each commandable flight mode is actually active on the flight controller, regardless of the source (manual switch, RC override, or GCS command). Three new telemetry fields — `fmcrs`, `fmalt`, and `fmwp` — allow the UI to accurately gate command availability based on real FC state rather than just what was last commanded.

### FC Firmware Version Telemetry

The firmware version of the flight controller is now read at startup via `MSP_FC_VERSION` and reported to the UI as a low-priority telemetry field (`fcver:M.m.p`). This allows the UI to conditionally enable features based on FC capability without requiring manual configuration.

### Map Overhaul (MapLibre GL JS)

The map engine has been completely replaced, switching from OpenLayers to **MapLibre GL JS** with vector tile rendering. Key improvements:

- Smooth, GPU-accelerated vector map rendering
- Two map styles selectable from the Settings panel: **CARTO Dark Matter** (default) and **OpenFreeMap Liberty**
- Aircraft marker scaled correctly for HiDPI/Retina displays
- All EFIS canvas elements and line widths also scaled by `devicePixelRatio`
- New **center-on-aircraft** button to snap the map view back to the UAV
- Compass button resized for comfortable touch use on mobile screens

### Command Channel Status Improvements

The command status icon in the top bar now has three distinct states — error, warning, and ok — reflecting both the MQTT downlink connection and the MSP RC Override flight mode state on the FC. This gives operators a clearer picture of whether remote commands will actually reach the aircraft.

### Developer Console Override

The internal telemetry data object is now exposed as `window._gcssData` in the browser console. This allows developers and advanced users to inspect live data or override values (e.g., simulating extended command support) without modifying source files.

---

## Firmware Fixes

### GPRS Reconnect Reliability

A significant reliability issue was fixed where the firmware would stall for up to 2 minutes after a mid-flight TCP disconnect. The root cause was that any network failure triggered a full cold-start sequence (modem restart + 600-second network wait), blocking all MQTT publishing and telemetry.

The reconnect logic has been reworked:

- Cold-start (first boot) and mid-flight reconnect are now separate code paths.
- On TCP disconnect, the firmware attempts a fast reconnect without restarting the modem.
- After 5 consecutive failures, a soft modem restart is attempted.
- After 15 failures, the ESP32 performs a full restart as a last resort.

### SIM7600 Power-On Sequence

The SIM7600 4G modem power-on sequence has been corrected. The `MODEM_POWER_ON` pin is now asserted high before the `PWKEY` pulse, giving the power rail time to stabilize before the modem begins its startup sequence. This resolves initialization failures reported on some hardware configurations.

---

## UI Improvements

- **Input validation** on Set Altitude, Set Course, and Jump to Waypoint fields — invalid or out-of-range values are rejected with a visual error state before sending.
- **WP Mission ON** button is now gated on a valid loaded mission being present on the FC (`isWaypointMissionValid`). The button is disabled if no mission is loaded, preventing accidental activation.
- **OFF buttons** for flight mode commands are only enabled once the firmware confirms the mode is actually active, avoiding race conditions between command and telemetry.
- **Jump to WP** input maximum is updated dynamically to match the number of waypoints in the loaded mission.
- Command panel layout reordered for a more logical flow: RTH → Altitude Hold → Set Altitude → Cruise → Set Course → WP Mission → Jump to WP → Beeper → Ping.
- User location compass heading accuracy fix.
- Sidebar z-index fix (sidebar was appearing behind map elements on some browsers).
- Map and data state now reset correctly when a new session begins.

---

## Documentation

- Protocol reference (`docs/BulletGCSS_protocol.md`) updated with all new telemetry fields, the three extended commands, command channel state semantics, and the FC version capability detection mechanism.
- User interface documentation updated to describe the new command icon states.
- MQTT broker settings documentation revised.

---

## INAV Contribution

The three extended commands (`Set Altitude`, `Set Course`, `Jump to Waypoint`) are backed by three new MSP write commands contributed to the INAV project:

| Command | MSP Code | Description |
|---|---|---|
| `MSP2_INAV_SET_WP_INDEX` | 0x2221 | Jump to waypoint N during active mission |
| `MSP2_INAV_SET_ALT_TARGET` | 0x2222 | Set target altitude (cm, relative to home) |
| `MSP2_INAV_SET_CRUISE_HEADING` | 0x2223 | Set cruise/course-hold heading (centidegrees) |

These commands are available in INAV 9.0.2 and later.

---

## Upgrade Notes

- No configuration file changes are required for existing deployments.
- The extended commands (Set Altitude, Set Course, Jump to Waypoint) require **INAV 9.0.2+** on the flight controller. On older firmware, these buttons remain disabled automatically.
- If deploying with a SIM7600 modem, reflashing the firmware is recommended to benefit from the improved power-on sequence and reconnect reliability fixes.
