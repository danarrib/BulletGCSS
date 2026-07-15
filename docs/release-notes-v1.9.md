# Bullet GCSS v1.9 Release Notes

## Firmware — Flight Mode Command Channel Rework

This release replaces the flight mode command mechanism entirely, removing a significant setup burden and a source of fragility.

### MSP2_INAV_SET_AUX_RC Replaces MSP_SET_RAW_RC
Flight mode commands (RTH, AltHold, PosHold, Cruise, WP Mission, Beeper) previously relied on `MSP_SET_RAW_RC` overriding a receiver channel, which required the operator to have MSP RC Override configured and a compatible aux channel mapped in INAV Configurator beforehand. The firmware now uses `MSP2_INAV_SET_AUX_RC` (0x2230) instead:

- At startup, the firmware scans all 40 INAV mode condition slots, claims a free dedicated aux channel at CH25+, and writes fixed activation ranges via `MSP_SET_MODE_RANGE` automatically — no manual INAV Configurator setup required.
- On an ESP32 watchdog reboot, the existing BulletGCSS channel is detected and reused rather than creating a duplicate.
- Values persist on the flight controller indefinitely, so the command is now sent only when the commanded state actually changes, instead of every telemetry cycle.

### MSP RC Override Dependency Removed
Because the new channel doesn't need MSP RC Override, all dependencies on that flight mode have been removed — firmware no longer tracks or publishes an override-active field, and the UI no longer gates commands or icons on it, including in the Mission Planner's upload/download flow.

### AltHold Co-Activation for PosHold and Cruise
The AltHold range was widened so that PosHold and Cruise now sit nested inside it. Commanding either one automatically co-activates AltHold via range overlap on the shared aux channel, matching INAV's own mode-condition behavior.

### Set Altitude Payload Fix
Fixed the `MSP2_INAV_SET_ALT_TARGET` command code and payload format to match the INAV 10 specification (a datum byte plus a signed 32-bit centimetre offset). The "Set Altitude" command in the UI is now gated on INAV 10.0.0+ specifically, rather than the general extended-commands flag.

---

## UI — General

### Status Icon Bar Scales with Viewport
Status bar icons (connection, signal, battery, GPS, command channel) were previously fixed at a constant size regardless of window size. They now scale with the viewport, consistent with how telemetry text already scales, so the bar looks right on everything from a small phone to a wide desktop window. The layout below the navbar now also tracks the navbar's real rendered height instead of assuming a constant, so nothing overlaps or gaps as icon size changes.

---

## Documentation

- Rewrote the README with a full feature list, screenshot table, and architecture diagram, and downloaded referenced hardware/wiring images locally instead of hotlinking.
- Refreshed the hardware, wiring, MQTT broker, and self-hosting guides with corrections, INAV version requirements, and clearer explanations.
- Added a local HTTPS dev server (`scripts/https-dev-server.py`) for iterating on UI features that require a secure context (such as Web Crypto for Ed25519 key handling) without deploying to staging first.
- Added a written plan exploring a future long-range LoRa telemetry link as an alternative to cellular, including Meshtastic as a lower-effort option that would reuse the existing MQTT-based UI unchanged.

---

## Notes for Upgraders

Aircraft running firmware from v1.8 or earlier will need to be re-flashed to pick up the new aux RC channel mechanism — the old `MSP_SET_RAW_RC`-based command channel has been removed. No changes to INAV Configurator mode setup are required after upgrading; the firmware configures its own channel automatically on first boot.
