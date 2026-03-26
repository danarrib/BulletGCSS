# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities. Items are grouped by category and re-enumerated after each cleanup pass.

---

## Roadmap

The following sequence defines the planned implementation order. Each step is a prerequisite for the next.

### ~~Step 1 — ES Modules refactor~~ ✓ COMPLETE

### ~~Step 2 — Flight Sessions (item F5)~~ ✓ COMPLETE

### ~~Step 3 — Protocol version detection (item 7)~~ ✓ COMPLETE
`pv` field added to the low-priority message (sent every 60 s). UI reads `pv` and stores it as `data.protocolVersion`. Missing `pv` field (old firmware) is treated as version 1. Version information is logged to the browser console for debugging only — no user-facing warnings for now.

### ~~Step 4 — Bidirectional channel setup + ping~~ ✓ COMPLETE
Two MQTT topics configured separately — uplink (`bulletgcss/telem/<callsign>`) and downlink (`bulletgcss/cmd/<callsign>`). Firmware subscribes to downlink on connect, reports `dls:0/1` in telemetry. UI shows a downlink status icon. Commands carry a 6-character random `cid`; firmware echoes it back in `id:ack,cid:...,`. UI tracks pending commands and marks them received or lost (no ack after 10 messages). "Send command to UAV" panel in the sidebar has a Ping button and a live command history list.

### ~~Step 5 — Key pair setup and distribution~~ ✓ COMPLETE
UI generates an Ed25519 key pair using the Web Crypto API. Private key stored as JWK in `localStorage`; public key stored as hex. A new **Security** panel in the sidebar displays the ready-to-paste C declaration (`const uint8_t commandPublicKey[32] = { ... };`) with a copy button. `Config.h.example` now includes the `commandPublicKey` field (all zeros until the user generates and pastes a real key). The chosen approach is **Manual (Option A)** — the public key never travels over any network.

### ~~Step 6 — Encrypted ping~~ ✓ COMPLETE
All commands (starting with ping) use Ed25519 message signing. UI signs every command with the private key; firmware verifies with the stored `commandPublicKey` before responding. A monotonically increasing sequence number is included in the signed payload to prevent replay attacks, and the last accepted sequence number is persisted to NVS so replay protection survives a firmware reboot. Commands with a bad signature, missing fields, replayed sequence numbers, or sent without a configured public key are silently dropped (no ack).

This is the foundation for all future uplink commands — adding new command types only requires dispatching on `cmd` in `mqttCommandCallback` after the shared verification logic passes.

### After step 6 — Flight controller commands
With a verified, signed bidirectional channel in place, implement actual commands. Each command follows the same signing pattern established in step 6.

Planned commands (all require Step 6 complete):

| Command | MSP command | In msp_library.h? | Notes |
|---------|-------------|-------------------|-------|
| RTH on/off | `MSP_SET_RAW_RC` (200) | ✓ | Set the RC channel mapped to BOXNAVRTH within its activation range |
| Mission Mode on/off | `MSP_SET_RAW_RC` (200) | ✓ | Set the RC channel mapped to BOXNAVWP within its activation range |
| Mission Upload | `MSP_SET_WP` (209) | ✓ | Send waypoints sequentially; `msp_set_wp_t` struct already defined |
| Change current WP | TBD | TBD | Needs further research |
| Cruise Mode on/off | `MSP_SET_RAW_RC` (200) | ✓ | Set the RC channel mapped to BOXNAVCRUISE within its activation range |
| Altitude Hold on/off | `MSP_SET_RAW_RC` (200) | ✓ | Set the RC channel mapped to BOXNAVALTHOLD within its activation range |
| Change target altitude | TBD | TBD | Needs further research |
| Change target course | TBD | TBD | Needs further research |
| Beeper on/off | `MSP_SET_RAW_RC` (200) | ✓ | Set the RC channel mapped to BOXBEEPERON within its activation range |

#### Reference implementation

- [stronnag/msp_set_rx](https://github.com/stronnag/msp_set_rx) — reference implementation demonstrating `MSP_SET_RAW_RC` for INAV/Betaflight. Read this before implementing.

#### MSP implementation notes

**Why `MSP_SET_RAW_RC` and not `MSP_SET_BOX`:**
`MSP_SET_BOX` (203) is defined in the MSP protocol spec but was **never implemented in INAV**. There is no direct way to toggle flight modes via MSP. The only supported mechanism is `MSP_SET_RAW_RC` — simulating RC channel input so that INAV's normal mode-switching logic activates the desired box.

**How the firmware discovers channel-to-mode mapping — `MSP_MODE_RANGES` (34):**

Rather than hardcoding channel assignments in `Config.h`, the firmware can query `MSP_MODE_RANGES` at startup. INAV returns up to 40 entries (one per configured mode condition), each exactly 4 bytes:

```
uint8_t permanentId     // Box permanent ID (see table below)
uint8_t auxChannelIndex // AUX channel, 0-based (AUX1=0, AUX2=1, ...)
uint8_t startStep       // Range start, in steps
uint8_t endStep         // Range end, in steps
```

Steps are converted to PWM values with: `PWM = 900 + step × 25`
- Step 0 = 900 µs, step 24 = 1500 µs, step 48 = 2100 µs

The `auxChannelIndex` is 0-based AUX channel. Since channels 1–4 are typically AETR sticks, `MSP_SET_RAW_RC` channel index = `auxChannelIndex + 4` (0-based).

**Permanent IDs for the modes we care about** (from INAV `rc_modes.h`):

| Mode | permanentId |
|------|------------|
| Altitude Hold (`BOXNAVALTHOLD`) | 3 |
| RTH (`BOXNAVRTH`) | 8 |
| Beeper (`BOXBEEPERON`) | 11 |
| Waypoint/Mission (`BOXNAVWP`) | 19 |
| Cruise (`BOXNAVCRUISE`) | 44 |

**Startup routine to implement:**
1. Call `MSP_MODE_RANGES` (34) — no request payload.
2. Parse the response in 4-byte chunks (up to 40 entries).
3. Skip entries where `startStep >= endStep` (unused/unconfigured slots).
4. For each entry whose `permanentId` matches a mode we care about, store:
   - RC channel index: `auxChannelIndex + 4` (0-based, for use in `MSP_SET_RAW_RC`)
   - ON value: midpoint of range → `900 + (startStep + endStep) / 2 * 25`
   - OFF value: `900` (safely below all ranges)
5. Store alongside existing `boxId*` globals (e.g. add `boxRthChannel`, `boxRthOnValue`, etc.).

Entries with `permanentId = 0` and `startStep = endStep = 0` are empty slots — skip them. A mode not found in the response means it has no RC channel assigned in INAV — the firmware should log a warning and disable that command.

**`MSP_SET_RAW_RC` (200) — sending channel values:**
- No reply from FC.
- Payload: array of `uint16_t` channel values (little-endian), 2 bytes each, all channels in one message.
- Channels 0–3 (AETR sticks) should be set to neutral (1500) or left at current values — do not touch them unexpectedly.
- To activate a mode: set the relevant channel to the ON value derived from `MSP_MODE_RANGES`.
- To deactivate: set the channel back to OFF value (900).
- Both `MSP_RC` (105, read channels) and `MSP_SET_RAW_RC` (200) are already defined in `msp_library.h`.

**MSP_SET_WP (209) — waypoint upload:**
- Payload: 21 bytes (`msp_set_wp_t`, already defined in `msp_library.h`).
- Coordinates: degrees × 10⁷ (int32). Altitude: centimetres (int32).
- Send waypoints sequentially (index 1, 2, … N). Mark the last one with flag `0xA5`.
- Waypoint action enum values to add for readability: `0x01` = fly to WP, `0x03` = hold, `0x04` = RTH, `0x06` = jump, `0x07` = set heading, `0x08` = land.

**What needs to be added to `msp_library.h` before implementation:**
- `#define MSP_MODE_RANGES 34` and a 4-byte struct for each entry.
- Global variables for the discovered channel/value pairs per mode (e.g. `boxRthChannel`, `boxRthOnValue`, `boxWpChannel`, etc.).
- A `modeRangesFetched` flag (like `boxIdsFetched`) to guard the startup fetch.

---

## Known Bugs

### ~~1. SIM cards with PIN lock are not supported~~ ✓ FIXED
`connectToGprsNetwork()` now checks `modem.getSimStatus()` after modem init. If the SIM is locked and `GSM_PIN` is non-empty, it calls `modem.simUnlock()` and checks the return value. On failure it logs a clear error to Serial and restarts, rather than silently hanging on network registration. The redundant unused `simPIN` variable was removed from `Config.h`; `GSM_PIN` is the single setting.

### 2. User location icon on the map always points north
The operator's location marker on the map has a fixed orientation — it does not rotate with the phone's compass, so the arrow always points north regardless of which direction the operator is facing.

**What to do:**
- Use the `DeviceOrientationEvent` browser API to read compass heading. On iOS, `webkitCompassHeading` gives a direct magnetic heading. On Android, heading must be derived from the `alpha` angle with appropriate correction.
- Note: iOS 13+ requires a user permission prompt before `DeviceOrientationEvent` fires — this must be triggered by a user gesture (e.g. a button tap).
- Rotate the user marker SVG/icon on the map to match the live heading.
- Degrade gracefully on devices that do not expose orientation (desktop browsers, some Android devices) — keep the marker visible but without rotation.

---

## Security

### 3. No authentication on the telemetry stream ⚠️ Partially mitigated — see roadmap
The telemetry uplink is unauthenticated by design: a bad actor injecting messages can only display incorrect data on the UI, not affect the flight. However, now that a command downlink exists, injected command messages are a real concern. This is addressed in the roadmap via Ed25519 signing (Steps 5–6) — the firmware will reject any unsigned command. Until Steps 5–6 are complete, using a private broker is strongly recommended for anyone using the command channel. Users who require privacy or data integrity can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is already documented in `README.md`.

---

## Architecture

### 4. Custom CSV telemetry format — consider migrating to binary ⏳ Deferred
JSON was considered but rejected — it adds overhead compared to the current key:value format, and the long-term plan is to migrate to a compact binary protocol instead. Deferred until the binary format design is ready.

### 5. MQTT QoS 0 — no delivery guarantee ⚠️ Won't fix / by design
QoS 0 is intentional. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display; the 10-second force-refresh cycle re-syncs all fields automatically when connectivity returns. QoS 1 would add broker state and acknowledgment overhead without meaningfully improving the operator experience. Documented in `docs/BulletGCSS_protocol.md`.

### 6. Flight controller commands over the uplink ⏳ Deferred (channel infrastructure complete)
The bidirectional MQTT channel is now in place (Step 4 complete): separate uplink (`bulletgcss/telem/<callsign>`) and downlink (`bulletgcss/cmd/<callsign>`) topics, firmware subscribes and acknowledges commands with a `cid`, UI tracks pending commands and marks them received or lost. The Ping command is implemented end-to-end.

What remains is actual **flight controller commands** — RTH, mission upload, arm/disarm — which depend on Steps 5–6 (key pair setup and signed ping). The firmware already defines `MSP_SET_WP`, `msp_set_wp_t`, and `BOXGCSNAV` in the MSP enum, so the MSP groundwork is there.

**Security decision:** all commands will be signed using **Ed25519** (public/private key). The operator's private key stays in the browser; only the public key is stored in `Config.h` on the firmware. The firmware verifies the signature before acting on any command. This ensures that even on a public MQTT broker, nobody can forge commands to the aircraft. A monotonically increasing sequence number will be included in each signed message to prevent replay attacks. Library: Rhys Weatherley's Arduino Crypto (`rweather/arduinolibs`).

### 7. Protocol version detection and backwards compatibility ✓ COMPLETE
`pv` field added to the low-priority message (sent every 60 s). The UI reads `pv` and stores it in `data.protocolVersion`. Missing `pv` (old firmware) is treated as version 1. Version information is logged to the browser console only. See `docs/BulletGCSS_protocol.md` for the full field reference.

---

## Documentation

### 8. No security warning in `README.md`
The README gives no indication that the default configuration broadcasts the aircraft's real-time GPS location to a public server readable by anyone. This is a meaningful privacy and safety concern for new users.

**What to do:**
- Add a **Security Notice** section to `README.md` explaining the public broker risk and linking to `docs/Self-Hosting-a-MQTT-server--(broker).md`.

### 9. Documentation screenshots are outdated
Several docs contain screenshots of the UI and `Config.h` that no longer match the current state of the project (new topics, new fields, UI changes from Step 4, etc.).

**Known files to update:**
- `docs/Find-a-MQTT-Broker.md` — screenshot of `Config.h` shows old topic structure.
- Other docs may have stale screenshots — do a full pass before closing this item.

---

## Future Features

### F1. Binary telemetry protocol
Replace the current ASCII key:value format with a compact binary protocol. Goal: reduce bytes per message, add implicit versioning, and make parsing deterministic on both the firmware and UI sides. Design TBD — the protocol format, framing, and versioning strategy need to be decided before implementation.

### F2. Multi-aircraft monitoring
Allow the UI to subscribe to multiple MQTT topics simultaneously and display all aircraft on a single map. Each aircraft would have its own icon on the map. A quick-selector in the data/EFIS panel would let the operator switch which aircraft's telemetry is displayed in detail.

Design considerations:
- UI subscribes to a list of topics (configured in settings).
- `data` object becomes a keyed map of per-aircraft state.
- Map renders all aircraft icons simultaneously; clicking one selects it as the active aircraft for the data panel and EFIS.

### F3. Replace status bar PNG icons with inline SVG
The status bar icons (connection, cell signal, RC signal, battery, GPS, hardware health) are currently ~35 PNG files in `UI/img/`. Replace them with inline SVG elements embedded directly in `basicui.html`. JS switches state by changing a CSS class or attribute rather than swapping `src` paths. Benefits: eliminates the PNG files from the repo, reduces first-load network requests, and SVG is the right format for symbolic icons.

**Design notes:**
- One SVG element per icon type in the HTML, with state-driven CSS classes (e.g. `class="icon-ok"`, `class="icon-warning"`, `class="icon-error"`).
- The `getXxxIcon()` functions in `InfoPanelScripts.js` would change from returning a file path to returning a class name or state string.
- CSS handles color/opacity changes per state — no drawing code needed.
- The `aircraft.png` map marker and other non-status images are out of scope for this change.

### F4. Migrate map from OpenLayers + raster tiles to MapLibre GL JS + vector tiles

**Motivation:** The current OpenLayers map uses raster image tiles (PNG/JPEG), which are large, zoom-level-specific, and optimised for street navigation rather than UAV operations. Aircraft typically fly in remote areas with poor cellular coverage, making data usage and offline capability important.

**Chosen stack:**
- **MapLibre GL JS** — open-source, WebGL-accelerated vector map renderer. No API key, no account required. Replaces OpenLayers entirely.
- **OpenFreeMap** as the vector tile source — free, no API key, based on OpenStreetMap data. Includes contour lines and land cover.
- **OpenTopoData.org** for point elevation queries — already in use for waypoint terrain elevation. No change needed here.

**Why vector tiles over raster:**
- Vector tiles (PBF format) are typically 5–20 KB vs 20–80 KB for raster tiles for the same area.
- Zoom and pan re-render existing geometry client-side — fewer new tile fetches.
- PWA tile caching becomes practical: pre-caching a flying area over WiFi before heading to a remote location is feasible with vector tiles but impractical with raster due to file count.

**Map style for UAV operations:**
- Dark theme matching the existing UI.
- Terrain contour lines prominent.
- Land cover colours: water, forest, open land clearly distinct.
- Roads and settlements visible but visually secondary.
- POI clutter (shops, restaurants, etc.) hidden or minimal.

**Optional overlay — airspace:**
OpenAIP provides free aviation overlay data (airspace classes, airfields, NOTAMs) that could be added as an optional layer without requiring an account.

**Migration scope:** `UI/js/MapScripts.js` will largely be a rewrite. Core operations to reimplement: map initialisation, aircraft marker with heading rotation, flight track polyline, waypoint markers, track/pan mode, home position marker. OpenLayers (`UI/ol/`) can be removed from the repository once migration is complete.

### F6. Mission planner
Allow the operator to plan a waypoint mission directly in the UI, rather than requiring a separate ground control application.

**High-level scope:**
- Operator taps/clicks on the map to place waypoints.
- Each waypoint has configurable properties: target altitude, fly-over speed, and action (none, take photo, loiter, RTH, land).
- Waypoints can be reordered, moved (drag on map), or deleted.
- The planned mission can be previewed as a polyline on the map with distance and estimated flight time.
- When ready, the mission is uploaded to the aircraft via the uplink channel (depends on item 6 — uplink capability).
- Mission files should be exportable/importable in a standard format (INAV/Betaflight mission JSON or similar).

**Dependencies:** uplink capability (item 6) must exist before missions can be sent to the aircraft. The planner UI can be built independently of that.

### F7. ESP32-Cam integration — aerial image capture
Support an ESP32-Cam module connected to the UAV, allowing the operator to trigger image captures and view them in the ground station.

**Scope:**
- The ESP32-Cam would operate as a companion device alongside the telemetry ESP32, or as a combined board if hardware allows.
- The operator triggers a capture from the UI; the image is transferred to the ground station and displayed in a dedicated panel or overlay.

**Open design questions:**
- **Transport:** MQTT is not well-suited for binary image data. Options include: a separate direct HTTP endpoint on the ESP32-Cam, encoding images as base64 over MQTT (inefficient), or a dedicated WebSocket stream.
- **Trigger mechanism:** the uplink channel (item 6) could carry the capture command, or a separate command channel could be defined for the camera.
- **Image size vs data usage:** on cellular, full-resolution JPEG frames are expensive. The firmware should compress aggressively and allow the operator to configure resolution/quality.
- **Live video:** out of scope for the initial implementation — single frame capture on demand is the target.

---

## Completed

| Item | Description |
|------|-------------|
| 10 | **ES Modules refactor** — All UI JS files migrated from global scripts to ES Modules (`import`/`export`). Global state eliminated; `CommScripts.js` owns the central `data` object. Cache-busting added via import map. |
| F5 | **Flight Sessions** — Every MQTT message is persisted to IndexedDB in real time (two-store schema: `sessions` metadata + `session_messages` log lines). On page load the open session state is restored by fast-forwarding all stored messages through the parser. Sessions can be renamed, replayed with the existing timeline UI, or deleted from the Sessions panel in the sidebar menu. |
| 7 | **Protocol version detection** — `pv` field added to the low-priority message (every 60 s). UI reads `pv` and stores it in `data.protocolVersion`. Missing `pv` (old firmware) is treated as version 1. Version 1 is the current protocol. |
