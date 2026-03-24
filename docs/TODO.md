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

### Step 5 — Key pair setup and distribution
UI generates an Ed25519 key pair. Private key stored in `localStorage` (foundation laid in step 2). Public key must reach the firmware securely so it can verify signed commands.

**Chosen approach: Manual (Option A)**
The UI displays the generated public key and provides a copy button. The user pastes it into `Config.h` and re-flashes the firmware. Fully secure — the public key never travels over any network. Requires re-flashing if the operator changes devices or regenerates the key pair, which is acceptable for this use case.

### Step 6 — Encrypted ping
Change the ping implementation to use Ed25519 message signing. UI signs the ping with the private key; firmware verifies with the stored public key before responding. A monotonically increasing sequence number is included in the signed payload to prevent replay attacks.

This is the foundation for all future uplink commands — once signed ping works reliably, adding new command types is straightforward.

### After step 6 — Flight controller commands
With a verified, signed bidirectional channel in place, implement actual commands: RTH, mission upload, arm/disarm, and eventually ESP32-Cam trigger. Each command follows the same signing pattern established in step 6.

---

## Known Bugs

### 1. SIM cards with PIN lock are not supported
When the SIM card requires a PIN to unlock, the firmware fails to connect — it proceeds directly to network registration without first unlocking the SIM. TinyGSM provides a `modem.simUnlock(pin)` method for this, but it is never called.

**What to do:**
- Add an optional `SIM_PIN` setting to `Config.h` (empty string by default, meaning no PIN).
- In `connectToGprsNetwork()`, after modem initialisation, check SIM status with `modem.getSimStatus()`. If the SIM is locked and `SIM_PIN` is configured, call `modem.simUnlock(SIM_PIN)` before attempting network registration.
- Log a clear error to Serial if the unlock fails (wrong PIN), rather than silently hanging on network registration.

### 2. User location icon on the map always points north
The operator's location marker on the map has a fixed orientation — it does not rotate with the phone's compass, so the arrow always points north regardless of which direction the operator is facing.

**What to do:**
- Use the `DeviceOrientationEvent` browser API to read compass heading. On iOS, `webkitCompassHeading` gives a direct magnetic heading. On Android, heading must be derived from the `alpha` angle with appropriate correction.
- Note: iOS 13+ requires a user permission prompt before `DeviceOrientationEvent` fires — this must be triggered by a user gesture (e.g. a button tap).
- Rotate the user marker SVG/icon on the map to match the live heading.
- Degrade gracefully on devices that do not expose orientation (desktop browsers, some Android devices) — keep the marker visible but without rotation.

---

## Security

### 3. No authentication on the telemetry stream ⚠️ Won't fix / by design
Bullet GCSS is a **receive-only** system — it does not send commands to the aircraft. The worst a bad actor can do by injecting messages is display incorrect telemetry on the UI; they cannot affect the flight. Users who require privacy or data integrity can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is already documented.

---

## Architecture

### 4. Custom CSV telemetry format — consider migrating to binary ⏳ Deferred
JSON was considered but rejected — it adds overhead compared to the current key:value format, and the long-term plan is to migrate to a compact binary protocol instead. Deferred until the binary format design is ready.

### 5. MQTT QoS 0 — no delivery guarantee ⚠️ Won't fix / by design
QoS 0 is intentional. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display; the 10-second force-refresh cycle re-syncs all fields automatically when connectivity returns. QoS 1 would add broker state and acknowledgment overhead without meaningfully improving the operator experience. Documented in `docs/BulletGCSS_protocol.md`.

### 6. Data flow is one-way — no uplink capability ⏳ Deferred
Uplink is planned for a future version. When the time comes, design should cover: a dedicated MQTT command topic (`bulletgcss/cmd/<callsign>`), an authentication strategy, and a command acknowledgment mechanism. The firmware already defines `MSP_SET_WP` and `msp_set_wp_t`, and `BOXGCSNAV` exists in the MSP enum, so the groundwork is there.

**Security decision:** uplink messages will be signed using **Ed25519** (public/private key). The operator's private key stays in the browser; only the public key is stored in `Config.h` on the firmware. The firmware verifies the signature before acting on any command. This ensures that even on a public MQTT broker, nobody can forge commands to the aircraft. A monotonically increasing sequence number will be included in each signed message to prevent replay attacks. Library: Rhys Weatherley's Arduino Crypto (`rweather/arduinolibs`).

### 7. Protocol version detection and backwards compatibility ✓ COMPLETE
`pv` field added to the low-priority message (sent every 60 s). The UI reads `pv` and stores it in `data.protocolVersion`. Missing `pv` (old firmware) is treated as version 1. Version information is logged to the browser console only. See `docs/BulletGCSS_protocol.md` for the full field reference.

---

## Documentation

### 8. No security warning in `README.md`
The README gives no indication that the default configuration broadcasts the aircraft's real-time GPS location to a public server readable by anyone. This is a meaningful privacy and safety concern for new users.

**What to do:**
- Add a **Security Notice** section to `README.md` explaining the public broker risk and linking to `docs/Self-Hosting-a-MQTT-server--(broker).md`.

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
