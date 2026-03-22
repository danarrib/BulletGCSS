# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities identified during a comprehensive codebase review. Items are grouped by priority.

---

## Security — Critical

### 3. No authentication on the telemetry stream — fake data injection ⚠️ Won't fix / by design
Bullet GCSS is a **receive-only** system — it does not send commands to the aircraft. The worst a bad actor can do by injecting messages is display incorrect telemetry on the UI; they cannot affect the flight. Users who require privacy or data integrity can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is already documented.

---

## Known Bugs

### 16. SIM cards with PIN lock are not supported
When the SIM card requires a PIN to unlock, the firmware fails to connect — it proceeds directly to network registration without first unlocking the SIM. TinyGSM provides a `modem.simUnlock(pin)` method for this, but it is never called.

**What to do:**
- Add an optional `SIM_PIN` setting to `Config.h` (empty string by default, meaning no PIN).
- In `connectToGprsNetwork()`, after modem initialisation, check SIM status with `modem.getSimStatus()`. If the SIM is locked and `SIM_PIN` is configured, call `modem.simUnlock(SIM_PIN)` before attempting network registration.
- Log a clear error to Serial if the unlock fails (wrong PIN), rather than silently hanging on network registration.

---

### 17. User location icon on the map always points north
The operator's location marker on the map has a fixed orientation — it does not rotate with the phone's compass, so the arrow always points north regardless of which direction the operator is facing. This makes it less useful as a situational awareness tool.

**What to do:**
- Use the `DeviceOrientationEvent` browser API to read compass heading. On iOS, `webkitCompassHeading` gives a direct magnetic heading. On Android, heading must be derived from the `alpha` angle with appropriate correction.
- Note: iOS 13+ requires a user permission prompt before `DeviceOrientationEvent` fires — this must be triggered by a user gesture (e.g. a button tap).
- Rotate the user marker SVG/icon on the map to match the live heading.
- Degrade gracefully on devices that do not expose orientation (desktop browsers, some Android devices) — keep the marker visible but without rotation.

---

## Architecture

### 7. Custom CSV telemetry format — consider migrating to binary ⏳ Deferred
JSON was considered but rejected — it adds overhead compared to the current key:value format, and the long-term plan is to migrate to a compact binary protocol instead. Deferred until the binary format design is ready.

---

### 8. MQTT QoS 0 — no delivery guarantee ⚠️ Won't fix / by design
QoS 0 is intentional. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display; the 10-second force-refresh cycle re-syncs all fields automatically when connectivity returns. QoS 1 would add broker state and acknowledgment overhead without meaningfully improving the operator experience. Documented in `docs/BulletGCSS_protocol.md`.

---

### 9. Data flow is one-way — no uplink capability ⏳ Deferred
Uplink is planned for a future version. When the time comes, design should cover: a dedicated MQTT command topic (`bulletgcss/cmd/<callsign>`), an authentication strategy, and a command acknowledgment mechanism. The firmware already defines `MSP_SET_WP` and `msp_set_wp_t`, and `BOXGCSNAV` exists in the MSP enum, so the groundwork is there.

**Security decision:** uplink messages will be signed using **Ed25519** (public/private key). The operator's private key stays in the browser; only the public key is stored in `Config.h` on the firmware. The firmware verifies the signature before acting on any command. This ensures that even on a public MQTT broker, nobody can forge commands to the aircraft. A monotonically increasing sequence number will be included in each signed message to prevent replay attacks. Library: Rhys Weatherley's Arduino Crypto (`rweather/arduinolibs`).

---

### 10. Global mutable state in the UI JavaScript
All JS files share a single global `data` object and dozens of global variables. This works at the current scale but makes the code brittle and hard to extend — a naming collision with a third-party library or a future script is easy to introduce silently.

**Chosen approach: ES Modules (`import`/`export`)**
- `CommScripts.js` owns and exports the `data` object and related functions (`resetDataObject`, `parseTelemetryData`, etc.).
- `MapScripts.js`, `EfisScripts.js`, `InfoPanelScripts.js`, and `PageScripts.js` import what they need explicitly.
- Script tags in `basicui.html` change from plain `<script src="...">` to `<script type="module" src="...">`.
- No build step required — browsers handle ES Modules natively. All target browsers (modern Chrome/Firefox/Safari) support this.

**Things to watch out for during the refactor:**
- `type="module"` scripts are always deferred (run after HTML is parsed), so load order assumptions in the current code need to be verified.
- Inline `onclick` handlers in HTML cannot reference module-scoped functions directly — any functions called from HTML must either stay global or be wired up via `addEventListener` in the module.
- Third-party libraries (OpenLayers, Paho MQTT, etc.) are still loaded as plain scripts and will continue to attach to `window` as before.

---

### 15. Protocol version detection and backwards compatibility
There is currently no version field in the telemetry messages. If a breaking change is made to the protocol, the UI will silently misparse data from older firmware with no indication that something is wrong. Users who keep older firmware versions will get a broken experience without any explanation.

**What to do:**

- Add a protocol version field (`pv:<number>`) to the **Session Start message** (`id:0,`). This is sent once when the ESP32 connects, making it the natural place to declare the version. Example: `id:0,pv:2,`
- The UI reads `pv` from the session start message and stores the active protocol version.
- Parsing logic is gated on the detected version: new fields introduced in a later version are only expected if `pv` is high enough; unknown fields from newer firmware are already silently ignored by the parser.
- If no `pv` field is received (old firmware), the UI assumes version 1 and applies the v1 parsing rules.
- A visible notice should be shown when the UI detects that the firmware is running a protocol version older than what it fully supports, explaining that some features may be unavailable.

**Design notes:**
- The version number is an integer, incremented only on **breaking changes** (removed or reinterpreted fields). Adding new optional fields is not a breaking change and does not require a version bump.
- This must be implemented on both sides: firmware increments `pv` when making a breaking change; UI maintains a compatibility table mapping version numbers to parsing rules.
- This is a prerequisite for safely making any breaking changes to the protocol in the future, including the binary protocol migration (item F1).

---

## Documentation

### 12. No security warning in `README.md`
The README gives no indication that the default configuration broadcasts the aircraft's real-time GPS location to a public server readable by anyone. This is a meaningful privacy and safety concern for new users.

**What to do:**
- Add a **Security Notice** section to `README.md` explaining the public broker risk and linking to `docs/Self-Hosting-a-MQTT-server--(broker).md`.

---

## Future Features

Larger features that are planned but not yet scoped or scheduled.

### F4. Migrate map from OpenLayers + raster tiles to MapLibre GL JS + vector tiles

**Motivation:** The current OpenLayers map uses raster image tiles (PNG/JPEG), which are large, zoom-level-specific, and optimised for street navigation rather than UAV operations. Aircraft typically fly in remote areas with poor cellular coverage, making data usage and offline capability important. The current map also has no dark theme and poor terrain visibility.

**Chosen stack:**
- **MapLibre GL JS** — open-source, WebGL-accelerated vector map renderer. No API key, no account required. Replaces OpenLayers entirely.
- **OpenFreeMap** as the vector tile source — free, no API key, based on OpenStreetMap data. Includes contour lines and land cover. Designed for direct use without registration.
- **OpenTopoData.org** for point elevation queries — already in use for waypoint terrain elevation via `proxy.php`. No change needed here.
- No Mapbox, no Stadia, no API keys of any kind.

**Why vector tiles over raster:**
- Vector tiles (PBF format) are typically 5–20 KB vs 20–80 KB for raster tiles for the same area.
- Zoom and pan re-render existing geometry client-side — fewer new tile fetches.
- PWA tile caching becomes practical: pre-caching a flying area over WiFi before heading to a remote location is feasible with vector tiles but impractical with raster due to file count.

**Map style for UAV operations:**
MapLibre uses a JSON style spec that controls what is rendered and how. The style should be tuned for aerial situational awareness rather than street navigation:
- Dark theme matching the existing UI.
- Terrain contour lines prominent (available in OSM vector data — no extra tile source needed).
- Land cover colours: water, forest, open land clearly distinct.
- Roads and settlements visible but visually secondary.
- POI clutter (shops, restaurants, etc.) hidden or minimal.

**Optional overlay — airspace:**
OpenAIP provides free aviation overlay data (airspace classes, airfields, NOTAMs). This could be added as an optional layer on top of the base map without requiring an account for basic use.

**Migration scope:**
`UI/js/MapScripts.js` will largely be a rewrite — the MapLibre API is different enough from OpenLayers that find-and-replace is not practical. Core operations to reimplement:
- Map initialisation and dark style application.
- Aircraft marker: placement, rotation (heading), real-time position updates.
- Flight track polyline: append points as telemetry arrives.
- Waypoint markers: render mission waypoints from telemetry.
- Track mode vs manual pan mode (auto-center on aircraft vs free pan).
- Home position marker.

OpenLayers (`UI/ol/`) can be removed from the repository once migration is complete, which eliminates a large bundled dependency.

---

### F3. Replace status bar PNG icons with inline SVG
The status bar icons (connection, cell signal, RC signal, battery, GPS, hardware health) are currently ~35 PNG files in `UI/img/`. Replace them with inline SVG elements embedded directly in `basicui.html`. JS switches state by changing a CSS class or attribute rather than swapping `src` paths. Benefits: eliminates the PNG files from the repo, reduces first-load network requests, and SVG is the right format for symbolic icons.

**Design notes:**
- One SVG element per icon type in the HTML, with state-driven CSS classes (e.g. `class="icon-ok"`, `class="icon-warning"`, `class="icon-error"`).
- The `getXxxIcon()` functions in `InfoPanelScripts.js` would change from returning a file path to returning a class name or state string.
- CSS handles color/opacity changes per state — no drawing code needed.
- The `aircraft.png` map marker and other non-status images are out of scope for this change.

### F5. Persistent flight data across page refreshes
If the browser tab is accidentally closed or the page refreshes mid-flight, all telemetry data and the flight track are lost. The idea is to persist enough state to resume the display meaningfully when the page reloads.

**Design considerations:**
- Periodically write the current `data` object and the accumulated GPS track to `localStorage` (e.g. every 5 seconds).
- On page load, check for stored state. If found and recent enough (e.g. less than 30 minutes old), restore it and display it as the starting point before new telemetry arrives.
- Include a TTL so stale data from a previous flight session is not mistakenly restored.
- Be mindful of `localStorage` size limits (~5 MB). The GPS track can grow large on long flights — consider storing only the last N track points, or downsampling older points.
- The MQTT connection already reconnects automatically on page load, so live telemetry will resume on its own; this feature is only about restoring the visual state (track history, last known values) in the gap before reconnection.

---

### F6. Mission planner
Allow the operator to plan a waypoint mission directly in the UI, rather than requiring a separate ground control application. This is a large feature that needs detailed specification before implementation.

**High-level scope:**
- Operator taps/clicks on the map to place waypoints.
- Each waypoint has configurable properties: target altitude, fly-over speed, and action (none, take photo, loiter, RTH, land).
- Waypoints can be reordered, moved (drag on map), or deleted.
- The planned mission can be previewed as a polyline on the map with distance and estimated flight time.
- When ready, the mission is uploaded to the aircraft via the uplink channel (depends on TODO item 9 — uplink capability).
- Mission files should be exportable/importable in a standard format (INAV/Betaflight mission JSON or similar) so they can be used with other GCS tools.

**Dependencies:** uplink capability (item 9) must exist before missions can be sent to the aircraft. The planner UI can be built independently of that.

---

### F7. ESP32-Cam integration — aerial image capture
Support an ESP32-Cam module connected to the UAV, allowing the operator to trigger image captures and view them in the ground station.

**Scope:**
- The ESP32-Cam would operate as a companion device alongside the telemetry ESP32, or as a combined board if hardware allows.
- The operator triggers a capture from the UI; the image is transferred to the ground station and displayed in a dedicated panel or overlay.

**Open design questions:**
- **Transport:** MQTT is not well-suited for binary image data (large payloads, broker limits). Options include: a separate direct HTTP endpoint on the ESP32-Cam, encoding images as base64 over MQTT (inefficient), or a dedicated WebSocket stream. This needs to be decided before implementation.
- **Trigger mechanism:** the uplink channel (item 9) could carry the capture command, or a separate command channel could be defined specifically for the camera.
- **Image size vs data usage:** on cellular, full-resolution JPEG frames are expensive. The firmware should compress aggressively and allow the operator to configure resolution/quality.
- **Live video:** out of scope for the initial implementation — single frame capture on demand is the target. Streaming would require a fundamentally different transport approach.

---

### F1. Binary telemetry protocol
Replace the current ASCII key:value format with a compact binary protocol. Goal: reduce bytes per message, add implicit versioning, and make parsing deterministic on both the firmware and UI sides. Design TBD — the protocol format, framing, and versioning strategy need to be decided before implementation.

### F2. Multi-aircraft monitoring
Allow the UI to subscribe to multiple MQTT topics simultaneously and display all aircraft on a single map. Each aircraft would have its own icon on the map. A quick-selector in the data/EFIS panel would let the operator switch which aircraft's telemetry is displayed in detail.

Design considerations:
- UI subscribes to a list of topics (configured in settings).
- `data` object becomes a keyed map of per-aircraft state.
- Map renders all aircraft icons simultaneously; clicking one selects it as the active aircraft for the data panel and EFIS.
- Depends on the ES Modules refactor (item 10) to make per-aircraft state management clean.
