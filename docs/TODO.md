# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities. Items are grouped by category and re-enumerated after each cleanup pass.

---

## Security

### 1. No authentication on the telemetry stream ⚠️ Partially mitigated
The telemetry uplink is unauthenticated by design: a bad actor injecting messages can only display incorrect data on the UI, not affect the flight. Command messages are protected by Ed25519 signing (Steps 5–6 complete) — the firmware rejects any unsigned command. Users who require privacy or data integrity on telemetry can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is documented in `README.md`.

---

## Architecture

### 2. Custom CSV telemetry format — consider migrating to binary ⏳ Deferred
JSON was considered but rejected — it adds overhead compared to the current key:value format, and the long-term plan is to migrate to a compact binary protocol instead. Deferred until the binary format design is ready.

### 3. MQTT QoS 0 — no delivery guarantee ⚠️ Won't fix / by design
QoS 0 is intentional. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display; the 10-second force-refresh cycle re-syncs all fields automatically when connectivity returns. QoS 1 would add broker state and acknowledgment overhead without meaningfully improving the operator experience. Documented in `docs/BulletGCSS_protocol.md`.

---

## Documentation

### 4. Documentation screenshots are outdated
Several docs contain screenshots of the UI and `Config.h` that no longer match the current state of the project (new topics, new fields, UI changes, MapLibre map migration).

**Known files to update:**
- `docs/Find-a-MQTT-Broker.md` — screenshot of `Config.h` shows old topic structure; UI screenshot shows old broker settings panel.
- `docs/User-Interface.md` — map screenshot shows old OpenLayers raster map; UI has changed significantly since (Commands panel, Sessions panel, Security panel, new status icons, MapLibre map).
- `docs/README.md` — UI screenshot shows the old interface.
- Do a full pass before closing this item.

---

## UI Improvements

### U1. Migrate UI to Bootstrap 5 ⏳ Deferred
Eventually migrate the UI to Bootstrap 5 for consistent, mobile-correct components. Priority: replace the sidebar menus with Bootstrap's **Offcanvas** component (directly fixes the touch handling issues). Bootstrap 5.3 dark mode (`data-bs-theme="dark"`) reduces the conflict with the existing dark theme. Integration approach: adopt Bootstrap JS + utility classes first, keep existing custom CSS for layout and theme, audit conflicts incrementally.

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

### F4. Mission planner
Allow the operator to plan a waypoint mission directly in the UI, rather than requiring a separate ground control application.

**High-level scope:**
- Operator taps/clicks on the map to place waypoints.
- Each waypoint has configurable properties: target altitude, fly-over speed, and action (none, take photo, loiter, RTH, land).
- Waypoints can be reordered, moved (drag on map), or deleted.
- The planned mission can be previewed as a polyline on the map with distance and estimated flight time.
- When ready, the mission is uploaded to the aircraft via the command channel.
- Mission files should be exportable/importable in a standard format (INAV/Betaflight mission JSON or similar).

### F5. ESP32-Cam integration — aerial image capture
Support an ESP32-Cam module connected to the UAV, allowing the operator to trigger image captures and view them in the ground station.

**Scope:**
- The ESP32-Cam would operate as a companion device alongside the telemetry ESP32, or as a combined board if hardware allows.
- The operator triggers a capture from the UI; the image is transferred to the ground station and displayed in a dedicated panel or overlay.

**Open design questions:**
- **Transport:** MQTT is not well-suited for binary image data. Options include: a separate direct HTTP endpoint on the ESP32-Cam, encoding images as base64 over MQTT (inefficient), or a dedicated WebSocket stream.
- **Trigger mechanism:** the command channel could carry the capture command, or a separate command channel could be defined for the camera.
- **Image size vs data usage:** on cellular, full-resolution JPEG frames are expensive. The firmware should compress aggressively and allow the operator to configure resolution/quality.
- **Live video:** out of scope for the initial implementation — single frame capture on demand is the target.

---

## Completed

| Item | Description |
|------|-------------|
| Step 1 | **ES Modules refactor** — All UI JS files migrated from global scripts to ES Modules (`import`/`export`). Global state eliminated; `CommScripts.js` owns the central `data` object. Cache-busting added via import map. |
| Step 2 | **Flight Sessions** — Every MQTT message is persisted to IndexedDB in real time (two-store schema: `sessions` metadata + `session_messages` log lines). On page load the open session state is restored by fast-forwarding all stored messages through the parser. Sessions can be renamed, replayed, exported, imported, or deleted from the Sessions panel. |
| Step 3 | **Protocol version detection** — `pv` field added to the low-priority message (every 60 s). UI reads `pv` and stores it in `data.protocolVersion`. Missing `pv` (old firmware) is treated as version 1. |
| Step 4 | **Bidirectional channel setup + ping** — Separate uplink/downlink MQTT topics. Firmware subscribes and acknowledges commands with a `cid`. UI tracks pending commands and marks them received or lost. |
| Step 5 | **Key pair setup and distribution** — Ed25519 key pair generated via Web Crypto API. Private key in `localStorage`; public key pasted into `Config.h`. Security panel displays ready-to-paste C declaration. |
| Step 6 | **Encrypted ping** — All commands signed with Ed25519. Monotonically increasing sequence number for replay protection, persisted to NVS across reboots. |
| Step 7 | **Flight controller commands** — RTH, Altitude Hold, Cruise, Mission (WP), Angle, and Beeper toggled via `MSP_SET_RAW_RC`. Channel deactivation uses a gap-finding algorithm. UI commands panel with ON/OFF buttons, visual state indication, and a FAB for quick access. Single command channel status icon with 3 states (error / warning / ok). |
| Bug 1 (SIM) | **SIM cards with PIN lock** — `connectToGprsNetwork()` now calls `modem.simUnlock()` when `GSM_PIN` is set. On failure, logs a clear error and restarts. |
| Bug 2 | **User location compass heading** — `DeviceOrientationEvent` integrated in `MapScripts.js`. iOS uses `webkitCompassHeading`; Android/Chrome uses `deviceorientationabsolute`. iOS 13+ button in sidebar triggers permission; all other platforms start automatically. |
| U1 | **CSS touch fixes** — `touch-action: manipulation`, `user-select: none`, and `cursor: pointer` added to all interactive elements. `:hover` styles restricted to pointer devices via `@media (hover: hover)`. |
| U2 | **Command state telemetry + visual feedback** — Per-command active state fields (`cmdrth`, `cmdalt`, `cmdcrs`, `cmdwp`, `cmdbep`) added to telemetry uplink. Commands panel buttons driven by firmware-reported state. |
| U3 | **Alpine.js** — Included via CDN to replace manual DOM manipulation for panel/menu state and button active states. |
| Doc 1 | **INAV 9 minimum version** — All relevant documentation updated to state that Bullet GCSS requires INAV 9 or newer. |
| Doc 2 | **README security notice** — Prominent ⚠ Security Notice section added explaining public broker risk (GPS location visible to anyone), linking to self-hosting guide, and noting commands are Ed25519-protected. |
| F (map) | **MapLibre GL JS migration** — OpenLayers replaced with MapLibre GL JS vector map. Two user-selectable styles (OpenFreeMap Liberty default, CARTO Dark Matter). All elements scaled by `devicePixelRatio`. Compass and center-on-aircraft controls added. |
