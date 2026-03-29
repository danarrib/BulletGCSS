# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities. Items are grouped by category and re-enumerated after each cleanup pass.

---

## Known Bugs

### ~~1. User location icon on the map always points north~~ ✓ COMPLETE
`DeviceOrientationEvent` integrated in `MapScripts.js` via `startOrientationTracking()`. iOS uses `webkitCompassHeading`; Android/Chrome uses `deviceorientationabsolute` (alpha converted to CW heading). iOS 13+ permission prompt triggered by the "Enable compass heading" sidebar button (shown only on iOS; hidden on all other platforms). Non-iOS devices start automatically on page load. GPS movement heading (`position.coords.heading`) is only used as fallback when the compass sensor is not active. Degrades gracefully on desktop — marker remains visible without rotation.

---

## Security

### 2. No authentication on the telemetry stream ⚠️ Partially mitigated
The telemetry uplink is unauthenticated by design: a bad actor injecting messages can only display incorrect data on the UI, not affect the flight. Command messages are protected by Ed25519 signing (Steps 5–6 complete) — the firmware rejects any unsigned command. Users who require privacy or data integrity on telemetry can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is documented in `README.md`.

---

## Architecture

### 3. Custom CSV telemetry format — consider migrating to binary ⏳ Deferred
JSON was considered but rejected — it adds overhead compared to the current key:value format, and the long-term plan is to migrate to a compact binary protocol instead. Deferred until the binary format design is ready.

### 4. MQTT QoS 0 — no delivery guarantee ⚠️ Won't fix / by design
QoS 0 is intentional. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display; the 10-second force-refresh cycle re-syncs all fields automatically when connectivity returns. QoS 1 would add broker state and acknowledgment overhead without meaningfully improving the operator experience. Documented in `docs/BulletGCSS_protocol.md`.

---

## Documentation

### ~~5. Document INAV 9 minimum version requirement~~ ✓ COMPLETE
Update all relevant documentation to state that Bullet GCSS requires **INAV 9 or newer**. Files to update: `README.md`, `docs/README.md`, `docs/Setup-modem.md`, and any other doc that mentions INAV without a version constraint.

### ~~6. No security warning in `README.md`~~ ✓ COMPLETE
Added a prominent **⚠ Security Notice** section to `README.md` explaining the public broker risk (GPS location visible to anyone on the same topic), linking to the self-hosting guide, and noting that commands are protected by Ed25519 signatures regardless.

### 7. Documentation screenshots are outdated
Several docs contain screenshots of the UI and `Config.h` that no longer match the current state of the project (new topics, new fields, UI changes, MapLibre map migration).

**Known files to update:**
- `docs/Find-a-MQTT-Broker.md` — screenshot of `Config.h` shows old topic structure; UI screenshot shows old broker settings panel.
- `docs/User-Interface.md` — map screenshot shows old OpenLayers raster map; UI has changed significantly since (Commands panel, Sessions panel, Security panel, new status icons, MapLibre map).
- `docs/README.md` — UI screenshot shows the old interface.
- Do a full pass before closing this item.

---

## UI Improvements

### ~~U1. CSS touch fixes~~ ✓ COMPLETE
Add the following CSS rules to all interactive elements (menu items, buttons, links) to fix the "highlights but doesn't click" issue on mobile and eliminate the 300 ms tap delay:

```css
touch-action: manipulation;
-webkit-tap-highlight-color: transparent;
user-select: none;
cursor: pointer;
```

Restrict `:hover` styles to pointer devices so they don't get stuck on touchscreens:

```css
@media (hover: hover) {
  .menu-item:hover { ... }
}
```

### ~~U2. Command state telemetry fields + commands panel visual feedback~~ ✓ COMPLETE

Add per-command active state fields to the telemetry uplink so the UI knows which flight modes are actively being overridden by the firmware, independently of what the UI last sent.

**New telemetry fields (firmware → UI):**

| Field | Description |
|---|---|
| `cmdrth` | `1` = firmware is actively overriding the RTH channel, `0` = not overriding |
| `cmdalt` | `1` = firmware is actively overriding the Altitude Hold channel |
| `cmdcrs` | `1` = firmware is actively overriding the Cruise channel |
| `cmdwp` | `1` = firmware is actively overriding the Waypoint/Mission channel |
| `cmdbep` | `1` = firmware is actively overriding the Beeper channel |

These fields reflect the firmware's actual RC override state — if the mode is deactivated for any reason (command cancel, MSP RC Override mode turned off, FC reconnect, etc.) the firmware sends `0` and the UI resets the button state accordingly.

**Firmware changes:**
- Add 6 new fields to `uav_status` struct in `uav_status.h`.
- Set them from the `active` flag in each `cmdModes[]` entry inside `msp_send_rc_override()`.
- Include them in the telemetry round-robin (add to an appropriate group in `ESP32-Modem.cpp`).

**UI changes:**
- Parse the new fields in `CommScripts.js` → `data.cmdRth`, `data.cmdAlt`, etc.
- In the commands panel, drive the ON/OFF button visual state from these `data.cmdXxx` fields (not from the last button the user tapped). Green border = firmware confirms active; dimmed = firmware confirms inactive; default = field not yet received.

**Protocol note:** these fields should be added to `docs/BulletGCSS_protocol.md` under Status Flags once implemented.

### ~~U4. Add Alpine.js~~ ✓ COMPLETE
Include Alpine.js (CDN, ~16 KB, no build step) to replace manual DOM manipulation for panel/menu open-close state, button active states, and any other reactive UI logic. This will make the menu and commands panel code cleaner and more reliable on mobile.

### U5. Migrate UI to Bootstrap 5 ⏳ Deferred
Eventually migrate the UI to Bootstrap 5 for consistent, mobile-correct components. Priority: replace the sidebar menus with Bootstrap's **Offcanvas** component (directly fixes the touch handling issues). Bootstrap 5.3 dark mode (`data-bs-theme="dark"`) reduces the conflict with the existing dark theme. Integration approach: adopt Bootstrap JS + utility classes first, keep existing custom CSS for layout and theme, audit conflicts incrementally. **Prerequisite:** U1 and U2 should be done first.

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

### ~~F4. Migrate map from OpenLayers + raster tiles to MapLibre GL JS + vector tiles~~ ✓ COMPLETE

Migration complete. MapLibre GL JS replaces OpenLayers entirely. Two map styles available (user-selectable in UI Settings): **OpenFreeMap Liberty** (default) and **CARTO Dark Matter**. All map elements (line widths, marker sizes, fonts) are scaled by `window.devicePixelRatio`. Custom controls added: compass (north-up reset) and center-on-aircraft button. The `UI/ol/` directory has been removed.

### F5. Mission planner
Allow the operator to plan a waypoint mission directly in the UI, rather than requiring a separate ground control application.

**High-level scope:**
- Operator taps/clicks on the map to place waypoints.
- Each waypoint has configurable properties: target altitude, fly-over speed, and action (none, take photo, loiter, RTH, land).
- Waypoints can be reordered, moved (drag on map), or deleted.
- The planned mission can be previewed as a polyline on the map with distance and estimated flight time.
- When ready, the mission is uploaded to the aircraft via the command channel.
- Mission files should be exportable/importable in a standard format (INAV/Betaflight mission JSON or similar).

### F6. ESP32-Cam integration — aerial image capture
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
| Step 2 / F5 | **Flight Sessions** — Every MQTT message is persisted to IndexedDB in real time (two-store schema: `sessions` metadata + `session_messages` log lines). On page load the open session state is restored by fast-forwarding all stored messages through the parser. Sessions can be renamed, replayed, exported, imported, or deleted from the Sessions panel. |
| Step 3 | **Protocol version detection** — `pv` field added to the low-priority message (every 60 s). UI reads `pv` and stores it in `data.protocolVersion`. Missing `pv` (old firmware) is treated as version 1. |
| Step 4 | **Bidirectional channel setup + ping** — Separate uplink/downlink MQTT topics. Firmware subscribes and acknowledges commands with a `cid`. UI tracks pending commands and marks them received or lost. |
| Step 5 | **Key pair setup and distribution** — Ed25519 key pair generated via Web Crypto API. Private key in `localStorage`; public key pasted into `Config.h`. Security panel displays ready-to-paste C declaration. |
| Step 6 | **Encrypted ping** — All commands signed with Ed25519. Monotonically increasing sequence number for replay protection, persisted to NVS across reboots. |
| Step 7 | **Flight controller commands** — RTH, Altitude Hold, Cruise, Mission (WP), Angle, and Beeper toggled via `MSP_SET_RAW_RC`. Channel deactivation uses a gap-finding algorithm. UI commands panel with ON/OFF buttons, visual state indication, and a FAB for quick access. Single command channel status icon with 3 states (error / warning / ok). |
| Bug 1 | **SIM cards with PIN lock** — `connectToGprsNetwork()` now calls `modem.simUnlock()` when `GSM_PIN` is set. On failure, logs a clear error and restarts. |
| Arch 6 | **Flight controller commands over uplink** — Complete. See Step 7. |
| Arch 7 | **Protocol version detection** — Complete. See Step 3. |
