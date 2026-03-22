# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities identified during a comprehensive codebase review. Items are grouped by priority.

---

## Security — Critical

### ~~1. Credentials hardcoded in `Config.h`~~ ✅ Done
`Config.h` is now gitignored. `Config.h.example` with placeholder values is committed instead. Users copy it to `Config.h` and fill in their credentials. Documented in `docs/Development-setup.md`.

> Note: The credentials that were previously committed (`WiFi password`, `MQTT user/password`) should still be rotated, as they remain in the git history.

---

### ~~2. `proxy.php` is an open proxy~~ ✅ Done
`CSAJAX_FILTERS` set to `true`. Allowlist cleaned up to contain only `api.open-elevation.com`. Note added to `docs/Host-the-user-interface.md` for self-hosters.

---

### 3. No authentication on the telemetry stream — fake data injection ⚠️ Won't fix / by design
Bullet GCSS is a **receive-only** system — it does not send commands to the aircraft. The worst a bad actor can do by injecting messages is display incorrect telemetry on the UI; they cannot affect the flight. Users who require privacy or data integrity can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is already documented.

---

### ~~4. ESP32 → Broker link is unencrypted (port 1883, no TLS)~~ ✅ Done
TLS enabled using `govorox/SSLClient` wrapping `TinyGsmClient` (cellular) and `WiFiClientSecure` (WiFi). Encryption is active but certificate verification is skipped (`VERIFY_NONE`) — protects against passive eavesdropping without requiring certificate management. MQTT port updated to 8883 in `Config.h.example`.

---

## Security — Medium

### ~~5. Public broker by default — no privacy~~ ✅ Done
Added a friendly privacy note to `README.md` informing users that the default public broker makes telemetry visible to anyone on the same topic, with a link to the self-hosting guide for those who want privacy. Kept the tone light — this is a hobbyist project.

---

### ~~6. No input validation on the telemetry parser~~ ✅ Done
`parseStandardTelemetryMessage()` in `UI/js/CommScripts.js` calls `parseInt`/`parseFloat` directly on untrusted values with no bounds checking. A malformed or adversarial MQTT message could result in nonsensical display values (negative satellite counts, impossible altitudes, etc.).

**What to do:**
- Add range checks for critical fields (GPS coordinates, altitude, battery voltage, satellite count).
- Silently clamp or discard out-of-range values rather than displaying them.

---

## Architecture

### 7. Custom CSV telemetry format — consider migrating to binary ⏳ Deferred
JSON was considered but rejected — it adds overhead compared to the current key:value format, and the long-term plan is to migrate to a compact binary protocol instead. Deferred until the binary format design is ready.

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

### 8. MQTT QoS 0 — no delivery guarantee ⚠️ Won't fix / by design
QoS 0 is intentional. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display; the 10-second force-refresh cycle re-syncs all fields automatically when connectivity returns. QoS 1 would add broker state and acknowledgment overhead without meaningfully improving the operator experience. Documented in `docs/BulletGCSS_protocol.md`.

---

### 9. Data flow is one-way — no uplink capability ⏳ Deferred
Uplink is planned for a future version. When the time comes, design should cover: a dedicated MQTT command topic (`bulletgcss/cmd/<callsign>`), an authentication strategy, and a command acknowledgment mechanism. The firmware already defines `MSP_SET_WP` and `msp_set_wp_t`, and `BOXGCSNAV` exists in the MSP enum, so the groundwork is there.

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

### ~~11. Bundled third-party libraries — no package manager~~ ✅ Done
Libraries are committed as minified files directly in the repo. Updating any of them requires manual file replacement, and the current versions are unknown.

**Investigation findings:**
Bootstrap, jQuery, and Popper are present in the repo but **not referenced anywhere in the HTML or JS** — they are dead files and can be deleted. The libraries actually in use are:
- `mqttws31.js` — Paho MQTT (WebSocket client)
- `ol/ol.js` + `css/ol.css` — OpenLayers (map)
- `js/olc.min.js` — Open Location Code
- `js/NoSleep.min.js` — NoSleep (prevents screen lock on mobile)

**Chosen approach: `package.json` as version manifest + `download-libs.sh` shell script**
- `UI/package.json` records the name and pinned version of each library (no `npm install`, no `node_modules`).
- `UI/download-libs.sh` fetches each library from jsdelivr.com at the pinned version and saves it in place. To upgrade: change the version in the script, run it, commit the updated file.
- Bootstrap, jQuery, and Popper were found to be **unused** (not loaded in HTML) and have been deleted from the repo.
- `UI/package.json` records the pinned version of each library in use (ol 6.5.0, nosleep.js 0.12.0, paho-mqtt 1.1.0, open-location-code 1.0.3).
- `UI/download-libs.sh` fetches each library from jsdelivr.com. To upgrade: edit the version variable, run the script, commit.
- Paho MQTT and Open Location Code versions are inferred (no version string in original file headers) — verify before upgrading.

---

## Documentation

### 12. No security warning in `README.md`
The README gives no indication that the default configuration broadcasts the aircraft's real-time GPS location to a public server readable by anyone. This is a meaningful privacy and safety concern for new users.

**What to do:**
- Add a **Security Notice** section to `README.md` explaining the public broker risk and linking to `docs/Self-Hosting-a-MQTT-server--(broker).md`.

---

### ~~13. No troubleshooting section~~ ✅ Done
Added `docs/Troubleshooting.md` covering: Serial Monitor diagnostics, WiFi/cellular/MQTT connection failures, MSP communication, verifying with `mosquitto_sub`, and UI no-data / stale-data scenarios.

---

### ~~14. Multi-aircraft monitoring is undocumented~~ ✅ Done
Added `docs/Multi-aircraft-monitoring.md` clarifying that multi-aircraft monitoring is **not yet supported** (the UI subscribes to one topic at a time), documenting the workaround (multiple browser tabs), and describing the planned feature.

---

## Already Fixed in v1.2

The following issues were identified and resolved during the v1.2 development cycle:

| Issue | Fix |
|---|---|
| MQTT client ID hardcoded as `"ESP32Client"` (collision risk) | Now uses `ESP32_<MAC address>` — unique per device |
| Timer overflow bug in `uint32_t` comparisons | Changed from `timer >= last + interval` to `timer - last >= interval` |
| `currentWPMission` array size was 255 | Fixed to 256 |
| Waypoint loop counter was `uint8_t` | Fixed to `uint16_t` |
| SIM status checked against magic number `3` | Now uses `SIM_READY` constant |
| Copy-paste error messages in telemetry fetch functions | Corrected per function |
| `return false` commented out in `msp_library.cpp` | Uncommented |
| `ont`/`flt` parsed in standard message and low-priority message | Removed from standard message |
| Wiki documentation separate from codebase | Moved to `docs/` |
| Battery planner files from another project in repo | Removed |

---

## Future Features

Larger features that are planned but not yet scoped or scheduled.

### F1. Binary telemetry protocol
Replace the current ASCII key:value format with a compact binary protocol. Goal: reduce bytes per message, add implicit versioning, and make parsing deterministic on both the firmware and UI sides. Design TBD — the protocol format, framing, and versioning strategy need to be decided before implementation.

### F2. Multi-aircraft monitoring
Allow the UI to subscribe to multiple MQTT topics simultaneously and display all aircraft on a single map. Each aircraft would have its own icon on the map. A quick-selector in the data/EFIS panel would let the operator switch which aircraft's telemetry is displayed in detail.

Design considerations:
- UI subscribes to a list of topics (configured in settings).
- `data` object becomes a keyed map of per-aircraft state.
- Map renders all aircraft icons simultaneously; clicking one selects it as the active aircraft for the data panel and EFIS.
- Depends on the ES Modules refactor (item 10) to make per-aircraft state management clean.
