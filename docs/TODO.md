# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities identified during a comprehensive codebase review. Items are grouped by priority.

---

## Security — Critical

### 3. No authentication on the telemetry stream — fake data injection ⚠️ Won't fix / by design
Bullet GCSS is a **receive-only** system — it does not send commands to the aircraft. The worst a bad actor can do by injecting messages is display incorrect telemetry on the UI; they cannot affect the flight. Users who require privacy or data integrity can self-host a private MQTT broker with ACLs (see `docs/Self-Hosting-a-MQTT-server--(broker).md`). The public broker risk is already documented.

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

### F1. Binary telemetry protocol
Replace the current ASCII key:value format with a compact binary protocol. Goal: reduce bytes per message, add implicit versioning, and make parsing deterministic on both the firmware and UI sides. Design TBD — the protocol format, framing, and versioning strategy need to be decided before implementation.

### F2. Multi-aircraft monitoring
Allow the UI to subscribe to multiple MQTT topics simultaneously and display all aircraft on a single map. Each aircraft would have its own icon on the map. A quick-selector in the data/EFIS panel would let the operator switch which aircraft's telemetry is displayed in detail.

Design considerations:
- UI subscribes to a list of topics (configured in settings).
- `data` object becomes a keyed map of per-aircraft state.
- Map renders all aircraft icons simultaneously; clicking one selects it as the active aircraft for the data panel and EFIS.
- Depends on the ES Modules refactor (item 10) to make per-aircraft state management clean.
