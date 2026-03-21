# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities identified during a comprehensive codebase review. Items are grouped by priority.

---

## Security — Critical

### 1. Credentials hardcoded in `Config.h`
`ESP32-Modem/Config.h` contains a real WiFi password and MQTT credentials committed to the public repository. These should be treated as compromised.

**What to do:**
- Rotate the WiFi password and MQTT credentials immediately.
- Convert `Config.h` into a `Config.h.example` template with placeholder values, and add `Config.h` to `.gitignore` so real credentials are never committed again.
- Document this pattern in the setup guide.

---

### 2. `proxy.php` is an open proxy
`UI/proxy.php` line 25 sets `CSAJAX_FILTERS` to `false`, which **completely disables** the domain allowlist in `$valid_requests`. In this state the proxy will forward HTTP requests to any URL on the internet, making the server exploitable for SSRF attacks or as an anonymous proxy.

**What to do:**
- Set `define('CSAJAX_FILTERS', true);` to enable the allowlist.
- Verify `$valid_requests` contains only the necessary domains (`api.open-elevation.com`).
- Remove `bulletgcss.outros.net` from the allowlist if it is no longer needed.

---

### 3. No authentication on the telemetry stream — fake data injection
Anyone who knows (or guesses) the MQTT topic `bulletgcss/uavs/<callsign>` can publish arbitrary messages to it. The UI will parse and display them as real telemetry — spoofed GPS position, false failsafe alerts, fake battery readings.

**What to do:**
- Use a private MQTT broker with per-user ACLs so only the ESP32 can publish and only authorised clients can subscribe.
- Long-term: consider adding a simple HMAC signature to each telemetry message so the UI can verify authenticity.

---

### 4. ESP32 → Broker link is unencrypted (port 1883, no TLS)
The modem publishes telemetry in plaintext over port 1883. Aircraft GPS position and status are visible to anyone with access to the cellular carrier network or the broker.

**What to do:**
- Switch the ESP32 MQTT connection to TLS (port 8883). `TinyGSM` and `PubSubClient` support TLS with a root CA certificate.
- Update `Config.h` with the new port and add instructions to the setup guide.

---

## Security — Medium

### 5. Public broker by default — no privacy
The default broker `broker.emqx.io` is fully public. All telemetry is readable by anyone who subscribes to the same topic. There is no access control.

**What to do:**
- Add a prominent security warning to `README.md` and `docs/Find-a-MQTT-Broker.md` explaining the privacy implications.
- Encourage users to follow `docs/Self-Hosting-a-MQTT-server--(broker).md` for production use.

---

### 6. No input validation on the telemetry parser
`parseStandardTelemetryMessage()` in `UI/js/CommScripts.js` calls `parseInt`/`parseFloat` directly on untrusted values with no bounds checking. A malformed or adversarial MQTT message could result in nonsensical display values (negative satellite counts, impossible altitudes, etc.).

**What to do:**
- Add range checks for critical fields (GPS coordinates, altitude, battery voltage, satellite count).
- Silently clamp or discard out-of-range values rather than displaying them.

---

## Architecture

### 7. Custom CSV telemetry format — consider migrating to JSON
The current format (`ran:1234,pan:567,...`) works but has no schema, no versioning, and no tooling support. Every new field requires changes in both firmware and the UI parser, and debugging requires reading both sides simultaneously.

**What to do:**
- Evaluate migrating to JSON (`{"ran":1234,"pan":567,...}`). The ESP32 and `PubSubClient` support it natively, and the UI can use `JSON.parse()` instead of the manual split/switch parser.
- If staying with the current format, document a versioning strategy so the UI can handle messages from older firmware gracefully.
- The protocol is already documented in `docs/BulletGCSS_protocol.md` — add a version field to messages as a first step.

---

### 8. MQTT QoS 0 — no delivery guarantee
All messages are published and subscribed at QoS 0 (fire-and-forget). Telemetry packets can be silently dropped with no feedback to the operator.

**What to do:**
- Evaluate using QoS 1 for critical messages (failsafe state, armed status). Note this increases broker load and requires `PubSubClient` to be configured accordingly.
- At minimum, document this limitation clearly so operators understand that a stale UI does not necessarily mean the aircraft is safe.

---

### 9. Data flow is one-way — no uplink capability
The system is purely read-only. The firmware already defines `MSP_SET_WP` and `msp_set_wp_t`, suggesting uplink was considered. The `BOXGCSNAV` flight mode box also exists in the MSP enum, hinting at GCS navigation support in INAV.

**What to do:**
- Decide whether uplink (e.g. uploading a waypoint mission from the UI) is in scope.
- If yes, design an MQTT command topic (`bulletgcss/cmd/<callsign>`), authentication strategy, and command acknowledgment mechanism before implementing.

---

### 10. Global mutable state in the UI JavaScript
All JS files share a single global `data` object and dozens of global variables. This works at the current scale but makes the code brittle and hard to extend.

**What to do:**
- No immediate action needed, but keep this in mind when adding features.
- When refactoring, consider encapsulating state in a module or a simple class to reduce the risk of naming collisions between scripts.

---

### 11. Bundled third-party libraries — no package manager
OpenLayers, Bootstrap, jQuery, Paho MQTT, NoSleep, and Popper are all committed as minified files directly in the repo. Updating any of them requires manual file replacement.

**What to do:**
- Consider introducing a simple build step (e.g. `npm` + a bundler, or just a `package.json` with download scripts) so library versions are declared and upgradable.
- At minimum, document the version of each bundled library in a `THIRD_PARTY.md` file so outdated dependencies are easy to spot.

---

## Documentation

### 12. No security warning in `README.md`
The README gives no indication that the default configuration broadcasts the aircraft's real-time GPS location to a public server readable by anyone. This is a meaningful privacy and safety concern for new users.

**What to do:**
- Add a **Security Notice** section to `README.md` explaining the public broker risk and linking to `docs/Self-Hosting-a-MQTT-server--(broker).md`.

---

### 13. No troubleshooting section
There is no guidance for when things go wrong — ESP32 failing to connect to MQTT, MSP communication not working, UI showing stale data, etc.

**What to do:**
- Add a `docs/Troubleshooting.md` covering at minimum:
  - How to read Serial Monitor output to diagnose firmware issues.
  - How to use `mosquitto_sub` to verify the ESP32 is publishing.
  - What to check when the UI shows no data.
  - Common cellular modem initialization failures.

---

### 14. Multi-aircraft monitoring is undocumented
The README mentions multiple clients can monitor the same aircraft, but monitoring multiple aircraft simultaneously is also possible and undocumented.

**What to do:**
- Document how multiple aircraft can be monitored (each with a unique callsign/topic) and any current UI limitations around this use case.

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
