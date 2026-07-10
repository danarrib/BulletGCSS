# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Bullet GCSS** (Ground Control Station System) is a web-based UAV ground control station that operates over cellular networks with no range limit. It is a PWA (no app installation required) that works cross-platform.

## Local Resources

- **INAV firmware repository** is available at `~/dev/inav` — useful for cross-referencing MSP protocol definitions, box IDs, and flight controller behavior.

## Two-Component Architecture

### 1. ESP32-Modem (Embedded Firmware — [ESP32-Modem/](ESP32-Modem/))
Runs on an ESP32 board aboard the UAV. Communicates with the flight controller via the **MSP v2 protocol** (UART, 115200 baud on Serial2: TX18/RX19), then publishes telemetry as key:value messages to an **MQTT broker** via WiFi or a cellular modem (SIM800/SIM7600).

- [ESP32-Modem.cpp](ESP32-Modem/ESP32-Modem.cpp): Main source file. Runs two tasks: `fcTask` (FreeRTOS, priority 2, Core 1 — all MSP, ticks every 160 ms via `TASK_MSP_READ_MS`) and the Arduino loop task (MQTT publish, every 1000 ms). Key startup sequence on FC connection: `msp_get_boxnames()` → `msp_setup_aux_channel()`. Per-cycle (every 160 ms): telemetry round-robin only — `msp_send_aux_rc()` is called only on command state change (not every cycle). Telemetry split into 6 round-robin groups (one per cycle, ~960 ms full refresh). Slow-poll every 10 s: `msp_get_callsign()` + `get_all_waypoints()`.
- [Config.h](ESP32-Modem/Config.h): All user-configurable settings: modem type (`USE_WIFI` vs `TINY_GSM_MODEM_SIM7600`/`SIM800`), WiFi/GPRS credentials, MQTT broker/topic/credentials, serial pin assignments, polling intervals.
- [msp_library.h](ESP32-Modem/msp_library.h) / [msp_library.cpp](ESP32-Modem/msp_library.cpp): MSPv2 protocol implementation (header `$X<`, flags, messageID, payload, CRC8-DVB-S2). Defines all telemetry message types, structs (`modeRangeEntry_t`, `FlightMode`, `msp_set_wp_t`, etc.), and permanent-ID constants (`MSP_PERM_ID_*`). Key MSP codes: `MSP_BOXNAMES` (116), `MSP_MODE_RANGES` (34), `MSP_SET_MODE_RANGE` (35), `MSP_ACTIVEBOXES` (113), `MSP2_INAV_SET_AUX_RC` (0x2230), `MSP2_COMMON_SETTING` (0x1003), `MSP2_COMMON_SET_SETTING` (0x1004). The `requestWithResponse()` method supports requests with a separate send and receive buffer (needed for settings get/set). `FlightMode` struct holds `active` and `boxId` per commandable mode.
- [uav_status.h](ESP32-Modem/uav_status.h): Central `uav_status` struct holding all parsed telemetry (GPS, altitude, speed, battery, heading, flight mode, waypoints, callsign, downlink status, etc.).

**Build:** PlatformIO only (see [docs/Development-setup.md](docs/Development-setup.md)). Two environments: `esp32-sim800` (SIM800 2G) and `esp32-sim7600` (SIM7600 4G). Required libraries — managed automatically via `platformio.ini`: `PubSubClient`, `TinyGSM`, `SSLClient`, `Crypto` (Rhys Weatherley's arduinolibs — Ed25519 signature verification).

**Debug/Testing workflow:** New firmware features are validated by adding `SerialMon.printf()` calls, flashing, and reading the output. Because `pio device monitor` requires an interactive terminal, use Python to read the port directly:

```bash
# Kill any process holding the port, flash, then read output
fuser /dev/ttyUSB1 | xargs -r kill
pio run -e esp32-sim7600 --target upload --upload-port /dev/ttyUSB1

python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
start = time.time()
while time.time() - start < 25:
    line = s.readline()
    if line:
        print(line.decode('utf-8', errors='replace'), end='', flush=True)
s.close()
"
```

Use `grep -E "pattern"` to filter output for specific messages during verification.

### 2. Web UI (Browser PWA — [UI/](UI/))
Static HTML/CSS/JS — no build step. Deployed directly to a web server.

- [basicui.html](UI/basicui.html): Single-page app entry point. Status icon bar (connection, signal, battery, GPS), three views: Data, Map, Sidebar/Settings.
- [js/CommScripts.js](UI/js/CommScripts.js): MQTT over WebSocket (Paho MQTT). Handles connect/subscribe/publish, message replay/logging, localStorage settings persistence. Exposes session recording hooks (`setOnMessageCallback`, `setOnReplayStop`) and replay/restore functions for IndexedDB sessions.
- [js/SessionScripts.js](UI/js/SessionScripts.js): IndexedDB persistence layer for flight sessions. Two-store schema: `sessions` (metadata) and `session_messages` (log lines). Level 0 — no imports from other app modules.
- [js/MapScripts.js](UI/js/MapScripts.js): MapLibre GL JS-based vector map. Renders aircraft icon, real-time position updates, waypoint visualization, flight path, home point, user location. Supports runtime style switching between CARTO Dark Matter and OpenFreeMap Liberty styles (`setMapStyle`). Track/manual pan modes, compass control, center-on-aircraft button. All sizes scaled by `window.devicePixelRatio` for HiDPI displays.
- [js/EfisScripts.js](UI/js/EfisScripts.js): Artificial horizon, heading indicator, altitude/speed gauges — canvas-based EFIS instruments.
- [js/InfoPanelScripts.js](UI/js/InfoPanelScripts.js): Telemetry data panels (battery, GPS, navigation, flight times).
- [js/PageScripts.js](UI/js/PageScripts.js): Viewport, NoSleep, sidebar menu (with Settings submenu), unit conversion preferences (speed/altitude/distance). Async `DOMContentLoaded` initialises IndexedDB, restores open session state, wires session recording and replay-stop callbacks. Also owns the Security panel and the Commands panel — Ed25519 key pair generation (Web Crypto API), key storage in `localStorage`, key status display (5 states), and RC command sending with `rcCommands[]` state tracking. FAB button (`#fabCommands`) provides quick access to the commands panel.
- [js/MissionPlannerScripts.js](UI/js/MissionPlannerScripts.js): Full-screen waypoint mission planner. Owns `plannedMission[]` state and all planner logic. Separate MapLibre GL JS map instance (lazy-initialised on first open). Tap-to-place waypoints with immediate parameter modal (action, altitude, speed, loiter time); drag markers to reposition. RTH enforced as last waypoint. Terrain elevation queried per waypoint. Mission name + unsaved-changes indicator. Save/Load to `localStorage`; Export/Import INAV-compatible JSON. Upload sends waypoints over the signed Ed25519 command channel; blocked if WP Mission mode is active; armed-mid-flight triggers a confirmation dialog. Download (`getmission`) reassembles `dlwp:` messages into the planner. Mission validity dot mirrors `wpv` telemetry.

**Key libraries bundled in repo:** Paho MQTT JS, NoSleep, Open Location Code. MapLibre GL JS is loaded from CDN (referenced in `basicui.html`). The old `UI/ol/` OpenLayers directory has been removed.

**Dev/screenshot injection:** `CommScripts.js` exposes three globals for console and Playwright use:
- `window.gcssInject(input)` — accepts a single payload, a single log line (`timestamp|payload`), or a multi-line block of either format; runs each line through the same parser path as a real MQTT message. Useful for putting the UI into a specific state without a live connection.
- `window._gcssData` — live getter (always reflects the current `data` reference even after `resetDataObject()`) for reading or overriding telemetry values from the console. Example: `_gcssData.extCmdsSupported = 1`.
- `window._gcssMap` — reference to the MapLibre GL JS `map` instance for zoom/pan control. Example: `_gcssMap.setZoom(14)`.

**Deployment:** GitHub Actions ([.github/workflows/](.github/workflows/)) — `rsync` to the server on push, no compilation, just file copy. `master` → production (https://bulletgcss.outros.net), `develop` → staging (https://bulletgcss.outros.net/dev/).

**Local HTTPS dev server:** [scripts/https-dev-server.py](scripts/https-dev-server.py) serves `UI/` over TLS on port 8766, for features that need a secure context (e.g. Web Crypto) and don't work over plain HTTP. Needs a TLS cert/key pair at `scripts/certs/dev-{cert,key}.pem` (gitignored — generate your own per machine, e.g. with `mkcert`; see the script's docstring). When testing from a device other than the one running the server (e.g. a phone or a separate desktop on the same LAN), make sure the cert's SAN list covers the hostname/IP you'll actually browse to, and that hostname resolves from the browsing device (a LAN-local DNS entry or an `/etc/hosts`/Windows-hosts-file override both work). Run with `python3 scripts/https-dev-server.py`.

### 3. Screenshot Automation ([scripts/](scripts/))

Node.js tooling for generating documentation screenshots automatically. Run from the `scripts/` directory:

```bash
cd scripts && npm install   # first time only
node take-screenshots.js
```

**How it works:**
1. Spins up a local HTTP server serving `UI/` on port 8765
2. Launches a headless Chromium browser via Playwright
3. Sets `mqttTopic` to `bulletgcss/telem/screenshot_invalid` via `addInitScript` so the broker connects (no "not connected" banner) but no real telemetry arrives
4. Injects flight log data from `tools/` via `window.gcssInject()` to populate the UI
5. Reconstructs the flight path by extracting `gla`/`glo` fields from log lines and populating `data.currentFlightWaypoints` directly (the `timerOneSecond` that normally builds the path doesn't fire during synchronous injection)
6. Centers the map on the aircraft via the `.maplibregl-ctrl-center` button click
7. Saves PNG files to `docs/screenshots/`

**Screenshot types:**
- **Single screenshot** (`file:` key): straightforward page capture at specified viewport/DPR
- **Composition** (`composition:` key): captures multiple frames in sequence on the same page (e.g. opening menus step by step), optionally draws an orange rounded-rectangle highlight around a specified CSS selector on each frame using an SVG overlay, then stitches all frames side-by-side with a configurable gap using Sharp. Supports `outputWidth` to resize the final image proportionally.

**Key files:**
- [scripts/take-screenshots.js](scripts/take-screenshots.js): main script — screenshot definitions, `injectLogFile()`, `applyHighlight()`, `captureComposition()`
- [scripts/package.json](scripts/package.json): dependencies (`playwright`, `serve-handler`, `sharp`)
- [docs/screenshots/](docs/screenshots/): output directory (committed to repo, referenced by docs)
- [tools/testflight*.txt](tools/): real flight log files used as mock data; format is `timestamp|key:val,key:val,...` one line per MQTT message

### 4. Browser Debugging (Playwright MCP)

Claude Code can drive a real Chrome browser to open and interact with the UI directly (production or a local dev server) instead of reasoning from source alone — useful for visual verification of UI tweaks.

- **Infra**, on the developer's Windows machine: Chrome (`--remote-debugging-port=9222`) + an SSH reverse tunnel (port 3100) + a Playwright MCP SSE server, all started together via a Windows-side script. **Must be running before browser tools work in a Claude Code session.**
- MCP server is registered project-locally for BulletGCSS (`claude mcp add --transport sse playwright http://localhost:3100/sse`, stored under this project's entry in `~/.claude.json`) — already set up, no need to redo.
- **If the Windows-side script gets closed/restarted while a Claude Code session is open, that session's MCP connection does not auto-retry.** Restart the Claude Code session (exit and relaunch) after confirming the Windows script is running again — the reconnect happens at session startup.
- Browser tools appear as deferred tools (`mcp__playwright__browser_navigate`, `browser_snapshot`, `browser_take_screenshot`, `browser_click`, `browser_type`, etc.) — load via ToolSearch before first use.
- **Production URL:** https://bulletgcss.outros.net — deploys automatically on push to `master`. Always shows a simulated flight in progress by design (cron jobs replay a recorded mission on the default MQTT channel every 10 min, staggered across 6 fake callsigns, so first-time visitors see a live demo). Populated telemetry with `Flt time: 0s` / `Mode: N/A` on first load is expected, not a stale-data bug.
- **Staging URL:** https://bulletgcss.outros.net/dev/ — deploys automatically on push to `develop`. Use this to verify in-progress work before it reaches `master`.
- **Local dev URL:** the local HTTPS dev server described under Web UI above — start it with `python3 scripts/https-dev-server.py`, then navigate to `https://<hostname-covered-by-your-cert>:8766/basicui.html`.

## Data Flow

```
Flight Controller (INAV/Betaflight)
  → MSP v2 (Serial2, 115200 baud)
  → ESP32: parse telemetry → uav_status struct → key:value message
  → MQTT Publish (WiFi or cellular)
  → MQTT Broker (default: broker.emqx.io:8883, TLS encrypted)
  → Web UI: MQTT Subscribe over WebSocket
  → Update map, EFIS, info panels
```

## Configuration

All ESP32 settings live in [Config.h](ESP32-Modem/Config.h). To switch between WiFi and GPRS, toggle `#define USE_WIFI`. Two MQTT topics are configured: uplink telemetry (`mqttUplinkTopic`, format `bulletgcss/telem/<callsign>`) and downlink commands (`mqttDownlinkTopic`, format `bulletgcss/cmd/<callsign>`).

Web UI settings (MQTT broker, topic, credentials, units) are persisted in browser `localStorage` — no server-side config file. Flight session data (telemetry log lines and session metadata) are persisted in browser `IndexedDB` via `SessionScripts.js`.

## Important Notes

- The MQTT broker is public by default (`broker.emqx.io`). Production deployments should use a private broker with authentication.
- All downlink commands are signed with Ed25519 (Web Crypto API in the UI, `arduinolibs` `Ed25519::verify()` in firmware). The operator generates a key pair in the UI Security panel, pastes the public key into `Config.h`, and re-flashes. The firmware rejects any command with a missing, invalid, or replayed signature. The last accepted sequence number is persisted to ESP32 NVS so replay protection survives reboots.
- **Aux RC command channel:** Flight mode commands use `MSP2_INAV_SET_AUX_RC` (0x2230). At startup, `msp_setup_aux_channel()` scans all 40 INAV mode condition slots, claims a free aux channel at CH25+, and writes fixed activation ranges via `MSP_SET_MODE_RANGE` — no manual INAV Configurator setup required. On ESP32 watchdog reboot, the existing BulletGCSS channel is detected and reused rather than creating a duplicate. Values persist on the FC indefinitely (no freshness window), so `msp_send_aux_rc()` is called only on command state change, not every cycle. No MSP RC Override flight mode dependency.
- **Table-driven commands:** `cmdModes[]` is a 6-entry table (RTH, AltHold, PosHold, Cruise, WP, Beeper), each a `FlightModeEntry` struct with `startStep`, `endStep`, and `centerPwm`. A single dedicated aux channel is divided into ranges: neutral (1000–1100 µs), RTH (1100–1200), AltHold (1200–1500, wide), PosHold (1300–1400, nested inside AltHold), Cruise (1400–1500, nested inside AltHold), WP (1500–1600), Beeper (1600–1700). Activating PosHold or Cruise automatically co-activates AltHold via range overlap. A FreeRTOS mutex (`cmdMutex`) protects command active states between the MQTT callback and the main task.
- **FC cold-boot:** The firmware waits for the flight controller to become available, probing every 2 seconds. If MSP contact is lost mid-flight (1-second timeout), all startup flags reset and the full init sequence reruns automatically on reconnect.
- The UI is designed mobile-first for outdoor use (dark theme, large touch targets).
- Multiple browser clients can simultaneously monitor the same aircraft by subscribing to the same MQTT topic.
- The PWA caches assets for offline use after first load.

## Git Rules

- **Never commit or push changes unless explicitly asked to do so.**

## Pending Work

Known issues, security concerns, and improvement opportunities are tracked in [docs/TODO.md](docs/TODO.md). Consult this file before starting new features to avoid duplicating effort or making changes that conflict with planned improvements.
